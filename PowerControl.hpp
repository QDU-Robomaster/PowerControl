#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_name: PowerControl
module_description: Power control for chassis (supports omni and helm wheel)
constructor_args:
  - superpower: '@&super_power'
  - is_helm: 'false'
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "RLS.hpp"
#include "SuperPower.hpp"
#include "app_framework.hpp"
#include "matrix.h"
#include "message.hpp"
#include "thread.hpp"

// 计算单个电机功率: P = kt*I*rpm + k1*I^2 + k2*rpm^2 + k3_share
inline float calculate_motor_power(float current, float rpm,
                                   float kt, float k1, float k2, float k3_share) {
  return kt * current * rpm +
         k1 * current * current +
         k2 * rpm * rpm +
         k3_share;
}

// 根据目标功率反解电流 (求解二次方程 k1*I^2 + kt*rpm*I + (k2*rpm^2 + k3 - P) = 0)
inline float solve_current_for_power(float target_power, float rpm,
                                     float kt, float k1, float k2, float k3_share,
                                     float original_current) {
  float a = k1;
  float b = kt * rpm;
  float c = k2 * rpm * rpm + k3_share - target_power;
  float delta = b * b - 4.0f * a * c;

  if (delta < 0.0f) {
    return std::clamp(-b / (2.0f * a), -16384.0f, 16384.0f);
  } else {
    if (original_current > 0.0f) {
      return std::clamp((-b + sqrtf(delta)) / (2.0f * a), -16384.0f, 16384.0f);
    } else {
      return std::clamp((-b - sqrtf(delta)) / (2.0f * a), -16384.0f, 16384.0f);
    }
  }
}

// 计算误差置信度
inline float calculate_error_confidence(float sum_error,
                                        float threshold_low, float threshold_high) {
  if (sum_error > threshold_high) {
    return 1.0f;
  }
  if (sum_error > threshold_low) {
    return (sum_error - threshold_low) / (threshold_high - threshold_low);
  }
  return 0.0f;
}
struct PowerControlData {
  float new_output_current_3508[4] = {};
  float new_output_current_6020[4] = {};
  bool is_power_limited = false;
};

class PowerControl : public LibXR::Application {
 public:
  PowerControl(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               SuperPower *superpower, bool is_helm = false,
               float chassis_static_power_loss = 4.0f)
      : superpower_(superpower),
        is_helm_(is_helm),
        rls_(1e-5f, 0.99999f),
        k3_chassis_(chassis_static_power_loss) {
    UNUSED(hw);
    UNUSED(app);
    params_3508_[0][0] = 2.0e-7f;
    params_3508_[1][0] = 2.0e-7f;
    k1_3508_ = params_3508_[0][0];
    k2_3508_ = params_3508_[1][0];
  }

  void SetMotorData3508(float output_current[4], float rotorspeed_rpm[4],
                        float target_omega[4], float current_omega[4]) {
    for (int i = 0; i < 4; i++) {
      output_current_3508_[i] = output_current[i];
      rotorspeed_rpm_3508_[i] = rotorspeed_rpm[i];
      target_omega_3508_[i] = target_omega[i];
      current_omega_3508_[i] = current_omega[i];
    }
  }

  void SetMotorData6020(float output_current[4], float rotorspeed_rpm[4],
                        float target_omega[4], float current_omega[4]) {
    for (int i = 0; i < 4; i++) {
      output_current_6020_[i] = output_current[i];
      rotorspeed_rpm_6020_[i] = rotorspeed_rpm[i];
      target_omega_6020_[i] = target_omega[i];
      current_omega_6020_[i] = current_omega[i];
    }
  }

  void CalculatePowerControlParam() {
    if (is_helm_) {
      CalculatePowerControlParamHelm();
    } else {
      CalculatePowerControlParamOmni();
    }
  }

  void OutputLimit(float max_power) {
    if (is_helm_) {
      OutputLimitHelm(max_power);
    } else {
      OutputLimitOmni(max_power);
    }
  }

  PowerControlData GetPowerControlData() {
    PowerControlData data;
    mutex_.Lock();
    data = powercontrol_data_;
    mutex_.Unlock();
    return data;
  }

  void OnMonitor() override {}

 private:
  //全向轮or麦轮 功率参数估计

  void CalculatePowerControlParamOmni() {
    measured_power_ = superpower_->GetChassisPower();

    machane_power_3508_ = 0.0f;
    samples_3508_[0][0] = 0.0f;
    samples_3508_[1][0] = 0.0f;

    for (int i = 0; i < 4; i++) {
      machane_power_3508_ += kt_3508_ * output_current_3508_[i] * rotorspeed_rpm_3508_[i];
      samples_3508_[0][0] += output_current_3508_[i] * output_current_3508_[i];
      samples_3508_[1][0] += rotorspeed_rpm_3508_[i] * rotorspeed_rpm_3508_[i];
    }

    params_3508_ = rls_.Update(samples_3508_, measured_power_ - machane_power_3508_ - k3_chassis_);

    k1_3508_ = static_cast<float>(fmax(params_3508_[0][0], 2.0e-7f));
    k2_3508_ = static_cast<float>(fmax(params_3508_[1][0], 2.0e-7f));

    estimated_power_3508_ = machane_power_3508_ +
                            k1_3508_ * samples_3508_[0][0] +
                            k2_3508_ * samples_3508_[1][0] +
                            k3_chassis_;
  }

  //舵轮 功率参数估计

  void CalculatePowerControlParamHelm() {
    measured_power_ = superpower_->GetChassisPower();

    // 3508参数估计
    machane_power_3508_ = 0.0f;
    samples_3508_[0][0] = 0.0f;
    samples_3508_[1][0] = 0.0f;

    for (int i = 0; i < 4; i++) {
      machane_power_3508_ += kt_3508_ * output_current_3508_[i] * rotorspeed_rpm_3508_[i];
      samples_3508_[0][0] += output_current_3508_[i] * output_current_3508_[i];
      samples_3508_[1][0] += rotorspeed_rpm_3508_[i] * rotorspeed_rpm_3508_[i];
    }

    // 6020功率计算 (使用固定参数)
    machane_power_6020_ = 0.0f;
    samples_6020_[0][0] = 0.0f;
    samples_6020_[1][0] = 0.0f;

    for (int i = 0; i < 4; i++) {
      machane_power_6020_ += kt_6020_ * output_current_6020_[i] * rotorspeed_rpm_6020_[i];
      samples_6020_[0][0] += output_current_6020_[i] * output_current_6020_[i];
      samples_6020_[1][0] += rotorspeed_rpm_6020_[i] * rotorspeed_rpm_6020_[i];
    }

    // 只拟合3508参数
    if (measured_power_ > 5.0f) {
      float residual = measured_power_ - machane_power_3508_ -
                       machane_power_6020_ - k3_chassis_;
      params_3508_ = rls_.Update(samples_3508_, residual);
    }

    k1_3508_ = static_cast<float>(fmax(params_3508_[0][0], 2.0e-7f));
    k2_3508_ = static_cast<float>(fmax(params_3508_[1][0], 2.0e-7f));

    estimated_power_3508_ = machane_power_3508_ +
                            k1_3508_ * samples_3508_[0][0] +
                            k2_3508_ * samples_3508_[1][0] + k3_chassis_;

    estimated_power_6020_ = machane_power_6020_ +
                            k1_6020_ * samples_6020_[0][0] +
                            k2_6020_ * samples_6020_[1][0] + k3_chassis_;
  }

  //全向轮or麦轮 功率限制

  void OutputLimitOmni(float max_power) {
    float chassis_power = 0.0f;
    float sum_error = 0.0f;
    float required_power = 0.0f;
    float allocated_power = max_power;

    for (int i = 0; i < 4; i++) {
      motor_power_3508_[i] = calculate_motor_power(output_current_3508_[i], rotorspeed_rpm_3508_[i],
                              kt_3508_, k1_3508_, k2_3508_, k3_chassis_ / 4.0f);

      chassis_power += motor_power_3508_[i];
      error_3508_[i] = fabs(target_omega_3508_[i] - current_omega_3508_[i]);

      if (motor_power_3508_[i] < 0) {
        allocated_power -= motor_power_3508_[i];
      } else {
        sum_error += error_3508_[i];
        required_power += motor_power_3508_[i];
      }
    }
    if (chassis_power > max_power) {
      powercontrol_data_.is_power_limited = true;

      float error_conf = calculate_error_confidence(
          sum_error, error_threshold_low_, error_threshold_high_);

      for (int i = 0; i < 4; i++) {
        if (motor_power_3508_[i] < 0) {
          powercontrol_data_.new_output_current_3508[i] = output_current_3508_[i];
          scaled_motor_power_3508_[i] = motor_power_3508_[i];
          continue;
        }

        float weight_error = error_3508_[i] / sum_error;
        float weight_prop = motor_power_3508_[i] / required_power;
        float weight = error_conf * weight_error + (1.0f - error_conf) * weight_prop;

        powercontrol_data_.new_output_current_3508[i] = solve_current_for_power(
            allocated_power * weight, rotorspeed_rpm_3508_[i], kt_3508_,
            k1_3508_, k2_3508_, k3_chassis_ / 4.0f, output_current_3508_[i]);

        scaled_motor_power_3508_[i] =
            calculate_motor_power(powercontrol_data_.new_output_current_3508[i],
                                rotorspeed_rpm_3508_[i], kt_3508_, k1_3508_,
                                k2_3508_, k3_chassis_ / 4.0f);
      }

      for (int i = 0; i < 4; i++) {
        output_current_3508_[i] = powercontrol_data_.new_output_current_3508[i];
      }
    } else {
      powercontrol_data_.is_power_limited = false;
      for (int i = 0; i < 4; i++) {
        powercontrol_data_.new_output_current_3508[i] = output_current_3508_[i];
      }
    }
  }

  //舵轮 功率限制
  void OutputLimitHelm(float max_power) {
    float chassis_power = 0.0f;
    float allocated_power = max_power;
    float required_power_3508 = 0.0f;
    float required_power_6020 = 0.0f;
    float sum_error_3508 = 0.0f;

    // 计算各电机功率
    for (int i = 0; i < 4; i++) {
      motor_power_3508_[i] =
          calculate_motor_power(output_current_3508_[i], rotorspeed_rpm_3508_[i],
                              kt_3508_, k1_3508_, k2_3508_, k3_chassis_ / 8.0f);

      motor_power_6020_[i] =
          calculate_motor_power(output_current_6020_[i], rotorspeed_rpm_6020_[i],
                              kt_6020_, k1_6020_, k2_6020_, k3_chassis_ / 8.0f);
      chassis_power += motor_power_3508_[i] + motor_power_6020_[i];
      error_3508_[i] = fabs(target_omega_3508_[i] - current_omega_3508_[i]);

      if (motor_power_3508_[i] > 0) {
        required_power_3508 += motor_power_3508_[i];
        sum_error_3508 += error_3508_[i];
      } else {
        allocated_power -= motor_power_3508_[i];
      }

      if (motor_power_6020_[i] > 0) {
        required_power_6020 += motor_power_6020_[i];
      } else {
        allocated_power -= motor_power_6020_[i];
      }
    }

    // 超功率时进行限制
    if (chassis_power > max_power) {
      powercontrol_data_.is_power_limited = true;

      // 6020优先分配 (保证转向优先级)
      float alloc_6020 = 0.0f;
      float alloc_3508 = 0.0f;
      if (required_power_6020 > 0) {
        alloc_6020 = allocated_power * power_ratio_6020_;
        alloc_3508 = allocated_power * (1.0f - power_ratio_6020_);
      } else {
        alloc_6020 = 0.0f;
        alloc_3508 = allocated_power;
      }

      // 限制6020 (均分功率)
      for (int i = 0; i < 4; i++) {
        if (motor_power_6020_[i] < 0) {
          powercontrol_data_.new_output_current_6020[i] =
              output_current_6020_[i];
          scaled_motor_power_6020_[i] = motor_power_6020_[i];
        } else {
          powercontrol_data_.new_output_current_6020[i] = solve_current_for_power(
              alloc_6020 / 4.0f, rotorspeed_rpm_6020_[i], kt_6020_, k1_6020_,
              k2_6020_, k3_chassis_ / 8.0f, output_current_6020_[i]);

          scaled_motor_power_6020_[i] =
              calculate_motor_power(powercontrol_data_.new_output_current_6020[i],
                                  rotorspeed_rpm_6020_[i], kt_6020_, k1_6020_,
                                  k2_6020_, k3_chassis_ / 8.0f);
        }
      }

      // 限制3508 (按误差/比例混合分配)
      float error_conf = calculate_error_confidence(
          sum_error_3508, error_threshold_low_, error_threshold_high_);

      for (int i = 0; i < 4; i++) {
        if (motor_power_3508_[i] < 0) {
          powercontrol_data_.new_output_current_3508[i] =
              output_current_3508_[i];
          scaled_motor_power_3508_[i] = motor_power_3508_[i];
          continue;
        }

        float weight_error = error_3508_[i] / sum_error_3508;
        float weight_prop = motor_power_3508_[i] / required_power_3508;
        float weight =
            error_conf * weight_error + (1.0f - error_conf) * weight_prop;

        powercontrol_data_.new_output_current_3508[i] = solve_current_for_power(
            alloc_3508 * weight, rotorspeed_rpm_3508_[i], kt_3508_, k1_3508_,
            k2_3508_, k3_chassis_ / 8.0f, output_current_3508_[i]);

        scaled_motor_power_3508_[i] =
            calculate_motor_power(powercontrol_data_.new_output_current_3508[i],
                                rotorspeed_rpm_3508_[i], kt_3508_, k1_3508_,
                                k2_3508_, k3_chassis_ / 8.0f);
      }

      // 更新总功率
      chassis_power_ = 0.0f;
      for (int i = 0; i < 4; i++) {
        chassis_power_ +=
            scaled_motor_power_3508_[i] + scaled_motor_power_6020_[i];
      }
    } else {
      powercontrol_data_.is_power_limited = false;
      for (int i = 0; i < 4; i++) {
        powercontrol_data_.new_output_current_3508[i] = output_current_3508_[i];
        powercontrol_data_.new_output_current_6020[i] = output_current_6020_[i];
      }
      chassis_power_ = chassis_power;
    }
  }

 private:
  LibXR::Mutex mutex_;
  SuperPower *superpower_;
  bool is_helm_;
  RLS<2> rls_;
  PowerControlData powercontrol_data_;

  //3508电机参数
  float kt_3508_ = 1.99688994e-6f;
  float k1_3508_ = 0.0f;
  float k2_3508_ = 0.0f;

  Matrixf<2, 1> samples_3508_;
  Matrixf<2, 1> params_3508_;
  float machane_power_3508_ = 0.0f;
  float estimated_power_3508_ = 0.0f;
  float motor_power_3508_[4] = {};
  float scaled_motor_power_3508_[4] = {};
  float error_3508_[4] = {};

  float output_current_3508_[4] = {};
  float rotorspeed_rpm_3508_[4] = {};
  float target_omega_3508_[4] = {};
  float current_omega_3508_[4] = {};

  // 6020电机参数 (舵轮专用)
  float kt_6020_ = 1.42074505e-5f;
  float k1_6020_ = 1.0e-6f;
  float k2_6020_ = 1.0e-9f;

  Matrixf<2, 1> samples_6020_;
  float machane_power_6020_ = 0.0f;
  float estimated_power_6020_ = 0.0f;
  float motor_power_6020_[4] = {};
  float scaled_motor_power_6020_[4] = {};

  float output_current_6020_[4] = {};
  float rotorspeed_rpm_6020_[4] = {};
  float target_omega_6020_[4] = {};
  float current_omega_6020_[4] = {};

  // 通用参数
  float measured_power_ = 0.0f;
  float chassis_power_ = 0.0f;
  //底盘功率的静态损耗
  float k3_chassis_ =4.5f;

  // 功率分配参数
  float power_ratio_6020_ = 0.8f;
  float error_threshold_high_ = 120.0f;
  float error_threshold_low_ = 60.0f;
};
