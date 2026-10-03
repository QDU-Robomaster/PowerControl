# PowerControl

底盘功率控制模块：按功率上限重新分配各电机的输出电流，支持全向轮与舵轮底盘 / Chassis power control Module that redistributes the motor output currents under a power limit, for omni-wheel and steering-wheel chassis

## 1. 模块作用 / Purpose

PowerControl 支持全向轮底盘（仅 3508）和舵轮底盘（3508 驱动加 6020 转向）。它由底盘在自己的控制循环中调用，所有公共接口内部由同一把互斥锁保护。

每个控制周期，底盘按下面顺序调用：

1. `SetMotorData3508(output_current, rotorspeed_rpm, speed_error = nullptr)`，舵轮再调用 `SetMotorData6020(...)`：写入各电机本周期的期望电流、转子转速（rpm）和可选的速度跟踪误差（取绝对值）。数组长度为构造时的电机数量。
2. `CalculatePowerControlParam()`：从 `SuperPower::GetChassisPower()` 读取实测底盘功率，扣除 3508 机械功率、底盘静态功耗（舵轮还扣除 6020 模型功率）后，用递归最小二乘（RLS，遗忘因子 0.99999）在线辨识 3508 功率模型 `P = kt·I·ω + k1·I² + k2·ω²` 中的 `k1`、`k2`。仅在超级电容在线、实测功率大于 5 W 且残差为正时更新，`k1`、`k2` 不小于 `1e-7`。
3. `OutputLimit(max_power)`：可用功率为 `max_power - chassis_static_power_loss - 4 W` 余量。模型预测的总需求功率超出可用功率时，按权重给每个电机分配功率配额，再解二次方程反算电流，限幅 ±16384；未超出时原样输出。权重在“按速度误差分配”和“按功率需求比例分配”之间混合：误差总和大于 20 时全部按误差，15 至 20 之间线性过渡，小于 15 时按需求比例。负功率（回馈）电机退出分配，其功率计入可用功率（全向轮）。
   - 全向轮：限制 3508。
   - 舵轮：6020 组最多占可用功率的 80%，剩余给 3508 组；两组内部各自按上述权重分配。
4. `GetPowerControlData()`：取回 `PowerControlData`，包含 `new_output_current_3508[]`、`new_output_current_6020[]` 和 `is_power_limited`。

其他接口：

- `SetAllocationBias3508(const AllocationBias3508&)`：全向轮路径使用。启用后先把可用功率的 `reserve_fraction` 作为保底池，按 `reserve_weight[]` 分给正功率电机（不超过其需求），剩余功率再按上述权重乘以 `allocation_weight_scale[]`（未配置或不大于 0 时为 1）分配。
- `GetMeasuredPower()`：最近一次 `CalculatePowerControlParam()` 读到的实测功率。
- `GetCapEnergy()`、`IsOnline()`：转发 `SuperPower` 的同名接口。

电机数量上限为 `PowerControl::MAX_MOTOR_COUNT`（6），超出部分被截断。3508 的 `kt` 与 6020 的 `kt`、`k1`、`k2` 是源码中的固定常数。

PowerControl supports omni-wheel chassis (3508 only) and steering-wheel chassis (3508 drive plus 6020 steering). The chassis calls it in its own control loop, and all public interfaces are protected internally by the same mutex.

Every control cycle, the chassis calls in the following order:

1. `SetMotorData3508(output_current, rotorspeed_rpm, speed_error = nullptr)`, and for the steering-wheel chassis also `SetMotorData6020(...)`: write the desired current, rotor speed (rpm) and optional speed tracking error (absolute value) of each motor for this cycle. The array length is the motor count given at construction.
2. `CalculatePowerControlParam()`: read the measured chassis power from `SuperPower::GetChassisPower()`, subtract the 3508 mechanical power and the chassis static power loss (and for the steering-wheel chassis the 6020 model power), and identify `k1` and `k2` of the 3508 power model `P = kt·I·ω + k1·I² + k2·ω²` online with recursive least squares (RLS, forgetting factor 0.99999). The update runs only when the supercapacitor is online, the measured power exceeds 5 W and the residual is positive, and `k1` and `k2` are at least `1e-7`.
3. `OutputLimit(max_power)`: the available power is `max_power - chassis_static_power_loss` minus a 4 W margin. When the total demanded power predicted by the model exceeds the available power, a power quota is assigned to each motor by weight and the current is solved back from a quadratic equation, clamped to ±16384; otherwise the currents are output unchanged. The weight mixes "by speed error" and "by power demand ratio": entirely by error when the error sum is above 20, a linear transition between 15 and 20, and by demand ratio below 15. Motors with negative (regenerative) power leave the allocation and their power is added to the available power (omni-wheel chassis).
   - Omni-wheel chassis: limits the 3508.
   - Steering-wheel chassis: the 6020 group takes at most 80% of the available power and the rest goes to the 3508 group; inside each group the allocation uses the weights above.
4. `GetPowerControlData()`: fetch `PowerControlData`, which contains `new_output_current_3508[]`, `new_output_current_6020[]` and `is_power_limited`.

Other interfaces:

- `SetAllocationBias3508(const AllocationBias3508&)`: used by the omni-wheel path. When enabled, `reserve_fraction` of the available power forms a reserve pool, which is distributed to the motors with positive power by `reserve_weight[]` (not exceeding their demand), and the remaining power is distributed with the weights above multiplied by `allocation_weight_scale[]` (1 when unset or not greater than 0).
- `GetMeasuredPower()`: the measured power read by the latest `CalculatePowerControlParam()`.
- `GetCapEnergy()`, `IsOnline()`: forward the same-named interfaces of `SuperPower`.

The motor count is limited to `PowerControl::MAX_MOTOR_COUNT` (6), and larger values are truncated. The `kt` of the 3508 and the `kt`, `k1` and `k2` of the 6020 are fixed constants in the source.

## 2. 构造接口 / Constructor

```cpp
PowerControl(SuperPower& super_power,
             bool is_helm = false,
             float chassis_static_power_loss = 0.0f,
             int motor_count_3508 = 4,
             int motor_count_6020 = 4);
```

依赖：

- `super_power`：`SuperPower` 实例，提供实测底盘功率和超级电容在线状态。

配置参数：

- `is_helm`：`true` 为舵轮底盘，同时限制 6020；`false` 为全向轮底盘，默认 `false`。
- `chassis_static_power_loss`：底盘静态功耗，单位 W，从可用功率中扣除，默认 `0.0`。
- `motor_count_3508`：3508 电机数量，默认 `4`，最大 6。
- `motor_count_6020`：6020 电机数量，默认 `4`，最大 6。

Dependencies:

- `super_power`: the `SuperPower` instance that provides the measured chassis power and the supercapacitor online state.

Configuration parameters:

- `is_helm`: `true` for a steering-wheel chassis, which also limits the 6020; `false` for an omni-wheel chassis, default `false`.
- `chassis_static_power_loss`: chassis static power loss in W, deducted from the available power, default `0.0`.
- `motor_count_3508`: number of 3508 motors, default `4`, at most 6.
- `motor_count_6020`: number of 6020 motors, default `4`, at most 6.

## 3. Topic

无 / None

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/PowerControl` 写入的实例，`super_power` 填写为 `QDU-Robomaster/SuperPower` 实例的 id，该实例在 `modules:` 中排在本实例之前。PowerControl 实例通常再作为依赖传给底盘模块，例如 `QDU-Robomaster/Chassis`：

An instance written by `xrobot instance add QDU-Robomaster/PowerControl`, with `super_power` set to the id of the `QDU-Robomaster/SuperPower` instance, which is listed before this instance in `modules:`. The PowerControl instance is usually passed on as a dependency to a chassis Module, for example `QDU-Robomaster/Chassis`:

```yaml
modules:
  - module: QDU-Robomaster/PowerControl
    id: power_control
    args:
      - super_power: superpower
      - is_helm: false
      - chassis_static_power_loss: 4.5
      - motor_count_3508: 4
      - motor_count_6020: 0
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/SuperPower`：提供实测底盘功率和超级电容在线状态。
- Eigen（`Eigen/Core`，由 LibXR 自带）：`RLS.hpp` 中的递归最小二乘估计器。
- LibXR。

硬件：全向轮底盘的 3508 电机，或舵轮底盘的 3508 驱动电机与 6020 转向电机；实测功率来自 `SuperPower`。

Dependencies:

- `QDU-Robomaster/SuperPower`: provides the measured chassis power and the supercapacitor online state.
- Eigen (`Eigen/Core`, shipped with LibXR): the recursive least squares estimator in `RLS.hpp`.
- LibXR.

Hardware: the 3508 motors of an omni-wheel chassis, or the 3508 drive motors and 6020 steering motors of a steering-wheel chassis; the measured power comes from `SuperPower`.
