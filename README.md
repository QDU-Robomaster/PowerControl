# PowerControl

底盘功率控制：按功率上限重新分配各电机的输出电流，支持全向轮（仅 3508）和舵轮
（3508 驱动 + 6020 转向）两种底盘。

模块没有线程，由底盘在自己的控制循环中调用。所有公共接口内部用同一把互斥锁保护。

## 工作方式

每个控制周期，底盘按下面顺序调用：

1. `SetMotorData3508(output_current, rotorspeed_rpm, speed_error = nullptr)`，舵轮再调用
   `SetMotorData6020(...)`：写入各电机本周期的期望电流、转子转速（rpm）和可选的速度跟踪误差
   （取绝对值）。数组长度为构造时的电机数量。
2. `CalculatePowerControlParam()`：从 `SuperPower::GetChassisPower()` 读取实测底盘功率，扣除
   3508 机械功率、底盘静态功耗（舵轮还扣除 6020 模型功率）后，用递归最小二乘（RLS，遗忘因子
   0.99999）在线辨识 3508 功率模型 `P = kt·I·ω + k1·I² + k2·ω²` 中的 `k1`、`k2`。只在超级电容
   在线、实测功率大于 5 W 且残差为正时更新；`k1`、`k2` 不小于 `1e-7`。
3. `OutputLimit(max_power)`：可用功率为 `max_power - chassis_static_power_loss - 4 W` 余量。
   模型预测的总需求功率超出可用功率时，按权重给每个电机分配功率配额，再解二次方程反算电流
   （限幅 ±16384）；未超出时原样输出。权重在“按速度误差分配”和“按功率需求比例分配”之间
   混合：误差总和大于 20 时完全按误差，15~20 之间线性过渡，小于 15 时按需求比例。
   负功率（回馈）电机不参与分配，其功率计入可用功率（全向轮）。
   - 全向轮：只限制 3508。
   - 舵轮：6020 组最多占可用功率的 80%，剩余给 3508 组；两组内部各自按上述权重分配。
4. `GetPowerControlData()`：取回 `PowerControlData`，包含 `new_output_current_3508[]`、
   `new_output_current_6020[]` 和 `is_power_limited`。

其他接口：

- `SetAllocationBias3508(const AllocationBias3508&)`：仅全向轮路径使用。启用后先把可用功率的
  `reserve_fraction` 作为保底池，按 `reserve_weight[]` 分给正功率电机（不超过其需求），剩余功率
  再按上述权重乘以 `allocation_weight_scale[]`（未配置或 ≤0 时为 1）分配。
- `GetMeasuredPower()`：最近一次 `CalculatePowerControlParam()` 读到的实测功率。
- `GetCapEnergy()`、`IsOnline()`：转发 `SuperPower` 的同名接口。

电机数量上限为 `PowerControl::MAX_MOTOR_COUNT`（6），超出部分被截断。3508 的 `kt` 和 6020 的
`kt`、`k1`、`k2` 是源码中的固定常数。

## 依赖

- `QDU-Robomaster/SuperPower`：提供实测底盘功率和超级电容在线状态。

外部依赖：Eigen（`Eigen/Core`，由 LibXR 自带），用于 `RLS.hpp` 中的递归最小二乘估计器。

## 构造接口

```cpp
PowerControl(SuperPower& super_power,
             bool is_helm = false,
             float chassis_static_power_loss = 0.0f,
             int motor_count_3508 = 4,
             int motor_count_6020 = 4);
```

依赖项：

- `super_power`：`SuperPower` 实例。

配置：

- `is_helm`：`true` 为舵轮底盘（同时限制 6020），`false` 为全向轮底盘，默认 `false`。
- `chassis_static_power_loss`：底盘静态功耗，单位 W，从可用功率中扣除，默认 `0.0`。
- `motor_count_3508`：3508 电机数量，默认 `4`，最大 6。
- `motor_count_6020`：6020 电机数量，默认 `4`，最大 6。

## 使用

```sh
xrobot module add QDU-Robomaster/PowerControl
xrobot setup
xrobot instance add QDU-Robomaster/PowerControl
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `super_power` 填为 SuperPower 实例的 `id`：

```yaml
modules:
  - module: QDU-Robomaster/PowerControl
    id: powercontrol_0
    args:
      - super_power: superpower
      - is_helm: 'false'
      - chassis_static_power_loss: 0.0f
      - motor_count_3508: '4'
      - motor_count_6020: '4'
```

`superpower` 是 `QDU-Robomaster/SuperPower` 实例的 `id`，必须在 `modules:` 中排在本实例之前。
本模块不直接使用 BSP 对象，不需要额外的 `XR_REGISTER`。PowerControl 实例通常再作为依赖传给
底盘模块（如 `QDU-Robomaster/Chassis`）。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/PowerControl`
（在 BSP 中）打印当前的构造函数。
