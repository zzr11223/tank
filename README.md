# STM32F103ZET6 履带车控制固件

本工程是基于 STM32 HAL 的双履带车控制固件。当前控制链由六通道 PWM 遥控输入、双路电调 PWM、四路 AB 相编码器和四路低电平触发红外输入组成；串口 IMU 解析和姿态遥测仍保留，但 IMU 已与运动控制隔离。

> 当前状态（2026-08-27）：第零阶段红外急停锁存和第一阶段 IMU 控制隔离已经写入源码；代码层面不包含遥控失联超时保护。烧录前必须悬空履带并准备硬件断电手段。

## 1. 工程概览

已确认的主要功能：

- CH1～CH6 独立 PWM 输入：GPIO 双边沿 EXTI 配合 TIM5 1 MHz 自由运行计数器测量脉宽。
- 遥控输入采用 7 点中值滤波，有效范围为 1000～2100 us。
- CH3 控制前进/后退，CH1 控制转向；当前启用转向优先，转向时两侧反向实现原地转向。
- TIM1 在 PE9、PE11 输出两路 50 Hz 电调 PWM，软件中 1500 us 为停止点。
- TIM2、TIM3、TIM4、TIM8 以正交编码器 x4 模式采集四路 AB 相编码器。
- 直行时根据左右履带编码器速度差执行 PI 同步修正。
- 任一路红外触发后立即输出 1500 us 并锁存；障碍解除后，CH1、CH3 必须先同时回中一次，随后重新推杆才能运动。
- USART1 输出遥控和实际 PWM 遥测，也可接收电调测试命令。
- USART2 接收串口九轴 IMU；USART3只输出姿态角。当前 IMU 数据不参与履带控制。

未实现或当前禁用：

- 没有遥控器上电锁存。
- 没有 CH1/CH3 丢帧超时停车；最后一次合法脉宽可能长期保留。
- 没有“回中保持 500 ms”解锁，红外障碍解除后只要求回中一次。
- 航向保持通过 `YAW_HOLD_CONTROL_ENABLED=0` 排除出当前编译。
- 不具备基于绝对航向的抗打滑直线保持。

## 2. 目标芯片与工具链

| 项目 | 当前证据 |
|---|---|
| MCU | STM32F103ZET6，LQFP144，Cortex-M3 |
| Flash / RAM | 512 KiB Flash / 64 KiB SRAM |
| 系统时钟 | 外部 HSE，经 PLL x9 得到 72 MHz；APB1 为 36 MHz，APB1 定时器时钟为 72 MHz |
| HAL | STM32Cube FW_F1 V1.8.7 |
| 编译宏 | `USE_HAL_DRIVER`、`STM32F103xE` |
| GNU 构建 | CMake 3.22+、Ninja、GNU Arm Embedded；Debug 为 `-Og -g3`，Release 为 `-Os` |
| Keil 构建 | `MDK-ARM/stm32zet6.uvprojx`，目标设备 STM32F103ZE |

芯片型号由 `stm32zet6.ioc`、Keil 工程、GNU startup、链接脚本和编译宏交叉确认。GNU 链接脚本为 `STM32F103ZETX_FLASH.ld`。

## 3. 固件引脚与外设映射

下表描述的是**当前源码配置的逻辑角色**。除代码注释明确写有实测外，外部传感器、电调和电机的真实物理连接仍应通过原理图或实车逐路确认。

### 3.1 遥控输入

| 逻辑通道 | MCU 引脚 | 模式 | 当前用途 |
|---|---:|---|---|
| CH1 | PA6 | 双边沿 EXTI，无内部上下拉 | 转向 |
| CH2 | PA2 | 双边沿 EXTI，无内部上下拉 | 采集并遥测，未参与当前运动控制 |
| CH3 | PA3 | 双边沿 EXTI，无内部上下拉 | 前进/后退油门 |
| CH4 | PA7 | 双边沿 EXTI，无内部上下拉 | 采集并遥测，未参与当前运动控制 |
| CH5 | PB0 | 双边沿 EXTI，无内部上下拉 | 采集并遥测，未参与当前运动控制 |
| CH6 | PB1 | 双边沿 EXTI，无内部上下拉 | 采集并遥测，未参与当前运动控制 |

TIM5 以 1 MHz 运行，脉冲宽度等于下降沿计数减去上升沿计数。合法帧进入 7 点中值滤波；首次合法帧立即采用，后续真实阶跃至少连续 4 帧才会成为中值。对典型 50 Hz 接收机，阶跃延迟约为 80 ms。

### 3.2 红外输入

| 逻辑名称 | MCU 引脚 | 模式 | 有效电平 | 用途 |
|---|---:|---|---|---|
| IR1 | PC4 | 双边沿 EXTI、内部上拉 | 低 | 急停输入 |
| IR2 | PC5 | 双边沿 EXTI、内部上拉 | 低 | 急停输入 |
| IR3 | PC8 | 双边沿 EXTI、内部上拉 | 低 | 急停输入 |
| IR4 | PC9 | 双边沿 EXTI、内部上拉 | 低 | 急停输入 |

主循环持续轮询四路电平，中断回调也会立即锁存急停，避免仅依赖单一检测路径。未接红外输入在内部上拉作用下通常表现为未触发，但实际模块输出电平和电气兼容性仍需现场确认。

### 3.3 电调 PWM 输出

| 软件角色 | MCU 引脚 | 外设 | 参数 |
|---|---:|---|---|
| 左履带 PWM | PE9 | TIM1_CH1 | 50 Hz，1000～2000 us，1500 us 停止 |
| 右履带 PWM | PE11 | TIM1_CH2 | 50 Hz，1000～2000 us，1500 us 停止 |

`CommitTrackPwm(left, right)` 实际把左参数写入 TIM1_CH1/PE9、右参数写入 TIM1_CH2/PE11。`main.c` 顶部仍有一处旧注释把左右写反，应以函数写寄存器的实际顺序为准；“PE9 物理上确实接左电调”仍需实车最终确认。

### 3.4 编码器

| 编码器逻辑号 | 定时器 | A/B 引脚 | 软件归属 |
|---|---|---|---|
| ENC1 | TIM2 | PA15 / PB3，TIM2 partial remap 1 | 右履带速度平均值的一部分 |
| ENC2 | TIM3 | PB4 / PB5，TIM3 partial remap | 右履带速度平均值的一部分 |
| ENC3 | TIM4 | PB6 / PB7 | 左履带速度平均值的一部分 |
| ENC4 | TIM8 | PC6 / PC7 | 左履带速度平均值的一部分 |

四个定时器均为 `TIM_ENCODERMODE_TI12`、16 位周期、双通道上升沿、数字滤波值 4。代码按计数器的 16 位回绕差值累计为 32 位计数。

当前距离比例修正：

| 宏 | 数值 |
|---|---:|
| `ENC1_DISTANCE_SCALE` | 794 / 1000 |
| `ENC2_DISTANCE_SCALE` | 869 / 1000 |
| `ENC3_DISTANCE_SCALE` | 1013 / 1000 |
| `ENC4_DISTANCE_SCALE` | 988 / 1000 |
| 右侧附加参考比例 `TRACK_RS_REFERENCE_Q10` | 1222 / 1024 |

这些数值来自源码注释中的既有实测标定，但编码器方向、机械安装位置和倍率仍应逐路复验。

### 3.5 串口

| 接口 | 引脚 | 参数 | 当前用途 |
|---|---|---|---|
| USART1 | PA9 TX / PA10 RX | 115200 8N1 | 遥控/PWM CSV 遥测；接收电调测试命令 |
| USART2 remap | PD5 TX / PD6 RX | 115200 8N1 | 串口九轴 IMU |
| USART3 | PB10 TX | 115200 8N1 | 仅发送 Yaw、Pitch、Roll |
| UART4 | PC10 TX / PC11 RX | 115200 8N1 | 驱动函数存在，但 `main()` 当前没有初始化，不属于运行路径 |

USART2 已重映射至 PD5/PD6，因此不会占用 CH2/CH3 使用的 PA2/PA3。

## 4. 关键控制参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `RC_INPUT_VALID_MIN_US` | 1000 us | 最小合法遥控脉宽 |
| `RC_INPUT_VALID_MAX_US` | 2100 us | 最大合法遥控脉宽 |
| `RC_DEADBAND_US` | ±40 us | CH1/CH3 中位死区，即 1460～1540 us |
| `RC_INPUT_MEDIAN_SAMPLES` | 7 | 遥控中值滤波窗口 |
| `TRACK_SPEED_SAMPLE_INTERVAL_MS` | 20 ms | 编码器速度采样周期 |
| `TRACK_SPEED_CONTROL_MIN_THROTTLE_US` | 60 us | 速度同步启动所需的最小油门偏差 |
| `TRACK_SPEED_CONTROL_MIN_AVERAGE_CPS` | 300 cps | 速度同步最低平均速度 |
| `TRACK_SPEED_CONTROL_DEADBAND_CPS` | 50 cps | 左右速度差死区 |
| `TRACK_SPEED_CONTROL_KP_US_PER_CPS` | 0.020 | 速度同步比例系数 |
| `TRACK_SPEED_CONTROL_KI_US_PER_SAMPLE` | 0.0015 | 速度同步积分系数 |
| `TRACK_SPEED_CONTROL_MAX_CORRECTION_US` | ±60 us | 同步修正限幅 |
| `TANK_STEERING_PRIORITY` | 1 | 有转向输入时优先原地转向 |
| `YAW_HOLD_CONTROL_ENABLED` | 0 | 航向保持不参与当前编译和控制 |

## 5. 运行流程

1. HAL 初始化并把 HSE/PLL 配置为 72 MHz 系统时钟。
2. 初始化 GPIO、USART1/2/3、TIM5、TIM2、TIM1、TIM3、TIM4、TIM8。
3. 启动两路电调 PWM前先把比较值设置为 1500 us。
4. 初始化可选 IMU 串口接收；IMU 缺失不会阻止车辆控制。
5. 启动 TIM5 时间基准和四路编码器。
6. 主循环读取红外、解析 IMU和串口测试命令、更新编码器速度。
7. 红外激活或锁存时，无条件清零速度同步并提交左右 1500 us。
8. 独立串口测试模式下直接提交左右测试 PWM。
9. 正常模式下以 CH3 为油门、CH1 为转向计算差速命令；直行时叠加编码器速度同步修正。
10. USART1 每 100 ms 输出一次遥测，USART3 每 20 ms 输出一次姿态角。

## 6. 保护逻辑与边界

### 6.1 当前已有保护

- 上电时电调 PWM 先置为 1500 us。
- 非法遥控脉宽不会进入控制命令；命令最终限制在 1000～2000 us。
- CH1/CH3 在 1460～1540 us 范围内视为回中。
- 红外低电平立即停车并锁存，所有运行时 PWM 都经过统一的 `CommitTrackPwm()` 急停检查。
- 障碍解除后，旧推杆命令不会自动恢复；需要 CH1/CH3 先回中一次。
- 速度同步只在油门足够、没有转向且平均编码器速度达到门槛时启用，并具有积分及输出限幅。
- 串口测试命令超过 500 ms 未收到新字节后，测试目标回到 1500 us。

### 6.2 明确不存在的保护

- MCU 不检查 `CHx_LastTick` 的新鲜度，遥控器或接收机断开后不保证自动停车。
- 不存在上电必须回中的锁存。
- IMU 数据不参与安全判断、速度同步或运动许可。
- `Error_Handler()` 会关闭中断并永久停在循环中，没有运行时恢复机制。

## 7. 串口使用

### 7.1 USART1 遥测

USART1 每 100 ms 输出一行，无表头：

```text
CH1,CH2,CH3,CH4,CH5,CH6,LPWM,RPWM
```

单位均为 us。前六列是滤波后的接收机脉宽，最后两列是最终提交给左右电调的软件 PWM 值。当前遥测没有输出四路编码器、左右速度、PI 误差或急停状态。

### 7.2 USART1 测试命令

命令以换行 `\n` 结束：

```text
T=1600
R=1600
L=1600
STOP
LIVE
```

- `T=1500..2000`：进入统一油门测试，转向固定回中。
- `R=1500..2000`、`L=1500..2000`：分别设置左右履带并进入独立测试模式。
- `STOP`：进入独立测试模式并把两侧设为 1500 us。
- `LIVE`：退出串口测试模式，恢复遥控器控制。
- 测试值只接受 1500～2000 us，因此当前串口命令不能测试反向。
- 500 ms 超时只把测试目标恢复为 1500 us，**不会退出测试模式**；遥控不响应时应先发送 `LIVE\n`。
- 为保持 USART1 输出为纯 CSV，命令处理函数当前不会发送文字确认。

### 7.3 USART2 IMU与 USART3 姿态输出

USART2 使用帧头 `0x7E 0x23` 接收 IMU 数据，支持加速度、角速度、磁场、四元数、欧拉角、气压和版本帧。欧拉角 `0x26` 按小端 float 的 Roll、Pitch、Yaw 弧度解析，再转换为角度。

USART3 输出格式：

```text
Yaw,Pitch,Roll
```

Yaw 被整理到 0.00～359.99°，Pitch和 Roll 带正负号。首个有效 IMU 姿态帧到达前输出三个 0。该输出仅用于观测，不参与当前控制。

## 8. 遥控 CH 不变化的排查

保护逻辑不会改写 `CH1_PulseWidth`～`CH6_PulseWidth`。如果前六列固定，问题位于遥控器、接收机、接线或 EXTI 脉宽采集链；如果前六列变化而 LPWM/RPWM固定为1500，则检查红外锁存和串口测试模式。

建议断开电机动力后按以下顺序检查：

1. 分别推动所有摇杆并保持至少 1 秒，观察 USART1 前六列，确认实际通道映射。
2. 确认接收机输出的是六路独立 PWM，而不是 PPM、SBUS或 iBUS。
3. 确认接收机与 STM32 共地，并在对应 MCU 引脚上测到约 20 ms 周期的脉冲。
4. Debug Watch 中观察 `CH3_LastRise`、`CH3_LastTick`、`CH3_State`和 `CH3_PulseWidth`：
   - `LastTick` 持续更新但脉宽不变：接收机输出本身固定或遥控通道映射不符；
   - `LastRise` 更新但 `LastTick` 不更新：测得脉宽不在 1000～2100 us；
   - 两者都不更新：PA3 没有被识别到边沿，应检查引脚、共地、输出模式和烧录固件版本。
5. 如果 CH 会变化但 LPWM/RPWM不变，检查 `EmergencyStopActive`、`EmergencyStopLatched`、`SerialTestMode`，并尝试发送 `LIVE\n`。

`CH3_RawPulseWidth` 当前只是对滤波后 `CH3_PulseWidth` 的复制，并非真正的滤波前原始脉宽，不能用它判断输入毛刺。

## 9. 构建

### 9.1 GNU Arm + CMake

工程自带本机 STM32Cube 工具链预设：

```powershell
cmake --preset debug
cmake --build --preset debug

cmake --preset release
cmake --build --preset release
```

主要输出：

```text
build/debug/stm32zet6.elf
build/debug/stm32zet6.hex
build/release/stm32zet6.elf
build/release/stm32zet6.hex
```

`CMakePresets.json` 中的工具链 PATH 是当前 Windows 用户的绝对路径；迁移到其他电脑时需要修改预设或提供等效 GNU Arm、Ninja 环境。

### 9.2 Keil MDK

打开：

```text
MDK-ARM/stm32zet6.uvprojx
```

工程目录中存在已跟踪的 Keil和 Release 构建产物，提交前应仔细检查暂存范围，避免把无关二进制变化一并提交。

## 10. 已知问题与待确认项

### 已确认的软件问题或限制

1. `stm32zet6.ioc` 已落后于当前源码：它仍描述旧的 TIM2/TIM3 遥控输入捕获，没有当前红外、TIM5、TIM4/TIM8 编码器、USART2重映射和 USART3配置。直接用 CubeMX 重新生成代码可能破坏当前工程。
2. 没有遥控信号新鲜度保护；最后一次合法 CH 值可能在丢帧后继续参与控制。
3. 7 点中值滤波会给真实阶跃增加约 80 ms 典型延迟。
4. USART1/USART3 使用阻塞式 `HAL_UART_Transmit()`，串口拥塞可能增加主循环抖动。
5. 串口测试超时后仍停留在测试模式，必须发送 `LIVE` 才恢复遥控。
6. `CH3_RawPulseWidth` 名称与实际内容不符。
7. `main.c` 顶部的 PE9/PE11 左右注释与 `CommitTrackPwm()` 实际提交顺序不一致。

### 待实车确认

- PE9是否物理连接左电调、PE11是否物理连接右电调。
- 四路编码器与具体电机/履带的物理对应关系、正方向和每圈计数。
- 电调实际中位、正反转范围以及是否接受当前 50 Hz、1000～2000 us 信号。
- 红外模块是否为低电平触发及其输出电压是否兼容 STM32 输入。
- 接收机通道映射、PWM范围、关机行为和自身 failsafe 设置。

## 11. 安全验收顺序

1. 履带悬空、限流供电，准备硬件断电。
2. 验证 CH1～CH6 实际脉宽与遥控器通道对应关系。
3. 验证左右 1500 us 都能可靠停止。
4. 分别低速测试左右电调方向和四路编码器方向。
5. 验证 IMU断开时遥控和编码器同步仍正常。
6. 运行中触发任一路红外，确认左右 PWM立即回到1500。
7. 移除障碍并保持原推杆，确认车辆不自动恢复。
8. CH1、CH3回中一次，再重新推杆，确认恢复控制。
9. 完成悬空验收后再进行低速地面测试。

## 12. 主要源码入口

- `Core/Src/main.c`：遥控采集、混控、红外锁存、编码器速度和主循环。
- `Core/Src/gpio.c`：遥控与红外 EXTI配置。
- `Core/Src/tim.c`：电调 PWM、TIM5时间基准和四路编码器。
- `Core/Src/usart.c`：USART1/2/3以及未启用的 UART4引脚配置。
- `Core/Src/imu_uart.c`：串口 IMU接收、帧解析和状态接口。
- `Core/Src/stm32f1xx_it.c`：EXTI和串口中断入口。
- `CMakeLists.txt`、`CMakePresets.json`：GNU Arm构建配置。
- `MDK-ARM/stm32zet6.uvprojx`：Keil MDK工程。
