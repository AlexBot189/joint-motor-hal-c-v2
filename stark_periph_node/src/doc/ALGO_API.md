# stark_periph_manager_node — 算法接口文档

## 1. 概述

stark_periph_manager_node 是基于 RV1126B + CANFD 的电机控制节点，通过共享内存 (SHM) 向上层算法提供控制接口。

算法进程只需一个头文件即可完成闭环控制：`stark_client.h`

### 数据流向

```
stark_periph_manager_node (CANFD 驱动 + RT 线程)
        |                         ^
        | SHM fb_buffer          | SHM mailbox
        v                         |
   算法进程 (stark_client.h)
  (步态 / WBC / MPC / 助行)
```

反馈: RT 线程 1KHz 采样电机数据, 写 SHM 双 Buffer, 算法从 active buffer 读取
控制: 算法写 SHM mailbox, RT 线程 1KHz 读取, 通过 PDO 发 CANFD 帧到电机驱动板

## 2. 文件交付

| 文件 | 说明 |
|---|---|
| `stark_client.h` | Header-Only API, static inline, 零依赖 |
| `stark_shm.h` | 共享内存数据结构定义 (反馈/命令/IMU/传感器) |
| `demo_algo.c` | 示例程序: 力矩/速度/位置/MIT/多轴 5 种模式 |
| `stark_tool` | CLI 调试工具 (编译: gcc -O2 stark_tool.c -lpthread -lrt -o stark_tool) |

## 3. API 接口

### 3.1 生命周期

```c
stark_client_t c;
int ret = stark_open(&c);      /* 连接 SHM, 返回 0 成功 */
/* ... 控制循环 ... */
stark_close(&c);               /* 断开 */
```

### 3.2 状态查询

```c
int stark_ready(&c);           /* node_state==RUNNING AND calib_state==done */
int stark_online(&c, motor_id); /* 电机在线? id: 1=右髋 2=左髋 */
int stark_state(&c);           /* 0=BOOTING 1=READY 2=RUNNING 3=FAULT */
int stark_calib(&c);           /* 0=空闲 1=校准中 2=完成 3=超时 */
int stark_severity(&c);        /* 0=OK 1=WARN 2=FAULT */
int stark_fault_reason(&c);    /* 故障原因码 */
```

### 3.3 反馈读取 (零拷贝)

```c
/* 电机反馈 */
motor_data_t stark_fb(&c, motor_id);   /* motor_id: 1=右髋 2=左髋 */

/* motor_data_t 字段:
   int16_t  position;     编码器角度, counts [-32768, 32767], 对应 [-180°, 180°]
   int16_t  velocity;     转速, RPM
   int16_t  current_iq;   Q轴电流, mA
   int16_t  temperature;  温度, 0.1°C (要转 °C 除以 10)
   uint8_t  status_byte;  状态位
   uint8_t  error_code;   故障码
*/

/* IMU 数据 (9轴融合) */
imu_data_t stark_imu(&c);

/* imu_data_t 字段:
   float roll, pitch, yaw;         欧拉角 (°)
   float acc_x, acc_y, acc_z;      加速度 (g)
   float gyro_x, gyro_y, gyro_z;   角速度 (dps)
   float quat_w, quat_x, quat_y, quat_z;  四元数
   float mag_x, mag_y, mag_z;      磁力计 (uT)
   float heading_deg;              航向角 (°)
   float temp_c;                   IMU 芯片温度
   int   stationary;               静止检测 0=运动 2=静止
   int   gyr_accuracy, mag_accuracy; 校准精度 0=未校准 3=精度高
*/

/* 传感器透传 */
stark_sensor_data_t stark_sensor(&c, motor_id);

/* 气压计 */
barometer_data_t stark_baro(&c);
```

### 3.4 控制命令 (实时路径, 写 SHM mailbox, PDO 直发)

#### 力矩控制

```c
void stark_torque(&c, motor_id, ma);
/* motor_id: 1=右髋 2=左髋, ma: 目标电流, 单位 mA, 范围 ±20000 */
```

#### 速度控制

```c
void stark_speed(&c, motor_id, rpm);
/* rpm: 目标速度, 单位 RPM */
```

#### 循环同步速度 (CSV 模式)

```c
void stark_csv(&c, motor_id, rpm);
```

#### 绝对位置控制 (CSP 模式)

```c
void stark_position(&c, motor_id, deg);
/* deg: 目标角度, 单位 °, 范围 ±180° */
```

#### 相对位置控制

```c
void stark_rel_position(&c, motor_id, delta_deg);
/* 自动读当前位置 + 偏移, 单位 ° */
```

#### 轮廓位置 (PP 模式, 带加减速)

```c
void stark_pp(&c, motor_id, deg, accel_rpm_per_s, vel_rpm);
```

#### MIT 阻抗控制

```c
void stark_mit(&c, motor_id, pos_deg, vel_rpm, kp, kd, torque_ma);
/* pos_deg: 目标位置 (°), vel_rpm: 目标速度, kp: 位置刚度, kd: 速度阻尼, torque_ma: 前馈力矩 */
```

#### 多轴广播 (一帧 64B CANFD 同时控制双电机, 推荐)

```c
void stark_multi(&c,
    int32_t torque1, int32_t vel1, int32_t pos1,   /* 电机1: 力矩(mA)/速度前馈(RPM)/位置(°) */
    int32_t torque2, int32_t vel2, int32_t pos2);   /* 电机2 */
/* 当前驱动板模式决定哪个字段生效.
   电流模式: 只用 torque1/torque2, vel 和 pos 填 0
   位置模式: 用 pos, vel 为速度前馈, torque 为前馈力矩 */
```

### 3.5 管理命令 (非实时, Byte0 修改, 下一周期生效)

```c
void stark_enable(&c, motor_id);        /* 使能 */
void stark_disable(&c, motor_id);       /* 失能 */
void stark_estop(&c, motor_id);         /* 急停: enable=0, bus=OFF */
void stark_recover(&c, motor_id);       /* 恢复: enable=1, bus=ON */
void stark_clear_fault(&c, motor_id);   /* 清故障 */
void stark_set_mode(&c, motor_id, mode);
/* mode: 1=PP 2=PV 3=CSP 4=CSV 5=Current 6=MIT */
```

### 3.6 关键注意事项

1. 管理命令 (enable/disable/estop/set_mode) 和实时命令 (torque/speed/multi) 不要在同一周期混合发送。管理命令触发 Byte0 修改, 实时命令复用该 Byte0。如需同时发送, 先发管理命令, usleep(2000) 后再发实时命令。
2. `stark_multi()` 是推荐的实时控制接口, 一帧 64B CANFD 同时发两个电机, 比单次 `stark_torque()` 分别发高效且同步。
3. 所有函数是 static inline, 零调用开销, 编译器会内联为直接内存访问。
4. SHM 中的 motor_online 掩码 bit0=电机1 bit1=电机2, 算法可以在线检测掉线。
5. stark_fb() 返回的是值拷贝 (16 字节), 在栈上直接传递, amd64 用单条 movaps。

## 4. 控制模式说明

| 模式 | mode 值 | 目标寄存器 | 说明 |
|---|---|---|---|
| 电流 (Current) | 5 | target_iq (0x6071) | Q轴电流直控, 常用 |
| 轮廓速度 (PV) | 2 | target_velocity (0x60FF) | 带加减速斜坡 |
| 循环同步位置 (CSP) | 3 | target_position | PDO 同步, 实时性高 |
| 循环同步速度 (CSV) | 4 | target_velocity | PDO 同步 |
| 轮廓位置 (PP) | 1 | target_position (0x607A) | 带加减速和轮廓速度 |
| MIT 阻抗 | 6 | pos/vel/kp/kd/torque | 阻抗控制 |

## 5. 示例程序

### 5.1 编译

```bash
gcc -O2 demo_algo.c -lpthread -lrt -lm -o demo_algo
```

### 5.2 运行

```bash
# 力矩控制模式: 正弦波扫描 5000mA
sudo ./demo_algo torque

# 速度控制模式: 梯形波 ±50RPM
sudo ./demo_algo speed

# 位置控制模式: 方波 ±30°
sudo ./demo_algo pos

# MIT 阻抗模式: kp=200, kd=50, 零位置
sudo ./demo_algo mit

# 多轴广播 + IMU 监控 (默认)
sudo ./demo_algo multi
```

### 5.3 核心控制循环 (最简示例)

```c
#include "stark_client.h"

int main() {
    stark_client_t c;
    stark_open(&c);                          /* 1. 连 SHM */
    while (!stark_ready(&c)) usleep(10000);  /* 2. 等就绪 */

    stark_enable(&c, 1); stark_enable(&c, 2);
    usleep(5000);

    while (1) {
        /* 读反馈 */
        motor_data_t m1 = stark_fb(&c, 1);
        motor_data_t m2 = stark_fb(&c, 2);
        imu_data_t imu  = stark_imu(&c);

        /* 算法: 这里是你的步态/WBC/MPC/助行逻辑 */
        int32_t torque1 = compute_torque_right(m1, m2, imu);
        int32_t torque2 = compute_torque_left(m1, m2, imu);

        /* 多轴广播 */
        stark_multi(&c, torque1, 0, 0, torque2, 0, 0);

        usleep(1000);  /* 1KHz */
    }

    stark_estop(&c, 1); stark_estop(&c, 2);
    stark_close(&c);
    return 0;
}
```

完整示例见 `demo_algo.c`, 包含 5 种控制模式和 IMU 数据打印。

## 6. CLI 调试工具 (stark_tool)

### 6.1 编译

```bash
gcc -O2 stark_tool.c -lpthread -lrt -o stark_tool
```

### 6.2 直接命令模式

```bash
# 状态查询
./stark_tool stat                    # 查看电机状态
./stark_tool watch 200               # 持续打印反馈 (200ms)

# 电流控制
./stark_tool torque 1 2000           # 电机1: 2000mA
./stark_tool torque 2 1000           # 电机2: 1000mA

# 速度控制
./stark_tool speed 1 50              # 电机1: 50RPM

# 位置控制
./stark_tool abs 1 45                # 电机1: 绝对 45°
./stark_tool rel 1 10                # 电机1: 相对 +10°

# 多轴广播
./stark_tool multi 2000 0 0 2000 0 0 # 双电机各 2000mA

# MIT 阻抗
./stark_tool mit 1 0 0 500 100 0     # kp=500, kd=100

# 管理命令
./stark_tool enable 1                # 电机1 使能
./stark_tool disable 2               # 电机2 失能
./stark_tool estop 1                 # 电机1 急停
./stark_tool recover 1               # 电机1 恢复
./stark_tool clearf 1                # 电机1 清故障
./stark_tool mode 1 5                # 电机1 切电流模式
./stark_tool stop                    # 全部电机急停
```

### 6.3 Daemon 模式 (Unix Socket)

```bash
# 终端1: 启动 daemon
./stark_tool daemon
# 输出: stark_tool daemon listening on /tmp/stark_cmd.sock

# 终端2: 通过 nc 发命令
echo "stat"           | nc -U /tmp/stark_cmd.sock
echo "torque 1 2000"  | nc -U /tmp/stark_cmd.sock
echo "watch 500"      | nc -U /tmp/stark_cmd.sock   # 注意: watch 长时间运行, 需独立终端
echo "stop"           | nc -U /tmp/stark_cmd.sock
```

### 6.4 典型调试流程

```bash
# (1) 确认节点运行
ps aux | grep stark_periph_manager_node

# (2) 查看电机状态
./stark_tool stat
# 期望: State: 2(RUNNING) Calib: 2(done) Motor: [1] pos=XX [2] pos=XX

# (3) 电流控制测试
./stark_tool enable 1
./stark_tool mode 1 5               # 电流模式
./stark_tool torque 1 500           # 500mA, 电机应该微微转动

# (4) 监控反馈
./stark_tool watch 200              # Ctrl+C 停止

# (5) 停止
./stark_tool stop
```

## 7. 联调流程

### 7.1 准备 (你来负责)

```bash
# 1. 配置 CANFD
ip link set can0 down
ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
ip link set can0 up

# 2. 电机上电
# 3. 启动节点
sudo ./stark_periph_manager_node -c stark_config.json

# 4. 等待校准完成 (日志输出)
# [main] calib done + algo connected, entering RUNNING
```

### 7.2 算法同事侧

```bash
# 1. 编译
gcc -O2 algo_control.c -lpthread -lrt -lm -o algo

# 2. 运行
sudo ./algo

# 输出:
# [init] SHM 已连接: /stark_shm
# [init] 等待电机就绪 (校准)...
# [init] 就绪! 在线电机: 1 在线 2 在线
# [init] 电机已使能, 开始 torque 模式
# ...
```

### 7.3 排查手段

| 问题 | 检查 | 命令 |
|---|---|---|
| SHM 连不上 | 节点是否在运行 | `ps aux \| grep stark` |
| 电机离线 | 电源/CAN 线 | `./stark_tool stat` |
| 校准未完成 | 校准状态 | `./stark_tool stat` 看 calib |
| FAULT 状态 | 故障原因 | `./stark_tool stat` 看 severity/fault |
| 命令没响应 | SHM 写入 | `cat /dev/shm/stark_shm \| xxd \| head` |
| CAN 断线 | CAN 接口状态 | `ip -details link show can0` |
| 周期超限 | RT 负载 | `./stark_tool stat` 看 overruns |


```
算法进程检测到 FAULT:
  1. 读 stark_severity() 和 stark_fault_reason() 确认原因
  2. 如果是 ALGO_TIMEOUT: 恢复发送命令, RT 线程自动回到 READY
  3. 如果是其他故障: 调用 stark_clear_fault(), 然后 stark_recover()
  4. 状态回到 READY 后, stark_enable(), 重新开始控制
```
