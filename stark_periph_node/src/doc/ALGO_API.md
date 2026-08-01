# Stark Motor Control API

## 文件交付

拷贝以下两个文件到你的项目:

- `stark_client.h` -- 所有 API, Header-Only, 零依赖
- `stark_shm.h`   -- 共享内存数据结构

编译: `gcc -O2 your_algo.c -lpthread -lrt -lm`

---

## 快速开始

```c
#include "stark_client.h"

int main() {
    stark_client_t c;

    /* 1. 连接 SHM (stark_node 后启动时会自动等待) */
    while (stark_open(&c) != 0) usleep(100000);

    /* 2. 等待校准完成 */
    while (!stark_ready(&c)) usleep(100000);

    /* 3. 使能 + 设电流模式 */
    stark_enable(&c, 1);  stark_enable(&c, 2);
    stark_set_mode(&c, 1, STARK_MODE_CURRENT);
    stark_set_mode(&c, 2, STARK_MODE_CURRENT);
    usleep(5000);

    /* 4. 控制循环 */
    uint32_t rt_cycle = 0;
    while (1) {
        motor_data_t m1 = stark_fb(&c, 1);
        motor_data_t m2 = stark_fb(&c, 2);
        imu_data_t   imu = stark_imu(&c);

        int32_t t1 = your_control_right(m1, m2, imu);
        int32_t t2 = your_control_left(m2, m1, imu);

        stark_multi(&c, t1, 0, 0, t2, 0, 0);
        usleep(1000);

        /* 每 200ms: 心跳 + stark_node 存活检测 */
        static int tick = 0;
        if (++tick % 200 == 0) {
            stark_heartbeat(&c);
            if (!stark_rt_alive(&c, &rt_cycle)) {
                while (!stark_ready(&c)) usleep(100000);
                stark_enable(&c, 1); stark_enable(&c, 2);
                usleep(5000);
            }
        }
    }

    /* 5. 退出 */
    stark_estop(&c, 1);  stark_estop(&c, 2);
    stark_close(&c);
}
```

---

## API 参考

### 生命周期

| 函数 | 说明 |
|------|------|
| `int stark_open(stark_client_t* c)` | 连接 SHM, 返回 0 成功, -1 失败. 可重复调用等待 stark_node 启动 |
| `void stark_close(stark_client_t* c)` | 断开 SHM |

### 状态查询

| 函数 | 返回 |
|------|------|
| `int stark_ready(c)` | 1=校准完成, 可控制 |
| `int stark_online(c, id)` | 1=电机在线, id=1右 2左 |
| `int stark_state(c)` | 0=BOOTING 1=READY 2=RUNNING 3=FAULT |
| `int stark_calib(c)` | 0=空闲 1=校准中 2=完成 3=超时 |
| `int stark_severity(c)` | 0=OK 1=WARN 2=FAULT |
| `int stark_fault_reason(c)` | 故障码, 见 fault_reason_t |

### 反馈读取 (零拷贝, 读 SHM 双 Buffer)

```c
motor_data_t stark_fb(stark_client_t* c, int id);
// motor_data_t 字段:
//   int16_t  position      编码器角度, counts [-32768,32767], [-180°,180°]
//   int16_t  velocity      转速, RPM
//   int16_t  current_iq    Q轴电流, mA
//   int16_t  temperature   温度, 0.1°C (除以10得°C)
//   uint8_t  status_byte  状态位
//   uint8_t  mode         当前控制模式
//   uint8_t  error_code   故障码

imu_data_t stark_imu(stark_client_t* c);
// imu_data_t 字段:
//   float  roll, pitch, yaw        欧拉角, °
//   float  acc_x, acc_y, acc_z     加速度, g
//   float  gyro_x, gyro_y, gyro_z  角速度, dps
//   float  quat_w/x/y/z            9轴四元数
//   float  mag_x, mag_y, mag_z     磁力计, uT
//   float  heading_deg             航向角, °
//   int    stationary              静止检测: 0=运动 2=静止
//   int    gyr_accuracy            陀螺校准精度: 0=未校准 3=精校准

stark_sensor_data_t stark_sensor(stark_client_t* c, int id);
// 传感器透传: hall_adc0/1/2, force_raw, knee_adc, key_landing

barometer_data_t stark_baro(stark_client_t* c);
// 气压计: pressure_hpa, temperature_c, altitude_m
```

### 控制命令 (实时, 1KHz PDO)

| 函数 | 参数 | 模式 |
|------|------|:---:|
| `stark_multi(c, t1,v1,p1, t2,v2,p2)` | 双电机一帧, t=mA v=RPM p=deg | 当前模式 |
| `stark_torque(c, id, mA)` | 电流 mA | CURRENT |
| `stark_speed(c, id, rpm)` | 速度 RPM | CSV |
| `stark_position(c, id, deg)` | 绝对角度 ° | CSP |
| `stark_pp(c, id, deg, acc, vel)` | 轮廓位置 °, RPM/s, RPM | PP |
| `stark_pv(c, id, rpm, acc)` | 轮廓速度 RPM, RPM/s | PV |
| `stark_mit(c, id, pos,vel, kp,kd, tor)` | MIT 阻抗 | MIT |

推荐使用 `stark_multi`, 一帧 CANFD 同时控制双电机, 同步更好.

`stark_multi` 的参数含义跟随当前控制模式:
- CURRENT 模式: 只用 t1/t2 (电流), v1/v2/p1/p2 填 0
- CSP 模式: p1/p2 为绝对角度
- CSV/PV 模式: v1/v2 为速度

### 管理命令 (非实时, 走独立通道)

| 函数 | 说明 |
|------|------|
| `stark_enable(c, id)` | 使能电机 |
| `stark_disable(c, id)` | 失能电机 + 刹车释放 + 电流置零 |
| `stark_estop(c, id)` | 急停: 失能 + 刹车抱死 + 电流置零 |
| `stark_recover(c, id)` | 从急停恢复 |
| `stark_clear_fault(c, id)` | 清故障 + 自动使能 + 切电流模式 target=0 |
| `stark_set_mode(c, id, mode)` | 切换控制模式, mode 取值见下文 |

管理命令和控制命令不要在同一周期混合发送, 管理命令后至少间隔 5ms.

### 双向心跳

| 函数 | 说明 |
|------|------|
| `stark_heartbeat(c)` | 算法声明存活, 建议每 200ms 调用 |
| `stark_rt_alive(c, &last)` | stark_node 反向存活检测, 返回 0 需重连 |

stark_node 心跳超时 (默认 1000ms, 可配) 后自动脱使能双电机, 不设 FAULT.
心跳恢复后算法自行 stark_enable 重新使能.

### 周期上报 (校准完成后自动开启, 5ms 推送)

| 函数 | 说明 |
|------|------|
| `PeriodicUploadData* stark_report_data(c)` | 数据指针, 未开启返回 NULL |
| `uint32_t stark_report_version(c)` | 版本号, 单调递增, 比对检测更新 |

PeriodicUploadData 包含: 电机转速/角度/电流/温度, IMU 角速度/四元数/欧拉角, 传感器 Hall/力矩/膝关节/着地开关, 气压.

---

## 控制模式

| 模式 | 常量 | 说明 |
|:---:|------|------|
| 1 | `STARK_MODE_PP` | 轮廓位置, 驱动板侧梯形加减速 |
| 2 | `STARK_MODE_PV` | 轮廓速度, 驱动板侧梯形加减速 |
| 3 | `STARK_MODE_CSP` | 循环同步位置, SYNC 触发 |
| 4 | `STARK_MODE_CSV` | 循环同步速度, SYNC 触发 |
| 5 | `STARK_MODE_CURRENT` | Q轴电流直控 (外骨骼推荐) |
| 6 | `STARK_MODE_MIT` | MIT 阻抗控制 |

---

## 电机 ID

- 1: 右髋
- 2: 左髋

---

## 运行规则

1. root 权限运行 (SHM 由 stark_periph_manager_node 以 root 创建)
2. 管理命令和控制命令间隔至少 5ms
3. 双电机推荐 stark_multi, 同步更好
4. 必须定期调用 stark_heartbeat (建议 200ms), 心跳超时自动脱使能
5. 外骨骼场景: 算法不发命令时反馈正常更新, 电机不发力, 关节自由
6. stark_node 后启动时 stark_open 可循环重试等待

---

## stark_node 心跳配置

config.json 中 safety 段:

```json
{
  "safety": {
    "heartbeat_timeout_ms": 1000,
    "overtemp_celsius": 80,
    "can_offline_ms": 2000,
    "encoder_stall_s": 3
  }
}
```

## 电机自动使能配置

默认 auto_enable=false (算法控制使能时机):

```json
{
  "motors": [
    { "id": 1, "auto_enable": false },
    { "id": 2, "auto_enable": false }
  ]
}
```

设为 true 恢复旧行为 (启动即自动使能 + 校准后重新使能).

---

## 编译示例

```bash
# 将 stark_client.h 和 stark_shm.h 放入项目目录
gcc -O2 your_algo.c -lpthread -lrt -lm -o your_algo

# 参考 demo_algo.c 了解完整用法
gcc -O2 demo_algo.c -lpthread -lrt -lm -o demo_algo
```
