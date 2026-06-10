/*
 * exo_shm.h — 共享内存布局
 *
 * 数据方向:
 *   motor_node 写 → fb_buffer (反馈帧) → 算法/ROS/Web 读
 *   算法写       → mailbox (控制命令)  → motor_node 读
 *
 * 并发模型: 所有字段单写者, 多读者安全. 不需要锁.
 *   - fb_buffer: 双 Buffer, RT 工作线程写, 多个非 RT 读者
 *   - mailbox:   算法进程写, RT 工作线程读 (seq_begin/seq_end snapshot 保护)
 *   - 状态字段:  motor_node 内部写, 算法只读
 *
 * v3.0 (2026-06-09):
 *   - 简化为 2 电机 (髋关节)
 *   - 加入端到端延迟追踪字段
 *   - 加入 mock IMU + 气压计数据
 *   - 加入双电机 mailbox
 */
#pragma once

#include <stdint.h>

#define EXO_SHM_NAME    "/exo_shm"
#define EXO_SHM_SIZE    (64 * 1024)
#define EXO_MOTOR_COUNT 2

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 电机反馈 (来自 CAN 反馈帧 0x300, 大端)
 * ============================================================================ */
typedef struct {
    int16_t  position;          /* 编码器角度, counts [-32768,32767] → [-180°,180°]   */
    int16_t  velocity;          /* 转速, RPM                                          */
    int16_t  current_iq;        /* Q轴电流, mA                                        */
    int16_t  temperature;       /* 温度, 0.1°C                                       */
    uint8_t  status_byte;       /* bit7:使能 bit6:抱闸 bit5:错误 bit4:到位             */
    uint8_t  mode;              /* 当前控制模式 (CiA 402)                              */
    uint8_t  error_code;        /* 故障码 (高4位=故障类型, 低4位=子码)                  */
    uint8_t  _pad;
} motor_data_t;

/* ============================================================================
 * 传感器透传 (来自 CAN 0x680 帧, 小端 bit-packed)
 * ============================================================================ */
typedef struct {
    uint16_t hall_adc0;         /* 线性霍尔 A, 0~4095                                */
    uint16_t hall_adc1;         /* 线性霍尔 B, 0~4095                                */
    uint16_t hall_adc2;         /* 线性霍尔 C, 0~4095                                */
    uint16_t force_raw;         /* DF181 力矩, 0~16383                              */
    uint16_t knee_adc;          /* 膝关节电位器, 0~4095                              */
    uint8_t  key_landing;       /* 着地开关, 0=低 1=高                               */
    uint8_t  data_valid;        /* 力矩数据有效标志                                    */
} exo_sensor_data_t;

/* ============================================================================
 * IMU 数据 (ICM45608, 当前 mock 填充)
 * ============================================================================ */
typedef struct {
    float    roll;              /* 横滚角, °                                         */
    float    pitch;             /* 俯仰角, °                                         */
    float    yaw;               /* 偏航角, °                                         */
    float    acc_x;             /* 加速度 X, m/s²                                    */
    float    acc_y;             /* 加速度 Y, m/s²                                    */
    float    acc_z;             /* 加速度 Z, m/s²                                    */
    float    gyro_x;            /* 角速度 X, °/s                                     */
    float    gyro_y;            /* 角速度 Y, °/s                                     */
    float    gyro_z;            /* 角速度 Z, °/s                                     */
    uint64_t timestamp_us;      /* IMU 采样时刻, μs                                   */
} imu_data_t;

/* ============================================================================
 * 气压计数据 (QMP6990, 当前 mock 填充)
 * ============================================================================ */
typedef struct {
    float    pressure_hpa;      /* 气压, hPa                                         */
    float    temperature_c;     /* 温度, °C                                          */
    float    altitude_m;        /* 估算海拔, m                                        */
    uint64_t timestamp_us;      /* 采样时刻, μs                                       */
} barometer_data_t;

/* ============================================================================
 * 反馈帧 — motor_node 组装后写入 SHM 双 Buffer
 * ============================================================================ */
typedef struct {
    motor_data_t      motor[EXO_MOTOR_COUNT];   /* [0]=右髋(ID=1) [1]=左髋(ID=2)       */
    exo_sensor_data_t sensor[EXO_MOTOR_COUNT];  /* 传感器透传, 与电机一一对应              */
    imu_data_t        imu;                      /* IMU 姿态数据                          */
    barometer_data_t  baro;                     /* 气压计数据                            */

    /* ── 端到端延迟追踪 (调试, 单位 μs) ── */
    uint64_t ts_can_rx;          /* T0: CANFD 反馈帧到达 (recv 线程)                     */
    uint64_t ts_cache_write;     /* T1: fb_cache 写入完成                                */
    uint64_t ts_shm_read;        /* T2: RT 线程读 fb_cache                               */
    uint64_t ts_shm_write;       /* T3: SHM 双 Buffer 切换 (atomic_store 前)             */
    uint64_t ts_algo_read;       /* T4: 算法读 SHM 完成 (算法填充)                        */
    uint64_t ts_algo_done;       /* T5: 算法计算完成 (算法填充)                           */
    uint64_t ts_mailbox_read;    /* T7: RT 线程读 mailbox (RT 填充)                      */
    uint64_t ts_pdo_sent;        /* T8: PDO/广播 下发完成 (RT 填充)                      */
    uint64_t ts_frame_assembly;  /* RT 线程组装反馈帧完成                                  */

    uint64_t timestamp_us;       /* 组装时刻, μs                                        */
    uint8_t  _pad[4];            /* 对齐到字边界                                         */
} feedback_frame_t;

/* ============================================================================
 * 电机控制命令
 * ============================================================================ */
typedef enum {
    EXO_CMD_TORQUE   = 1,       /* 力矩模式, value=mA                                   */
    EXO_CMD_SPEED    = 2,       /* 速度模式, value=RPM×100                              */
    EXO_CMD_POS      = 3,       /* 位置模式, value=°×100                                */
    EXO_CMD_MIT      = 4,       /* MIT 阻抗控制                                          */
    EXO_CMD_PP       = 5,       /* 轮廓位置模式 PP                                       */
    EXO_CMD_CSV      = 6,       /* 循环同步速度 CSV, value=RPM×100                          */
    /* 多轴广播: 当 cmd[0]和cmd[1]都设为MULTI时，RT线程打包成一帧64B多轴广播(COB 0x200) */
    EXO_CMD_MULTI    = 7,       /* 多轴广播, mode/value/value2/feedforward 字段有效            */
    /* PDO Byte0 控制 (不发 target, 只改 Byte0 状态) */
    EXO_CMD_ENABLE   = 10,      /* PDO使能 (Byte0 bit7=1)                                */
    EXO_CMD_DISABLE  = 11,      /* PDO失能 (Byte0 bit7=0)                                */
    EXO_CMD_ESTOP    = 12,      /* 急停: enable=0 + bus=OFF (Byte0=0x00, 保留mode)         */
    EXO_CMD_RECOVER  = 13,      /* 恢复: enable=1 + bus=ON (Byte0=0xC0, 保留mode)          */
    EXO_CMD_SET_MODE = 14,      /* 切换 PDO 控制模式, value=motor_mode_t                   */
    /* bit5 清错: 算法先读 fb->motor[i].error_code, 判断后决定是否清除 */
    EXO_CMD_CLEAR_FAULT = 15,   /* PDO Byte0 bit5 脉冲清除                               */
} exo_cmd_type_t;

typedef struct {
    uint8_t  motor_id;          /* 1=右髋 2=左髋                                       */
    uint8_t  cmd;               /* exo_cmd_type_t                                       */
    int32_t  value;             /* target1: 主目标值 (位置/速度/电流)                     */
    int32_t  value2;            /* target2: 加减速(PP/PV模式, RPM/s)                     */
    int32_t  feedforward;       /* 前馈 (PP模式轮廓速度 RPM, 其他模式填0)                */
    /* MIT 模式专用 (其他模式忽略) */
    uint16_t mit_pos;           /* 目标位置 [0-65535]                                  */
    uint16_t mit_vel;           /* 目标速度 [0-4095]                                   */
    uint16_t mit_kp;            /* 位置刚度 [0-4095]                                   */
    uint16_t mit_kd;            /* 速度阻尼 [0-4095]                                   */
    uint16_t mit_torque;        /* 前馈力矩 [0-4095]                                   */
    uint8_t  _pad[6];           /* 对齐到 offset 32, sizeof=40 */
    uint64_t timestamp_us;      /* 算法下发时刻                                          */
} motor_command_t;

/* ============================================================================
 * Mailbox — 双电机命令 (算法写, motor_node 读)
 * ============================================================================ */
typedef struct {
    uint64_t          seq_begin;   /* ★ 一机制三用途: 命令传输+握手信号+心跳              */
    motor_command_t   cmd[EXO_MOTOR_COUNT];  /* 双电机命令数组                           */
    uint64_t          seq_end;     /* 写完置为 seq_begin, reader 对比防撕裂               */
} exo_mailbox_t;

/* ============================================================================
 * 故障严重级别
 * ============================================================================ */
typedef enum {
    MOTOR_OK    = 0,            /* 正常运行                                            */
    MOTOR_WARN  = 1,            /* 降额: torque=0, 保持使能 (可自动恢复)                 */
    MOTOR_FAULT = 2,            /* 停机: DS402 Shutdown (需人工干预)                    */
} motor_severity_t;

/* ============================================================================
 * 故障原因
 * ============================================================================ */
typedef enum {
    FAULT_NONE             = 0,
    FAULT_ALGO_TIMEOUT,         /* 算法失联 (seq_begin 200ms 不变)                      */
    FAULT_CMD_STALL,            /* 输出停滞 (同一 cmd 重复 500ms)                       */
    FAULT_CAN_OFFLINE,          /* CAN 断线 (2s 无帧)                                  */
    FAULT_ENCODER_FAULT,        /* 编码器异常 (position 恒定 3s)                         */
    FAULT_OVERTEMP,             /* 驱动器过温 (> 80°C)                                 */
    FAULT_HANDSHAKE_TIMEOUT,    /* 握手超时 (ENABLED 状态 10s 无 cmd)                    */
    FAULT_CALIB_TIMEOUT,        /* 校准超时                                             */
} fault_reason_t;

/* ============================================================================
 * 节点状态机
 * ============================================================================ */
typedef enum {
    STATE_INIT        = 0,
    STATE_DISCOVERY   = 1,
    STATE_READY       = 2,
    STATE_CALIBRATING = 3,
    STATE_ENABLED     = 4,
    STATE_RUNNING     = 5,
    STATE_FAULT       = 6,
} exo_state_t;

#define EXO_STATE_COUNT  7

/* ============================================================================
 * 共享内存总结构 (64KB)
 * ============================================================================ */

typedef struct {
    /* ── 双 Buffer 反馈区 (motor_node 写, 算法/ROS/Web 读) ── */
    uint32_t          active_idx;            /* 0 或 1, atomic release/acquire          */
    feedback_frame_t  fb_buffer[2];          /* 双 Buffer 防撕裂                         */

    /* ── Mailbox 命令区 (算法写, motor_node 读) ── */
    exo_mailbox_t     mailbox;

    /* ── 状态区 (motor_node 写, 算法只读) ── */
    uint8_t   motor_online;               /* bit0=右(1) bit1=左(2)                       */
    uint8_t   calib_state;                /* 0=空闲 1=校准中 2=完成 3=超时                */
    uint8_t   motor_enabled;              /* 每 bit 对应电机使能状态                       */
    uint8_t   motor_severity;             /* 0=OK 1=WARN 2=FAULT                        */
    uint8_t   fault_reason;               /* fault_reason_t 枚举                         */
    uint8_t   node_state;                 /* exo_state_t                                 */

    /* ── 耗时追踪 (EXO_LATENCY_TRACE) ── */
    /* 反馈路径 (μs) */
    uint16_t  fb_read_avg_us;
    uint16_t  fb_read_max_us;
    uint16_t  fb_total_avg_us;       /* T1→T4 反馈总延迟 */
    uint16_t  fb_total_max_us;
    /* 控制路径 (μs) */
    uint16_t  ctrl_total_avg_us;     /* T5→T6 控制总延迟 */
    uint16_t  ctrl_total_max_us;
    /* 全局 */
    uint32_t  trace_cycle_count;     /* 已采样周期数 */
    uint32_t  shm_write_avg_us;      /* SHM 写入耗时 */
    uint16_t  cycle_overrun_count;   /* 周期超限次数 */
    uint8_t   _pad_latency[2];
    uint16_t  _pad2[3];                   /* 对齐                                         */

    uint8_t   _pad[3904];                 /* 对齐 64KB                                    */
} exo_shm_t;

#ifdef __cplusplus
}
#endif
