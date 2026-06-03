# Motor HAL — 关节模组 CANopen/CANFD 硬件抽象层 (纯C)

纯 C 实现的关节电机控制库，基于 CANopen CiA 402 协议。
适配**无锡巨蟹智能驱动**一体化通讯模组。

## 特性

- **纯 C11**: 无外部依赖, 无 C++ runtime
- **CANFD**: 仲裁1Mbps + 数据5Mbps, 标准帧11bit
- **CANopen CiA 402**: DS402 状态机完整实现
- **模块化**: 12个独立源文件, 按功能拆分
- **SDO + PDO**: 参数配置 + 实时控制双通道
- **反馈回调**: 自动解析位置/速度/电流/温度/错误码
- **MIT 阻抗控制**: 力位混合控制
- **多轴广播**: 一帧控制8电机 (CANFD 64B)
- **可集成 ROS**: 纯C库 → C++ bridge → ROS topic

## 平台

- **目标**: RV1126B + Linux
- **编译器**: GCC 5+ / C11
- **依赖**: 仅 Linux SocketCAN + pthread

## 文件结构

```
motor_hal_c/
├── CMakeLists.txt
├── README.md
├── inc/
│   ├── motor_hal.h          # 公共 API (纯C)
│   ├── motor_hal_types.h    # 类型/常量/回调
│   └── canopen_frames.h     # 帧构造/解析
├── src/
│   ├── can_driver.c         # SocketCAN 驱动
│   ├── canopen_frames.c     # 帧引擎
│   ├── sdo_client.c         # SDO 客户端
│   ├── nmt_master.c         # NMT 主站
│   ├── heartbeat.c          # 心跳/看门狗
│   ├── pdo_handler.c        # PDO 发送
│   ├── feedback_parser.c    # 反馈解析
│   ├── motor_hal_startup.c  # 启动流程
│   ├── utils.c              # 工具函数
│   └── motor_hal.c          # 核心 HAL
└── examples/
    ├── single_motor.c       # 单电机 demo
    ├── dual_motor.c         # 双关节 demo
    └── mit_control.c        # MIT 模式 demo
```

## 快速开始

```bash
mkdir build && cd build
cmake ..
make -j4

# 配置 CAN 接口
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

# 运行
./motor_example_single
```

## API 速查

```c
motor_hal_t *hal = motor_hal_create();
motor_hal_init(hal, "can0", 1000000, 5000000);

// 注册
motor_config_t cfg = { .node_id = 1, .disable_watchdog = true };
motor_hal_add_motor(hal, &cfg);

// 回调
void on_feedback(uint8_t id, const motor_feedback_t *fb, void *ctx) {
    float deg = motor_counts_to_deg(fb->position);  // -180~180°
}
motor_hal_set_feedback_cb(hal, 1, on_feedback, NULL);

// 启动 (等待bootup → 参数配置 → 使能)
motor_hal_startup(hal, 1, 5000);

// 控制 (直接发自定义PDO)
motor_hal_set_position(hal, 1, 30.0f);
motor_hal_set_velocity(hal, 1, 500.0f);
motor_hal_set_torque(hal, 1, 2000);

// 主循环
while (1) {
    motor_hal_poll(hal, 1);
    // ... 控制逻辑 ...
}

motor_hal_destroy(hal);
```

## COB-ID

| 功能 | ID | 字节 |
|------|-----|------|
| NMT | 0x000 | 2 |
| SYNC | 0x080 | 0 |
| SDO TX | 0x600+ID | 8 |
| SDO RX | 0x580+ID | 8 |
| 自定义PDO | 0x100+ID | 7 |
| MIT PDO | 0x110+ID | 9 |
| 多轴广播 | 0x200 | 64 |
| 反馈帧 | 0x300+ID | 12 |
| Bootup/HB | 0x700+ID | 1 |

## 交叉编译 (RV1126B)

```bash
cmake .. -DCROSS_RV1126=ON \
    -DCMAKE_SYSROOT=/opt/rv1126/sysroot
make -j4
```

## ROS 集成示例

```cpp
// motor_ros_bridge.cpp
#include "motor_hal.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

void feedback_cb(uint8_t id, const motor_feedback_t *fb, void *ctx) {
    auto *pub = (ros::Publisher*)ctx;
    sensor_msgs::JointState msg;
    msg.name.push_back("joint_" + std::to_string(id));
    msg.position.push_back(motor_counts_to_deg(fb->position) * M_PI / 180.0);
    msg.velocity.push_back(fb->velocity * M_PI / 30.0);  // RPM→rad/s
    msg.effort.push_back(motor_ma_to_a(fb->current_iq));
    pub->publish(msg);
}

void cmd_cb(const sensor_msgs::JointState::ConstPtr& msg, motor_hal_t *hal) {
    for (size_t i = 0; i < msg->name.size(); i++) {
        float deg = msg->position[i] * 180.0 / M_PI;
        motor_hal_set_position(hal, i + 1, deg);
    }
}
```

## 许可证

MIT
