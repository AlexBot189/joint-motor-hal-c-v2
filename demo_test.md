# 查看状态
./stark_tool stat
# 持续打印反馈 (200ms)
./stark_tool watch
# 使能电机
./stark_tool enable 1    # 电机1
./stark_tool enable 2    # 电机2
# 电流控制
./stark_tool mode 1 5    # 切电流模式
./stark_tool torque 1 500   # 电机1 500mA
# 双电机同时控制
./stark_tool multi 500 0 0 500 0 0   # 双电机各500mA
# 位置控制
./stark_tool mode 1 3    # 切CSP模式
./stark_tool abs 1 30    # 电机1 转到30度
# 急停
./stark_tool stop


# ===== 板子侧 =====

# 1. 配 CANFD
ip link set can0 down
ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
ip link set can0 up

# 2. 电机上电

# 3. 启动节点
sudo ./stark_periph_manager_node -c /data/config/stark/stark_config.json

# 4. 看日志, 等"entering RUNNING"出现

# ===== 另一个终端 =====

# 5. 查看状态
./stark_tool stat
# 应该看到: State: RUNNING  Calib: done  Motor: [1] pos=XX [2] pos=XX

# 6. 使能 + 小电流测试
./stark_tool enable 1
./stark_tool mode 1 5
./stark_tool torque 1 500       # 500mA, 电机应该微动

# 7. 开始监控
./stark_tool watch 200           # 实时看反馈

# 8. 跑算法 demo
./demo_algo torque               # 正弦波扫描

# 9. 停止
./stark_tool stop




#算法控制demo
# 电流控制 (正弦波, =振幅)
./demo_algo torque 200        # ±200mA

# 速度控制 (梯形波, =峰值)
./demo_algo speed 10          # ±10RPM

# 位置控制 (方波, =振幅)
./demo_algo pos 15            # ±15°

# MIT 阻抗
./demo_algo mit 100 50        # kp=100 kd=50

# 双电机恒定电流
./demo_algo multi 200 200     # M1=200mA M2=200mA

# 只读反馈, 不控制
./demo_algo stat
