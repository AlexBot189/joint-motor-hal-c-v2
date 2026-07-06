# motor_tool 单/双电机兼容性测试指令

## 环境准备

```bash
# 终端1: 启动 daemon
./motor_tool daemon can0

# 终端2: 执行测试命令
```

## 场景1: 只接1个电机 (电机1在线, 电机2不接)

### 1.1 启动电机1
```bash
./motor_tool startup 1
# 预期: OPERATION_ENABLED
```

### 1.2 广播命令 (id=0), 只应控电机1, 无延迟
```bash
# SDO 速度控制 (广播)
./motor_tool speed 0 5000 1000
# 预期: [1] 正常设置, 无 "[2] offline" 打印或无600ms延迟

# SDO 电流控制 (广播)
./motor_tool torque 0 1000
# 预期: 同上只控电机1

# SDO 位置控制 (广播)
./motor_tool abs 0 4500
# 预期: 同上

# 广播使能/脱使能
./motor_tool enable 0
# 预期: [1] enable OK, 无电机2 SDO超时

./motor_tool disable 0
# 预期: [1] disable OK, 无延迟

# 广播复位
./motor_tool reset 0
# 预期: 同上
```

### 1.3 读命令 (广播 id=0)
```bash
./motor_tool read all 0
# 预期: 只输出电机1信息, 无电机2 SDO超时

./motor_tool read angle 0
./motor_tool read speed 0
./motor_tool read current 0
./motor_tool read temp 0
./motor_tool read state 0
./motor_tool read error 0
# 预期: 各命令无600ms延迟
```

### 1.4 指定ID控制 (不广播)
```bash
./motor_tool speed 1 3000 1000
# 预期: 正常控电机1

./motor_tool speed 2 3000 1000
# 预期: WARN: motor 2 offline, skipping
```

### 1.5 持续监控
```bash
./motor_tool watch 500
# 预期: 显示电机1数据, 电机2显示 "-" 占位, 不卡顿
# Ctrl+C 退出
```

### 1.6 尝试 startup 不存在的电机, 验证清理
```bash
./motor_tool startup 2
# 预期: "Motor 2 startup failed", 电机2从广播列表移除

# 验证: 再次 broadcast 命令, 无 motor 2 相关输出
./motor_tool read all 0
# 预期: 只输出电机1
```

---

## 场景2: 接2个电机 (电机1和电机2都在线)

### 2.1 启动双电机
```bash
./motor_tool startup 0
# 预期: 两个电机都 OPERATION_ENABLED
```

或者单独启动:
```bash
./motor_tool startup 1
./motor_tool startup 2
```

### 2.2 同时控制两个电机 (广播 id=0)
```bash
# SDO 速度控制
./motor_tool speed 0 5000 1000
# 预期: [1] [2] 都设置成功

# SDO 电流控制
./motor_tool torque 0 2000
# 预期: [1] [2] 都设置成功

# SDO 位置控制
./motor_tool abs 0 9000
# 预期: [1] [2] 都移动到 90°

# 广播使能/脱使能
./motor_tool enable 0
./motor_tool disable 0
# 预期: 两电机同时操作
```

### 2.3 只控制单个电机
```bash
# 只控电机1
./motor_tool speed 1 3000 1000
# 预期: [1] 设置成功, [2] 不受影响

# 只控电机2
./motor_tool speed 2 -3000 1000
# 预期: [2] 设置成功, [1] 不受影响

# 只读电机1
./motor_tool read all 1
# 预期: 只输出电机1

# 只读电机2
./motor_tool read all 2
# 预期: 只输出电机2
```

### 2.4 读双电机
```bash
./motor_tool read all 0
# 预期: 输出两电机完整信息
```

### 2.5 PDO 实时控制 (验证不受影响)
```bash
# 单轴 PDO
./motor_tool pdo 1 pos 45
./motor_tool pdo 2 vel 30
./motor_tool pdo 1 cur 500

# 多轴广播 PDO
./motor_tool multi pos 1:45 2:-45
./motor_tool multi vel 1:50 2:-30
./motor_tool multi cur 1:1000 2:500
# 预期: 各命令即时生效, 无延迟
```

### 2.6 持续监控
```bash
./motor_tool watch 200
# 预期: 两电机数据实时更新
```

---

## 场景3: 接2个电机, 在线运行中拔掉1个

### 3.1 先启动双电机
```bash
./motor_tool startup 0
./motor_tool speed 0 3000 1000
# 两个都在转
```

### 3.2 物理拔掉电机1 (或关电)

### 3.3 广播命令, 确认电机2仍可控
```bash
./motor_tool speed 0 5000
# 预期: 电机2正常设置 (电机1 SDO超时会自动跳过, 不影响电机2)

./motor_tool disable 0
# 预期: 电机2成功脱使能

./motor_tool read all 0
# 预期: 电机2数据正常
```

### 3.4 指定ID控电机2
```bash
./motor_tool speed 2 2000 1000
# 预期: 电机2正常控制, 不受电机1离线影响
```

---

## 场景4: PDO Byte0 控制 (不走SDO, 不受离线影响)

```bash
# 使能
./motor_tool pdo_enable 1
./motor_tool pdo_enable 2

# 急停
./motor_tool estop 0
# 预期: 即时生效, 无延迟

# 恢复
./motor_tool recover 0
# 预期: 即时生效

# 脱使能
./motor_tool pdo_disable 1
./motor_tool pdo_disable 2
```

---

## 场景5: 传感器透传 & 校准

```bash
# 传感器
./motor_tool sensor config 1 250
./motor_tool sensor read 1
./motor_tool sensor watch 1
# Ctrl+C 退出
./motor_tool sensor stop 1

# 校准 (单电机)
./motor_tool calib start 1
# 校准 (双电机)
./motor_tool calib start 1 2

# 查看校准状态
./motor_tool calib status
```

---

## 场景6: 探测 & 通用 SDO

```bash
# 探测在线电机
./motor_tool probe 0
# 预期: 显示在线/离线状态

./motor_tool probe 1
# 预期: ONLINE 或 OFFLINE

# 通用 SDO 读
./motor_tool sdoread 1 0x100A
# 预期: 固件版本

# 通用 SDO 写 (慎用)
./motor_tool sdowrite 1 0x6040 0 0x0006 2
```

---

## 场景7: 停止和清理

```bash
# 脱使能所有
./motor_tool disable 0

# 停止 daemon
./motor_tool stop

# 确认进程已退出
ps aux | grep motor_tool
```

---

## 验证要点

| 检查项 | 方法 |
|--------|------|
| 广播命令不因离线电机延迟 | `speed 0` 后观察日志, 不应有600ms以上卡顿 |
| 指定ID控制不受干扰 | `speed 1` 和 `speed 2` 各自独立 |
| startup 失败自动清理 | `startup 2` 失败后, 后续广播不再引用电机2 |
| watch 持续显示 | 离线电机显示 "-", 不影响其他电机刷新 |
| PDO 命令不受影响 | `pdo`/`multi`/`estop` 即时响应 |
| enable/disable/reset 广播 | `enable 0` 等命令有在线过滤 |
| stark_node 不受影响 | 独立启动 stark_periph_node, 实时控制正常 |
