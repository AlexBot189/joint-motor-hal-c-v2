# motor_tool 使用手册

## 概述

motor_tool 是基于 motor_hal_c 的 CANFD 电机控制命令行工具，支持 SDO/PDO 双路径控制。
通过 Unix Domain Socket 与 daemon 通信，也可 client 模式直接发命令。

## 启动

```bash
motor_tool daemon can0 &       # 启动守护进程，挂载 can0

motor_tool help                # 查看全部命令
```

## 一、SDO 控制

SDO 走 CANopen 标准协议，每条命令自动完成使能、切模式、设参数、发目标值，有应答确认。
适合非实时场景：调试、校准、参数配置。

```
torque  <id> <mA>              电流控制，0~20000mA
        例: torque 1 500       电机1: 500mA

speed   <id> <rpm> [acc] [dec] 速度控制，RPM
        例: speed 1 50         电机1: 50RPM
        例: speed 1 50 100 100 电机1: 50RPM，加减速100RPM/s

abs     <id> <deg>             绝对位置，度
        例: abs 1 45           电机1: 45度

setzero <id>                   零位标定（自动失能后写零位）

pid     <id> <cp> <ci> <vp> <vi> <pp> <pi>  设置PID参数
```

## 二、PDO 控制

PDO 直发 CANFD 帧，无应答，延迟 < 0.5ms。适合实时控制。
需先 setmode 和 pdo_enable 设置好模式和使能状态。

```
pdo     <id> <pos|vel|cur|csp> <val>      单轴PDO
        例: pdo 1 cur 500                 电机1: 电流500mA
        例: pdo 1 pos 3000                电机1: 位置30度 (单位0.01度)
        例: pdo 1 vel 5000                电机1: 速度50RPM (单位0.01RPM)

multi   <pos|vel|cur|csp> <id:val> ...    多轴广播，一帧CANFD发多电机
        例: multi cur 1:500 2:500         双电机各500mA

mit     <id> <pos> <vel> <kp> <kd> <tor>  MIT阻抗控制
        例: mit 1 0 0 500 100 0           kp=500, kd=100, 目标位置0
```

## 三、PDO Byte0 控制

修改 PDO 控制字的 bit 位，不发目标值。字节内容在下一次 PDO 帧中生效。

```
pdo_enable   <id>   使能 (Byte0 bit7=1)
pdo_disable  <id>   失能 (Byte0 bit7=0)
estop        <id>   急停 (enable=0, bus=OFF)
recover      <id>   恢复 (enable=1, bus=ON)
clearcf      <id>   清故障脉冲 (Byte0 bit5)
setmode      <id> <1~6>  切换控制模式
                        1=PP(轮廓位置) 2=PV(轮廓速度)
                        3=CSP(循环同步位置) 4=CSV(循环同步速度)
                        5=Current(电流环) 6=MIT(阻抗)
byte0        <id> [0xHH] 读/写原始 Byte0 值，不填参数为读
```

## 四、读取

```
read angle   <id>   角度 (编码器 counts)
read speed   <id>   速度 (RPM)
read current <id>   Q轴电流 (mA)
read temp    <id>   温度 (°C)
read state   <id>   电机状态 (DS402状态字)
read error   <id>   故障码
read version <id>   固件版本
read all     <id>   全部信息

sensor read  <id>   霍尔/力矩/膝关节/着地开关
```

## 五、通用 SDO 读写

```
sdoread  <id> <0xIndex> [sub]            读任意 OD
    例: sdoread 1 0x6061                读当前模式

sdowrite <id> <0xIndex> <sub> <val> <size>  写任意 OD
    例: sdowrite 1 0x6071 0 500 2       写目标电流500mA (U16)
    例: sdowrite 1 0x6040 0 0x0F 2      写控制字15=使能 (U16)
```

## 六、常用 OD 地址

```
0x6040  控制字       R/W, U16   6=Shutdown 7=Switch on 15=Enable operation
0x6041  状态字       R,   U16   通过状态字判电机状态
0x6060  运行模式     R/W, S8    写模式值
0x6061  当前模式     R,   S8    读运行的模式
0x6071  目标电流     R/W, S16   单位 mA
0x607A  目标位置     R/W, S32   单位 encoder counts
0x60FF  目标速度     R/W, S32   单位 RPM
0x6064  实际位置     R,   S32
0x606C  实际速度     R,   S32
0x6083  轮廓加速度   R/W, U16   单位 RPM/s
0x6084  轮廓减速度   R/W, U16
0x6081  轮廓速度     R/W, U16   位置模式下的最大速度
0x2531  零位标定     W,   U32   写1完成标定
0x2539  保存Flash    W,   U32   写1保存
0x1017  心跳周期     R/W, U16   单位 ms
0x100A  固件版本     R,   U32
```

## 七、校准

```
calib start <id_r> <id_l> [timeout_ms]  启动校准，默认10s超时
    例: calib start 1 2 10000          校准电机1、2，阈值10s

calib status                            查看校准状态

calib exit                              退出校准
```

## 八、传感器透传

```
sensor config <id> <period_div>         开启透传
    例: sensor config 1 250            电机1, 250us采样周期

sensor watch  <id>                      实时看板 (Ctrl+C 停)

sensor read   <id>                      读一次传感器缓存

sensor stop   <id>                      停止透传
```

## 九、监控与数据上报

```
watch   [period_ms]                     持续轮询反馈 (安全可 Ctrl+C)
    例: watch 200                       200ms 刷新

report  [period_ms]                     数据上报 (0=停止)
    例: report 5                        5ms 上报 (200Hz)
```

## 十、生命周期

```
init     <can_iface>                    初始化 CANFD 接口
startup  <id>                           上电启动 (Bootup, NMT, DS402使能)
enable   <id>                           使能电机
disable  <id>                           脱使能
reboot   <id>                           电机重启
save     <id>                           参数保存到 Flash
probe    [id]                           探测电机是否在线
stop                                     停止 daemon
```

## 十一、典型调试流程

```bash
# 1. 启动 daemon (只需一次)
motor_tool daemon can0 &

# 2. 电机上电，等自动启动完成
#    看日志确认 auto-startup 成功

# 3. 验证电机状态
motor_tool read state 1        # 期望: OPERATION_ENABLED
motor_tool read all 1          # 看全部参数

# 4. 校准零位
motor_tool calib start 1 2
motor_tool calib status        # 确认 DONE

# 5. 开传感器透传
motor_tool sensor config 1 250
motor_tool sensor config 2 250

# 6. SDO 控制验证 (有应答)
motor_tool torque 1 300        # 电流 300mA
motor_tool read current 1      # 看实际电流是否匹配
motor_tool abs 1 45            # 转到 45 度

# 7. PDO 控制验证 (直发)
motor_tool setmode 1 5         # 切电流模式
motor_tool pdo_enable 1        # PDO 使能
motor_tool pdo 1 cur 300       # PDO 电流 300mA

# 8. 多轴广播
motor_tool setmode 1 5
motor_tool setmode 2 5
motor_tool multi cur 1:300 2:300

# 9. 持续监控另一终端
motor_tool watch 200

# 10. 急停
motor_tool estop 1
motor_tool estop 2
```

## 十二、SDO 和 PDO 选择

| 特性 | SDO (torque/speed/abs) | PDO (pdo/multi/mit) |
|---|---|---|
| 协议 | CANopen 标准，有应答 | 自定义帧，无应答 |
| 延迟 | 单次 2-10ms (等应答) | 单次 < 0.5ms |
| 自动使能 | 是 (完整时序) | 否 (需手动 setmode+pdo_enable) |
| 适用场景 | 调试、校准、参数配置 | 实时控制循环 |
| 控制精度 | SDO 每次读回确认 | 无确认，信任帧送达 |

调试先用 SDO，确认电机能正常响应后切 PDO 做实时控制。

## 十三、故障排查

```
# 电机不响应
motor_tool probe               # 探测在线
motor_tool read state 1         # 查状态
sdoread 1 0x6041                # 读状态字原始值

# 电流不匹配
motor_tool read current 1       # 读实际电流
sdoread 1 0x6071                # 读目标电流寄存器

# 模式不对
motor_tool read mode 1          # 读当前模式
sdoread 1 0x6061                # 读原始模式值

# 故障
motor_tool read error 1         # 读故障码
motor_tool fault_reset 1        # 清零故障
```
