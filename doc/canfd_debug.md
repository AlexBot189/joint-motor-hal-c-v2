# CAN FD 调试文档

> 适用平台: RK3576 CANFD 控制器 (兼容 RV1126B)  
> 内核驱动: rk3576_canfd  
> 最后更新: 2026-08-03

---

## 1. 问题描述

RV1126B 接收 CAN FD 帧时，只收到 8 字节有效数据，后 24 字节全为零。

**现象:**
```
candump: can0 6C1 [32] FA A6 6B 47 05 00 54 56 00 00 00 ... 00 00
                                              └── 后24字节全零 ──┘
# CAN FD 分析仪直连电机: 32 字节数据完整
```

---

## 2. 根因分析

### 2.1 DTS 配置

```dts
&can0 {
    rockchip,rx-max-data = <4>;   /* 关键: 4 个 word → 仅 8 字节 RX */
};
```

### 2.2 驱动代码

`rv1126b_canfd.c` → `rk3576_canfd_rx()`:

```c
u32 data[16] = {0};

dlc = rk3576_canfd_read(rcan, addr);        // header word 0
id  = rk3576_canfd_read(rcan, addr);        // header word 1

for (i = 0; i < (rcan->rx_max_data - 2); i++)
    data[i] = rk3576_canfd_read(rcan, addr);

// rx_max_data=4 → 读 4-2=2 words = 8 字节
// rx_max_data=18 → 读 18-2=16 words = 64 字节
```

`rk3576_canfd_start()`:

```c
if (rcan->rx_max_data > 4)
    ism = CANFD_DATA_CANFD_FIXED;     /* CAN FD 固定大小模式 */
else
    ism = CANFD_DATA_CAN_FIXED;       /* Classic CAN 模式 ← 当前 */
```

`rk3576_canfd_probe()` 驱动 probe 中只支持两个值:

```c
if (rcan->rx_max_data != 4 && rcan->rx_max_data != 18) {
    rcan->rx_max_data = 18;
    dev_warn(&pdev->dev, "rx_max_data is invalid, set to 18 words!\n");
}
```

### 2.3 原因

DTS 配 `rx-max-data = <4>` → 驱动配置 Classic CAN 固定模式 → 每帧只读 2 个数据 word = 8 字节。CAN FD 头部的 DLC 能正确识别（显示 [32]），但 payload 后续 24 字节未从 FIFO 读出。

**回环测试能通过 >8 字节** 的原因是回环模式下控制器内部自同步，不经过实际总线收发 buffer 的格式化路径。

---

## 3. 解决方案

### 3.1 DTS 修改

```dts
&can0 {
    compatible = "rockchip,rk3576-canfd";  /* 或 rv1126b-canfd */
    rockchip,rx-max-data = <18>;           /* 改: 4 → 18 */
};
```

### 3.2 影响

FIFO 深度变化:

```
SRAM_MAX_DEPTH = 256 words (内部 SRAM)

rx_max_data=4  → fifo_depth = 256/4  = 64 frames
rx_max_data=18 → fifo_depth = 256/18 = 14 frames
```

14 帧在 1KHz 周期下通过 NAPI poll 能够清空，不会有丢帧问题。

### 3.3 验证

修改 DTS、编译 DTB、烧录后验证:

```bash
# 确认模式生效
dmesg | grep "CAN info"
# 应输出: CAN info: use_dma=... rx_max_data=18 fifo_depth=14

# 抓 0x6C0 聚合帧验证
candump can0 | grep '6C1\|6C2'
# 应显示 32 字节都有有效数据
```

---

## 4. 诊断命令集

### 4.1 检查 CAN FD 状态

```bash
# 完整链路信息
ip -d link show can0
# 关键字段: <FD>, bitrate, dbitrate, dsample-point, berr-counter

# 检查 TX/RX 错误计数
ip -d -s link show can0 | grep berr

# 查看驱动日志
dmesg | grep -iE "can|rockchip|fd"

# 查看控制器兼容性
cat /proc/device-tree/can@*/compatible 2>/dev/null | tr '\0' ' '
```

### 4.2 CAN FD 接口配置

```bash
# 关闭接口
ip link set can0 down

# 配置 CAN FD 1M/5M + BRS
ip link set can0 type can bitrate 1000000 sample-point 0.8 \
    dbitrate 5000000 dsample-point 0.75 fd on

# 加大 sjw (容差)
ip link set can0 type can bitrate 1000000 dbitrate 5000000 \
    fd on dsjw 4

# 降速测试 (排除时序问题)
ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on

# CAN FD 同速率 (无 BRS)
ip link set can0 type can bitrate 1000000 dbitrate 1000000 fd on

# 启动
ip link set can0 up
```

### 4.3 抓帧分析

```bash
# 只抓 0x6C0 聚合帧
candump can0,6C0:7F0

# 抓 0x300 反馈 + 0x6C0 透传
candump can0,300:7F0,6C0:7F0

# 带时间戳
candump -t A can0

# 过滤特定 ID
candump can0 | grep -E '301|302|6C1|6C2'
```

---

## 5. 常见问题排查

| 现象 | 可能原因 | 排查 |
|------|---------|------|
| `ip link show` 无 `<FD>` | 硬件/驱动不支持 CAN FD | 检查内核配置+驱动 |
| candump `[32]` 但后 24 字节全零 | `rx-max-data=4` | 改为 18 |
| `berr-counter rx > 0` | 总线错误或阻抗不匹配 | 检查终端电阻 120Ω |
| CAN FD 发送失败 | `CAN_RAW_FD_FRAMES` 未使能 | 检查 socket 初始化 |
| candump 无输出 | 接口未 UP 或 bitrate 不匹配 | `ip link set can0 up` |
