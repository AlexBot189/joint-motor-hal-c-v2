# Known Issues

## [SDO-001] 同一电机并发 SDO 读写会误杀响应帧

**状态**: 已知, 未修复

**影响范围**: `sdo_client.c` — `_sdo_wait_response` / `sdo_push_response`

**问题描述**:

两个线程同时对同一电机(ID)发起 SDO 读不同对象时, 响应帧会被误杀。
SDO 同一电机的收/发共用 COB-ID (0x580+ID), 队列用 COB-ID + index 匹配。
当前代码在 `_sdo_wait_response` 中扫描到同 COB-ID 但不同 index 的帧时会直接丢弃,
导致另一线程的响应帧丢失。

```
线程 A: sdo_read(电机 1, 0x6041) → 等 COB 0x581, index=0x6041
线程 B: sdo_read(电机 1, 0x6064) → 等 COB 0x581, index=0x6064
                                 ↓
                        A 扫到 B 的 0x6064 帧 → "不匹配" → 丢弃 ❌
```

**触发条件**:
- 同一电机, 两个线程同时走 SDO 路径
- 不同电机不受影响 (COB-ID 不同, 天然隔离)

**当前规避**:
- exo_node 运行时: 启动阶段主线程单线程 SDO, 运行阶段全部 PDO + 共享内存
- motor_tool daemon: 单线程 accept 循环, 命令串行执行
- **不触发此问题**

**修复方案** (未实施):

1. `sdo_push_response` 中 `pthread_cond_signal` → `pthread_cond_broadcast`
2. `_sdo_wait_response` 中不匹配帧不丢弃, 改为 `continue` (跳过留给其他线程)

改动约 20 行, 仅 `sdo_client.c`, 风险低。

**发现日期**: 2026-06-11
**发现人**: 阳志强 / 张君宝
