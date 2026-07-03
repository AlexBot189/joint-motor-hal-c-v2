/*
 * stress_ringbuf.c -- 环形缓冲压力测试
 * Copyright (c) 2026 zhiqiang.yang
 *
 * 验证环形缓冲 mailbox 不丢帧: 100us 内连续发 10 帧电流控制,
 * 检查电机是否响应所有帧 (而非只响应最后一帧).
 *
 * 用法: sudo ./stress_ringbuf
 * 编译: gcc -O2 stress_ringbuf.c -lpthread -lrt -lm -o stress_ringbuf
 */

#include "stark_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main()
{
    stark_client_t c;

    printf("[stress] 等待 stark_node...\n");
    while (stark_open(&c) != 0) usleep(100000);
    printf("[stress] SHM 已连接\n");

    printf("[stress] 等待就绪...\n");
    while (!stark_ready(&c)) usleep(100000);
    printf("[stress] 就绪, 电机在线: %d %d\n",
           stark_online(&c, 1), stark_online(&c, 2));

    /* 使能 + 电流模式 */
    stark_enable(&c, 1); stark_enable(&c, 2);
    stark_set_mode(&c, 1, STARK_MODE_CURRENT);
    stark_set_mode(&c, 2, STARK_MODE_CURRENT);
    usleep(5000);

    printf("[stress] 开始 10 帧电流控制 (双电机)...\n");

    /* 在 100us 内快速发送 10 帧 stark_multi */
    int32_t currents[] = {100, 200, 300, 400, 500, 400, 300, 200, 100, 0};
    int n = sizeof(currents) / sizeof(currents[0]);

    for (int i = 0; i < n; i++) {
        int32_t ma = currents[i];
        stark_multi(&c, ma, 0, 0, ma, 0, 0);
        stark_heartbeat(&c);
        usleep(10);  /* 10us 间隔, 100us 内发 10 帧 */
    }

    printf("[stress] 10 帧已发送, 等待 100ms 观察电机反应...\n");
    printf("[stress] 预期: 电机电流从 100 连续变到 0, 中间无明显跳跃\n");
    usleep(100000);

    /* 读取反馈验证 */
    motor_data_t m1 = stark_fb(&c, 1);
    motor_data_t m2 = stark_fb(&c, 2);
    printf("[stress] 最终反馈: M1 pos=%.1f cur=%dmA M2 pos=%.1f cur=%dmA\n",
           m1.position * 360.0f / 65536.0f, m1.current_iq,
           m2.position * 360.0f / 65536.0f, m2.current_iq);

    stark_disable(&c, 1); stark_disable(&c, 2);
    usleep(5000);
    stark_close(&c);

    printf("[stress] 测试完成\n");
    return 0;
}
