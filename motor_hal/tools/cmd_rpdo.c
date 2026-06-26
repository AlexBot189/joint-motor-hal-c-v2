/**
 * @file cmd_rpdo.c
 * @brief 标准 RPDO 发送命令
 *
 * 用法:
 *   motor_tool rpdo_send <id> <hex_bytes...>
 *
 * 前提: 已通过 motor_tool rpdo_map 配置好映射。
 * 数据按 RPDO 映射顺序, 小端排列。
 *
 * 示例 — 映射: Controlword(16b) + TargetPos(32b):
 *   motor_tool rpdo_send 1 0F00 00004000  # CW=EnableOp, Pos=16384cnt(90°)
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int cmd_do_rpdo_send(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    /* rpdo_send <id> <bytes...> */
    if (argc < 4) {
        fprintf(stderr, "Usage: motor_tool rpdo_send <id> <hex_bytes...>\n");
        fprintf(stderr, "  Data bytes in hex, little-endian, matching RPDO map order.\n");
        fprintf(stderr, "Example:\n");
        fprintf(stderr, "  # Map: Controlword(16b) + TargetPos(32b) = 6 bytes\n");
        fprintf(stderr, "  motor_tool rpdo_send 1 0F00 00004000\n");
        return -1;
    }

    int id = atoi(argv[2]);

    /* 收集 hex 字节 */
    uint8_t data[8];
    int dlc = 0;
    for (int i = 3; i < argc && dlc < 8; i++) {
        const char *hex = argv[i];
        size_t len = strlen(hex);
        if (len % 2 != 0) {
            fprintf(stderr, "ERROR: hex string '%s' has odd length\n", hex);
            return -1;
        }
        for (size_t j = 0; j < len && dlc < 8; j += 2) {
            unsigned int byte;
            if (sscanf(hex + j, "%2x", &byte) != 1) {
                fprintf(stderr, "ERROR: invalid hex '%c%c'\n", hex[j], hex[j+1]);
                return -1;
            }
            data[dlc++] = (uint8_t)byte;
        }
    }

    if (dlc == 0) {
        fprintf(stderr, "ERROR: no data bytes provided\n");
        return -1;
    }

    /* 打印发送内容 */
    printf("RPDO %d: COB=0x%03X dlc=%d [", id, 0x200 + id, dlc);
    for (int i = 0; i < dlc; i++) printf(" %02X", data[i]);
    printf(" ]\n");

    int ret = tool_rpdo_send((uint8_t)id, data, (uint8_t)dlc);
    if (ret == 0) printf("✓ RPDO sent\n");
    else fprintf(stderr, "✗ RPDO send failed (ret=%d)\n", ret);
    return ret;
}
