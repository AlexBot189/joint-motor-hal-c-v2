/**
 * @file cmd_pdo.c
 * @brief PDO 映射命令: tpdo_map / rpdo_map
 *
 * 用法:
 *   motor_tool tpdo_map <id> <cob_hex> <trans_type> <0xIdx1> <sub1> <bits1> [<0xIdx2> <sub2> <bits2> ...]
 *   motor_tool rpdo_map <id> <cob_hex> <trans_type> <0xIdx1> <sub1> <bits1> [<0xIdx2> <sub2> <bits2> ...]
 *
 * 示例:
 *   motor_tool tpdo_map 1 0x181 1 0x6041 0 16 0x6064 0 32
 *   → TPDO1 node=1, COB=0x181, 每个SYNC发一次, 映射 Statusword(16b)+Position(32b)
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 * tpdo_map <id> <cob_hex> <trans_type> <idx1> <sub1> <bits1> ...
 * ================================================================ */

int cmd_do_tpdo_map(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    /* 最小: tpdo_map <id> <cob> <ttype> <idx> <sub> <bits> = 7 args */
    if (argc < 7) {
        fprintf(stderr, "Usage: motor_tool tpdo_map <id> <cob_hex> <trans_type> "
                "<0xIdx1> <sub1> <bits1> [<0xIdx2> <sub2> <bits2> ...]\n");
        fprintf(stderr, "  trans_type: 0=sync acyclic, 1~240=sync every N, 254/255=async\n");
        fprintf(stderr, "  bits: 8/16/32\n");
        fprintf(stderr, "Example: motor_tool tpdo_map 1 0x181 1 0x6041 0 16 0x6064 0 32\n");
        return -1;
    }

    int      id        = atoi(argv[2]);
    uint32_t cob_id    = (uint32_t)strtoul(argv[3], NULL, 0);
    uint8_t  trans_type = (uint8_t)atoi(argv[4]);

    /* 解析映射条目: (argc - 5) / 3 个条目 */
    int entry_count = (argc - 5) / 3;
    if (entry_count <= 0 || entry_count > 8) {
        fprintf(stderr, "ERROR: 1~8 PDO map entries required, got %d\n", entry_count);
        return -1;
    }

    pdo_map_entry_cfg_t entries[8];
    for (int i = 0; i < entry_count; i++) {
        entries[i].index  = (uint16_t)strtoul(argv[5 + i * 3], NULL, 0);
        entries[i].subidx = (uint8_t)atoi(argv[6 + i * 3]);
        entries[i].bitlen = (uint8_t)atoi(argv[7 + i * 3]);

        if (entries[i].bitlen != 8 && entries[i].bitlen != 16 && entries[i].bitlen != 32) {
            fprintf(stderr, "ERROR: entry %d bitlen=%d invalid (8/16/32)\n",
                    i, entries[i].bitlen);
            return -1;
        }
    }

    printf("Configuring TPDO: node=%d COB=0x%03X ttype=%d entries=%d\n",
           id, cob_id, trans_type, entry_count);
    for (int i = 0; i < entry_count; i++) {
        printf("  [%d] 0x%04X.%02X @%db\n",
               i, entries[i].index, entries[i].subidx, entries[i].bitlen);
    }

    int ret = tool_pdo_map((uint8_t)id, PDO_TYPE_TPDO, entries, entry_count,
                           cob_id, trans_type);
    if (ret == 0) printf("✓ TPDO configured\n");
    else fprintf(stderr, "✗ TPDO config failed (ret=%d)\n", ret);
    return ret;
}

/* ================================================================
 * rpdo_map <id> <cob_hex> <trans_type> <idx1> <sub1> <bits1> ...
 * ================================================================ */

int cmd_do_rpdo_map(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 7) {
        fprintf(stderr, "Usage: motor_tool rpdo_map <id> <cob_hex> <trans_type> "
                "<0xIdx1> <sub1> <bits1> [<0xIdx2> <sub2> <bits2> ...]\n");
        fprintf(stderr, "  trans_type: 0=sync acyclic, 254/255=async\n");
        fprintf(stderr, "  bits: 8/16/32\n");
        fprintf(stderr, "Example: motor_tool rpdo_map 1 0x201 255 0x6040 0 16 0x607A 0 32\n");
        return -1;
    }

    int      id        = atoi(argv[2]);
    uint32_t cob_id    = (uint32_t)strtoul(argv[3], NULL, 0);
    uint8_t  trans_type = (uint8_t)atoi(argv[4]);

    int entry_count = (argc - 5) / 3;
    if (entry_count <= 0 || entry_count > 8) {
        fprintf(stderr, "ERROR: 1~8 PDO map entries required, got %d\n", entry_count);
        return -1;
    }

    pdo_map_entry_cfg_t entries[8];
    for (int i = 0; i < entry_count; i++) {
        entries[i].index  = (uint16_t)strtoul(argv[5 + i * 3], NULL, 0);
        entries[i].subidx = (uint8_t)atoi(argv[6 + i * 3]);
        entries[i].bitlen = (uint8_t)atoi(argv[7 + i * 3]);

        if (entries[i].bitlen != 8 && entries[i].bitlen != 16 && entries[i].bitlen != 32) {
            fprintf(stderr, "ERROR: entry %d bitlen=%d invalid (8/16/32)\n",
                    i, entries[i].bitlen);
            return -1;
        }
    }

    printf("Configuring RPDO: node=%d COB=0x%03X ttype=%d entries=%d\n",
           id, cob_id, trans_type, entry_count);
    for (int i = 0; i < entry_count; i++) {
        printf("  [%d] 0x%04X.%02X @%db\n",
               i, entries[i].index, entries[i].subidx, entries[i].bitlen);
    }

    int ret = tool_pdo_map((uint8_t)id, PDO_TYPE_RPDO, entries, entry_count,
                           cob_id, trans_type);
    if (ret == 0) printf("✓ RPDO configured\n");
    else fprintf(stderr, "✗ RPDO config failed (ret=%d)\n", ret);
    return ret;
}
