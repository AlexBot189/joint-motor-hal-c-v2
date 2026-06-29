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
 *   ,  TPDO1 node=1, COB=0x181, 每个SYNC发一次, 映射 Statusword(16b)+Position(32b)
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

/* ================================================================
 * 快捷映射命令: tpdo / rpdo
 *
 * 用法:
 *   motor_tool tpdo <id> <sync_count> <item1> [item2] [item3] ...
 *   motor_tool rpdo <id> <trans_type> <item1> [item2] ...
 *
 *   TPDO items: cur(pos) / vel / status / temp / err
 *   RPDO items: cw(controlword) / pos / cur / vel (通常只选一个目标参数)
 *
 * 示例:
 *   motor_tool tpdo 1 1 cur pos vel          # TPDO: Current+Position+Velocity
 *   motor_tool rpdo 1 255 cw pos             # RPDO: Controlword+TargetPosition
 *   motor_tool rpdo 1 255 cw cur             # RPDO: Controlword+TargetCurrent
 * ================================================================ */

static int _lookup_item(const char *name, uint16_t *index, uint8_t *bitlen, pdo_type_t type)
{
    if (strcmp(name, "cur") == 0) {
        *index = (type == PDO_TYPE_TPDO) ? OD_CURRENT_ACTUAL : OD_TARGET_TORQUE;
        *bitlen = 16;
    } else if (strcmp(name, "vel") == 0) {
        *index = (type == PDO_TYPE_TPDO) ? OD_VELOCITY_ACTUAL : OD_TARGET_VELOCITY;
        *bitlen = 32;
    } else if (strcmp(name, "pos") == 0) {
        *index = (type == PDO_TYPE_TPDO) ? OD_POSITION_ACTUAL : OD_TARGET_POS;
        *bitlen = 32;
    } else if (strcmp(name, "status") == 0) {
        if (type != PDO_TYPE_TPDO) return -1;
        *index = OD_STATUSWORD; *bitlen = 16;
    } else if (strcmp(name, "temp") == 0) {
        if (type != PDO_TYPE_TPDO) return -1;
        *index = 0x2663; *bitlen = 16;  /* 电机线圈温度 */
    } else if (strcmp(name, "err") == 0) {
        if (type != PDO_TYPE_TPDO) return -1;
        *index = 0x603F; *bitlen = 16;  /* 故障码 */
    } else if (strcmp(name, "cw") == 0) {
        if (type != PDO_TYPE_RPDO) return -1;
        *index = OD_CONTROLWORD; *bitlen = 16;
    } else {
        return -1;
    }
    return 0;
}

static int _do_pdo_quick(uint8_t id, pdo_type_t type,
                         uint32_t cob_id, uint8_t trans_type,
                         int argc, char **argv, int start)
{
    int count = argc - start;
    if (count < 1 || count > 8) {
        fprintf(stderr, "ERROR: 1~8 items required\n");
        return -1;
    }

    pdo_map_entry_cfg_t entries[8];
    for (int i = 0; i < count; i++) {
        uint16_t idx; uint8_t bits;
        if (_lookup_item(argv[start + i], &idx, &bits, type) != 0) {
            fprintf(stderr, "ERROR: unknown item '%s'\n", argv[start + i]);
            return -1;
        }
        entries[i].index  = idx;
        entries[i].subidx = 0x00;
        entries[i].bitlen = bits;
    }

    const char *dir = (type == PDO_TYPE_TPDO) ? "TPDO" : "RPDO";
    printf("Configuring %s: node=%d COB=0x%03X ttype=%d items=", dir, id, cob_id, trans_type);
    for (int i = 0; i < count; i++) printf("%s ", argv[start + i]);
    printf("\n");

    int ret = tool_pdo_map(id, type, entries, (uint8_t)count,
                           cob_id, trans_type);
    if (ret == 0) printf("✓ %s configured\n", dir);
    else fprintf(stderr, "✗ %s config failed (ret=%d)\n", dir, ret);
    return ret;
}

int cmd_do_tpdo_quick(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 5) {
        fprintf(stderr, "Usage: motor_tool tpdo <id> <sync_count> <item> [item...]\n");
        fprintf(stderr, "  TPDO items: cur vel pos status temp err\n");
        fprintf(stderr, "  sync_count: 1=every SYNC, 2=every 2nd SYNC, ...\n");
        fprintf(stderr, "Examples:\n");
        fprintf(stderr, "  motor_tool tpdo 1 1 cur pos vel       # Current+Position+Velocity\n");
        fprintf(stderr, "  motor_tool tpdo 1 1 status pos cur    # Statusword+Position+Current\n");
        return -1;
    }

    int id         = atoi(argv[2]);
    int sync_count = atoi(argv[3]);
    return _do_pdo_quick((uint8_t)id, PDO_TYPE_TPDO,
                         (uint32_t)(0x180 + id), (uint8_t)sync_count,
                         argc, argv, 4);
}

int cmd_do_rpdo_quick(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;
    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 5) {
        fprintf(stderr, "Usage: motor_tool rpdo <id> <trans_type> <item> [item...]\n");
        fprintf(stderr, "  RPDO items: cw pos cur vel\n");
        fprintf(stderr, "  通常只选一个目标参数 (cw + 当前模式对应的目标)\n");
        fprintf(stderr, "  trans_type: 255=async, 1~240=sync, 0=sync_acyclic\n");
        fprintf(stderr, "Examples:\n");
        fprintf(stderr, "  motor_tool rpdo 1 255 cw pos           # CSP: CW+TargetPosition\n");
        fprintf(stderr, "  motor_tool rpdo 1 255 cw cur           # 电流: CW+TargetCurrent\n");
        fprintf(stderr, "  motor_tool rpdo 1 255 cw vel           # 速度: CW+TargetVelocity\n");
        return -1;
    }

    int id        = atoi(argv[2]);
    int trans_type = atoi(argv[3]);
    return _do_pdo_quick((uint8_t)id, PDO_TYPE_RPDO,
                         (uint32_t)(0x200 + id), (uint8_t)trans_type,
                         argc, argv, 4);
}
