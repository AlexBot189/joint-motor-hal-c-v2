/**
 * @file cmd_sdo.c
 * @brief SDO 控制命令: sdo cur / sdo vel / sdo pos
 *
 * 用法:
 *   motor_tool sdo cur <N> <mA>           单电机电流
 *   motor_tool sdo cur <N1> <N2> <mA>     双电机电流(同值)
 *   motor_tool sdo cur <N1> <N2> <mA1> <mA2> 双电机电流(异值)
 *
 * 自动使能 + 切模式 + 写目标
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================
 * sdo cur <N> <mA>  或  sdo cur <N1> <N2> <mA>  或  sdo cur <N1> <N2> <mA1> <mA2>
 * ================================================================ */

int cmd_do_sdo(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    if (argc < 5) {
        fprintf(stderr, "Usage: motor_tool sdo cur <N> <mA>\n");
        fprintf(stderr, "       motor_tool sdo cur <N1> <N2> <mA>\n");
        fprintf(stderr, "       motor_tool sdo cur <N1> <N2> <mA1> <mA2>\n");
        return -1;
    }

    const char *mode = argv[2];

    if (strcmp(mode, "cur") == 0) {
        int n1, n2, ma1, ma2, is_dual;

        if (argc == 5) {
            /* sdo cur <N> <mA> */
            n1 = atoi(argv[3]);
            ma1 = atoi(argv[4]);
            n2 = 0; ma2 = 0; is_dual = 0;
        } else if (argc == 6) {
            /* sdo cur <N1> <N2> <mA> */
            n1 = atoi(argv[3]);
            n2 = atoi(argv[4]);
            ma1 = ma2 = atoi(argv[5]);
            is_dual = 1;
        } else if (argc == 7) {
            /* sdo cur <N1> <N2> <mA1> <mA2> */
            n1 = atoi(argv[3]);
            n2 = atoi(argv[4]);
            ma1 = atoi(argv[5]);
            ma2 = atoi(argv[6]);
            is_dual = 1;
        } else {
            fprintf(stderr, "ERROR: too many args\n");
            return -1;
        }

        return tool_sdo_cur(n1, n2, ma1, ma2, is_dual);
    }

    fprintf(stderr, "Unsupported mode: %s (use: cur)\n", mode);
    return -1;
}
