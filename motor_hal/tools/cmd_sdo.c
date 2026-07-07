/**
 * @file cmd_sdo.c
 * @brief SDO 控制命令: sdo cur / sdo vel / sdo pos / sdo csp / sdo csv
 *
 * 用法:
 *   motor_tool sdo cur <N> <mA>             单电机电流
 *   motor_tool sdo cur <N1> <N2> <mA>       双电机电流(同值)
 *   motor_tool sdo cur <N1> <N2> <mA1> <mA2> 双电机电流(异值)
 *   motor_tool sdo vel <N> <rpm>            单电机速度
 *   motor_tool sdo vel <N1> <N2> <rpm>      双电机速度(同值)
 *   motor_tool sdo vel <N1> <N2> <rpm1> <rpm2> 双电机速度(异值)
 *   motor_tool sdo pos/csp/csv ...          同理
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
        fprintf(stderr, "       motor_tool sdo vel <N> <rpm>\n");
        fprintf(stderr, "       motor_tool sdo vel <N1> <N2> <rpm>\n");
        fprintf(stderr, "       motor_tool sdo vel <N1> <N2> <rpm1> <rpm2>\n");
        fprintf(stderr, "       motor_tool sdo pos <N> <deg>\n");
        fprintf(stderr, "       motor_tool sdo pos <N1> <N2> <deg>\n");
        fprintf(stderr, "       motor_tool sdo pos <N1> <N2> <deg1> <deg2>\n");
        fprintf(stderr, "       motor_tool sdo csp <N> <deg>\n");
        fprintf(stderr, "       motor_tool sdo csv ...  (同理)\n");
        return -1;
    }

    const char *mode = argv[2];

    if (strcmp(mode, "cur") == 0) {
        int n1, n2, ma1, ma2, is_dual;
        if (argc == 5) {
            n1 = atoi(argv[3]); ma1 = atoi(argv[4]);
            n2 = 0; ma2 = 0; is_dual = 0;
        } else if (argc == 6) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            ma1 = ma2 = atoi(argv[5]); is_dual = 1;
        } else if (argc == 7) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            ma1 = atoi(argv[5]); ma2 = atoi(argv[6]); is_dual = 1;
        } else {
            fprintf(stderr, "ERROR: too many args\n"); return -1;
        }
        return tool_sdo_cur(n1, n2, ma1, ma2, is_dual);
    }

    if (strcmp(mode, "pos") == 0) {
        int n1, n2, is_dual;
        float deg1, deg2;
        if (argc == 5) {
            n1 = atoi(argv[3]); deg1 = (float)atof(argv[4]);
            n2 = 0; deg2 = 0; is_dual = 0;
        } else if (argc == 6) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            deg1 = deg2 = (float)atof(argv[5]); is_dual = 1;
        } else if (argc == 7) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            deg1 = (float)atof(argv[5]); deg2 = (float)atof(argv[6]); is_dual = 1;
        } else {
            fprintf(stderr, "ERROR: too many args\n"); return -1;
        }
        return tool_sdo_pos(n1, n2, deg1, deg2, is_dual);
    }

    if (strcmp(mode, "csp") == 0) {
        int n1, n2, is_dual;
        float deg1, deg2;
        if (argc == 5) {
            n1 = atoi(argv[3]); deg1 = (float)atof(argv[4]);
            n2 = 0; deg2 = 0; is_dual = 0;
        } else if (argc == 6) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            deg1 = deg2 = (float)atof(argv[5]); is_dual = 1;
        } else if (argc == 7) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            deg1 = (float)atof(argv[5]); deg2 = (float)atof(argv[6]); is_dual = 1;
        } else {
            fprintf(stderr, "ERROR: too many args\n"); return -1;
        }
        return tool_sdo_csp(n1, n2, deg1, deg2, is_dual);
    }

    if (strcmp(mode, "vel") == 0) {
        int n1, n2, rpm1, rpm2, is_dual;
        if (argc == 5) {
            n1 = atoi(argv[3]); rpm1 = atoi(argv[4]);
            n2 = 0; rpm2 = 0; is_dual = 0;
        } else if (argc == 6) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            rpm1 = rpm2 = atoi(argv[5]); is_dual = 1;
        } else if (argc == 7) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            rpm1 = atoi(argv[5]); rpm2 = atoi(argv[6]); is_dual = 1;
        } else {
            fprintf(stderr, "ERROR: too many args\n"); return -1;
        }
        return tool_sdo_vel(n1, n2, rpm1, rpm2, is_dual);
    }

    if (strcmp(mode, "csv") == 0) {
        int n1, n2, rpm1, rpm2, is_dual;
        if (argc == 5) {
            n1 = atoi(argv[3]); rpm1 = atoi(argv[4]);
            n2 = 0; rpm2 = 0; is_dual = 0;
        } else if (argc == 6) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            rpm1 = rpm2 = atoi(argv[5]); is_dual = 1;
        } else if (argc == 7) {
            n1 = atoi(argv[3]); n2 = atoi(argv[4]);
            rpm1 = atoi(argv[5]); rpm2 = atoi(argv[6]); is_dual = 1;
        } else {
            fprintf(stderr, "ERROR: too many args\n"); return -1;
        }
        return tool_sdo_csv(n1, n2, rpm1, rpm2, is_dual);
    }

    fprintf(stderr, "Unsupported mode: %s (use: cur / pos / csp / vel / csv)\n", mode);
    return -1;
}
