/**
 * @file cmd_read.c
 * @brief 读取命令: read <item> <id>
 *
 * items: angle / speed / current / temp / state / error / version / mode / pid / all
 *        limit_pos / limit_neg
 */

#include "command_registry.h"
#include "tool_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int cmd_do_read(motor_hal_t *hal, int cmd_id, int argc, char **argv)
{
    (void)hal; (void)cmd_id;

    if (!g_hal) { fprintf(stderr, "ERROR: daemon not initialized\n"); return -1; }

    const char *item = argv[2];
    int id = atoi(argv[3]);

    if (strcmp(item, "angle") == 0)    return tool_read_angle(id);
    if (strcmp(item, "speed") == 0)    return tool_read_speed(id);
    if (strcmp(item, "current") == 0)  return tool_read_current(id);
    if (strcmp(item, "temp") == 0)     return tool_read_temp(id);
    if (strcmp(item, "state") == 0)    return tool_read_state(id);
    if (strcmp(item, "error") == 0)    return tool_read_error(id);
    if (strcmp(item, "version") == 0)  return tool_read_version(id);
    if (strcmp(item, "mode") == 0)     return tool_read_mode(id);
    if (strcmp(item, "pid") == 0)      return tool_read_pid(id);
    if (strcmp(item, "all") == 0)      return tool_read_all(id);

    fprintf(stderr, "Unknown read item: %s\n", item);
    fprintf(stderr, "  Items: angle speed current temp state error version mode pid all\n");
    return -1;
}
