/**
 * @file  app_menu_def.c
 * @brief 菜单树实例定义（根菜单 + 功能测试子菜单）。
 *
 * 加菜单项 = 改这两张表。任务描述符来自 app_checks.h（任务类型契约见 app_task.h）。
 */
#include "app_menu.h"
#include "app_checks.h"
#include "app_ball_scurve_task.h"
#include "app_line_task.h"

/* --- Device Check 子菜单：在用外设自检 --- */
static const MENU_ITEM device_check_items[] = {
    { .name = "H3 Hold 0cm",   .kind = MENU_ENTRY_TASK, .u.task = &APP_H3_BALL_HOLD },
    { .name = "H3 Hold -5cm",  .kind = MENU_ENTRY_TASK, .u.task = &APP_H3_BALL_HOLD_M5 },
    { .name = "Gyro JY61P",   .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GYRO_JY61P },
    { .name = "Acc Calib",    .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_ACCEL_CALIB },
    { .name = "Gray I2C",     .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_GRAY_I2C },
    { .name = "TB6612",       .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_TB6612 },
    { .name = "Step Motor",   .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_STEP_MOTOR },
    { .name = "Pipe Swing",   .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_PIPE_SWING },
    { .name = "Encoder",      .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_ENCODER },
    { .name = "Speed PID",    .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_SPEED_PID },
    { .name = "Duty Sweep",   .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_DUTY_SWEEP },
    { .name = "Rpi UART",     .kind = MENU_ENTRY_TASK, .u.task = &APP_CHK_RPI_UART },
};

static const MENU_NODE device_check_menu = {
    "Device Check",
    device_check_items,
    (uint8_t)(sizeof(device_check_items) / sizeof(device_check_items[0])),
};

/* --- 根菜单 --- */
static const MENU_ITEM root_items[] = {
    { .name = "H2 Lap",        .kind = MENU_ENTRY_TASK,    .u.task = &APP_H2_LAP },
    { .name = "H3 Challenge",  .kind = MENU_ENTRY_TASK,    .u.task = &APP_H3_CHALLENGE },
    { .name = "H4 Loaded A-B", .kind = MENU_ENTRY_TASK,    .u.task = &APP_H4_LOADED_STRAIGHT },
    { .name = "H5 Loaded Lap O", .kind = MENU_ENTRY_TASK,  .u.task = &APP_H5_LOADED_LAP_CENTER },
    { .name = "H6 Loaded Any", .kind = MENU_ENTRY_TASK,    .u.task = &APP_H6_LOADED_LAP_TARGET },
    { .name = "Device Check",  .kind = MENU_ENTRY_SUBMENU, .u.submenu = &device_check_menu },
};

const MENU_NODE APP_ROOT_MENU = {
    "Main Menu",
    root_items,
    (uint8_t)(sizeof(root_items) / sizeof(root_items[0])),
};
