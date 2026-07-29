#include "app_launcher.h"

#include "app_device_check.h"
#include "app_e_task.h"
#include "bsp_time.h"
#include "key.h"
#include "ui.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_MENU_DELAY_MS 20U

typedef enum {
    APP_MENU_E1_LINE = 0,
    APP_MENU_YAW_SPEED_TEST,
    APP_MENU_E2_AIM_2S,
    APP_MENU_E3_YAW_POS,
    APP_MENU_E3_YAW_NEG,
    APP_MENU_DEVICE_CHECK,
    APP_MENU_COUNT
} APP_MENU_ITEM;

typedef enum {
    APP_E1_LAPS_1 = 0,
    APP_E1_LAPS_2,
    APP_E1_LAPS_3,
    APP_E1_LAPS_4,
    APP_E1_LAPS_5,
    APP_E1_LAPS_COUNT
} APP_E1_LAPS_ITEM;

typedef enum {
    APP_YAW_SPEED_1 = 0,
    APP_YAW_SPEED_2,
    APP_YAW_SPEED_5,
    APP_YAW_SPEED_10,
    APP_YAW_SPEED_15,
    APP_YAW_SPEED_20,
    APP_YAW_SPEED_COUNT
} APP_YAW_SPEED_ITEM;

static const char *const s_app_menu_items[APP_MENU_COUNT] = {
    "E1 Line",
    "Yaw spd test",
    "E2 Aim",
    "E3 Yaw+",
    "E3 Yaw-",
    "Device check",
};

static const char *const s_app_e1_laps_items[APP_E1_LAPS_COUNT] = {
    "1 lap",
    "2 laps",
    "3 laps",
    "4 laps",
    "5 laps",
};

static const char *const s_app_yaw_speed_items[APP_YAW_SPEED_COUNT] = {
    "1 deg/s",
    "5 deg/s",
    "20 deg/s",
    "45 deg/s",
    "90 deg/s",
    "120 deg/s",
};

static const float s_app_yaw_speed_values[APP_YAW_SPEED_COUNT] = {
    1.0f,
    5.0f,
    20.0f,
    45.0f,
    90.0f,
    120.0f,
};

static uint8_t App_GetFirstVisibleIndex(uint8_t selected_index){
    if (selected_index < UI_LIST_VISIBLE_COUNT){
        return 0U;
    }

    return (uint8_t)(selected_index - UI_LIST_VISIBLE_COUNT + 1U);
}

static void App_RenderMenu(APP_MENU_ITEM selected){
    UI_LIST_PAGE page = {
        .title = "NUEDC 2025 E",
        .items = s_app_menu_items,
        .item_count = APP_MENU_COUNT,
        .selected_index = (uint8_t)selected,
        .first_visible_index = App_GetFirstVisibleIndex((uint8_t)selected),
    };

    Ui_RenderListPage(&page);
}

static void App_RenderE1Menu(APP_E1_LAPS_ITEM selected){
    UI_LIST_PAGE page = {
        .title = "E1 Line Laps",
        .items = s_app_e1_laps_items,
        .item_count = APP_E1_LAPS_COUNT,
        .selected_index = (uint8_t)selected,
        .first_visible_index = App_GetFirstVisibleIndex((uint8_t)selected),
    };

    Ui_RenderListPage(&page);
}

static void App_RenderYawSpeedMenu(APP_YAW_SPEED_ITEM selected){
    UI_LIST_PAGE page = {
        .title = "Yaw Speed",
        .items = s_app_yaw_speed_items,
        .item_count = APP_YAW_SPEED_COUNT,
        .selected_index = (uint8_t)selected,
        .first_visible_index = App_GetFirstVisibleIndex((uint8_t)selected),
    };

    Ui_RenderListPage(&page);
}

static bool App_IsNextMenuEvent(void){
    return Key_IsShortPress(KEY_ID_1) || Key_IsDoubleClick(KEY_ID_1);
}

static void App_RunE1Menu(void){
    APP_E1_LAPS_ITEM selected = APP_E1_LAPS_1;

    Key_ClearAllEvents();
    App_RenderE1Menu(selected);

    while (1){
        if (App_IsNextMenuEvent()){
            selected = (APP_E1_LAPS_ITEM)(((uint8_t)selected + 1U) % (uint8_t)APP_E1_LAPS_COUNT);
            Key_ClearAllEvents();
            App_RenderE1Menu(selected);
        }

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            AppE_RunLineFollow((uint8_t)selected + 1U);
            return;
        }

        BSP_DelayMs(APP_MENU_DELAY_MS);
    }
}

static void App_RunYawSpeedMenu(void){
    APP_YAW_SPEED_ITEM selected = APP_YAW_SPEED_1;

    Key_ClearAllEvents();
    App_RenderYawSpeedMenu(selected);

    while (1){
        if (App_IsNextMenuEvent()){
            selected = (APP_YAW_SPEED_ITEM)(((uint8_t)selected + 1U) % (uint8_t)APP_YAW_SPEED_COUNT);
            Key_ClearAllEvents();
            App_RenderYawSpeedMenu(selected);
        }

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            AppE_RunYawSpeedTest(s_app_yaw_speed_values[selected]);
            return;
        }

        BSP_DelayMs(APP_MENU_DELAY_MS);
    }
}

static void App_RunMenuItem(APP_MENU_ITEM selected){
    if (selected == APP_MENU_E1_LINE){
        App_RunE1Menu();
        return;
    }

    if (selected == APP_MENU_YAW_SPEED_TEST){
        App_RunYawSpeedMenu();
        return;
    }

    if (selected == APP_MENU_E2_AIM_2S){
        AppE_RunAimCenter2s();
        return;
    }

    if (selected == APP_MENU_E3_YAW_POS){
        AppE_RunRectScanYawPositive();
        return;
    }

    if (selected == APP_MENU_E3_YAW_NEG){
        AppE_RunRectScanYawNegative();
        return;
    }

    if (selected == APP_MENU_DEVICE_CHECK){
        AppDeviceCheck_Run();
    }
}

void App_Launch(void){
    APP_MENU_ITEM selected = APP_MENU_E1_LINE;

    Key_ClearAllEvents();
    Ui_RenderStatusPage("NUEDC 2025 E", UI_STATUS_NORMAL, "Short:next", "Long:enter");
    BSP_DelayMs(800U);
    App_RenderMenu(selected);

    while (1){
        if (App_IsNextMenuEvent()){
            selected = (APP_MENU_ITEM)(((uint8_t)selected + 1U) % (uint8_t)APP_MENU_COUNT);
            Key_ClearAllEvents();
            App_RenderMenu(selected);
        }

        if (Key_IsLongPress(KEY_ID_1)){
            Key_ClearAllEvents();
            App_RunMenuItem(selected);
            App_RenderMenu(selected);
        }

        BSP_DelayMs(APP_MENU_DELAY_MS);
    }
}
