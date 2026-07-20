#include "app_launcher.h"

#include "app_device_check.h"
#include "app_e_calibration_test.h"
#include "app_e_task.h"
#include "bsp_time.h"
#include "key.h"
#include "ui.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_MENU_DELAY_MS 20U

typedef enum {
    APP_MENU_E1_LINE = 0,
    APP_MENU_AIM_TRACK,
    APP_MENU_E2_AIM_2S,
    APP_MENU_E3_AUTO_AIM,
    APP_MENU_F1_LINE_AIM,
    APP_MENU_F1_LINE_AIM_SLOW,
    APP_MENU_F2_LINE_AIM,
    APP_MENU_F3_LINE_CIRCLE,
    APP_MENU_CALIBRATION,
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
    APP_E3_SCAN_POSITIVE = 0,
    APP_E3_SCAN_NEGATIVE,
    APP_E3_SCAN_COUNT
} APP_E3_SCAN_ITEM;

static const char *const s_app_menu_items[APP_MENU_COUNT] = {
    "E1 Line",
    "Aim track",
    "E2 Aim",
    "E3 Scan aim",
    "F1 Line+aim",
    "F1 slow",
    "F2 Line+aim",
    "F3 Line+circle",
    "Calibration",
    "Device check",
};

static const char *const s_app_e1_laps_items[APP_E1_LAPS_COUNT] = {
    "1 lap",
    "2 laps",
    "3 laps",
    "4 laps",
    "5 laps",
};

static const char *const s_app_e3_scan_items[APP_E3_SCAN_COUNT] = {
    "Scan +",
    "Scan -",
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

static void App_RenderE3Menu(APP_E3_SCAN_ITEM selected){
    UI_LIST_PAGE page = {
        .title = "E3 Scan Dir",
        .items = s_app_e3_scan_items,
        .item_count = APP_E3_SCAN_COUNT,
        .selected_index = (uint8_t)selected,
        .first_visible_index = App_GetFirstVisibleIndex((uint8_t)selected),
    };

    Ui_RenderListPage(&page);
}

static bool App_IsMenuDownEvent(void){
    return Key_IsShortPress(KEY_ID_DOWN);
}

static bool App_IsMenuUpEvent(void){
    return Key_IsShortPress(KEY_ID_UP);
}

static bool App_IsMenuEnterEvent(void){
    return Key_IsShortPress(KEY_ID_ENTER);
}

static void App_RunE1Menu(void){
    APP_E1_LAPS_ITEM selected = APP_E1_LAPS_1;

    Key_ClearAllEvents();
    App_RenderE1Menu(selected);

    while (1){
        if (App_IsMenuDownEvent()){
            selected = (APP_E1_LAPS_ITEM)(((uint8_t)selected + 1U) % (uint8_t)APP_E1_LAPS_COUNT);
            Key_ClearAllEvents();
            App_RenderE1Menu(selected);
        }

        if (App_IsMenuUpEvent()){
            selected = (APP_E1_LAPS_ITEM)(((uint8_t)selected + (uint8_t)APP_E1_LAPS_COUNT - 1U) %
                (uint8_t)APP_E1_LAPS_COUNT);
            Key_ClearAllEvents();
            App_RenderE1Menu(selected);
        }

        if (App_IsMenuEnterEvent()){
            Key_ClearAllEvents();
            AppE_RunLineFollow((uint8_t)selected + 1U);
            return;
        }

        BSP_DelayMs(APP_MENU_DELAY_MS);
    }
}

static void App_RunE3Menu(void){
    APP_E3_SCAN_ITEM selected = APP_E3_SCAN_POSITIVE;

    Key_ClearAllEvents();
    App_RenderE3Menu(selected);

    while (1){
        if (App_IsMenuDownEvent()){
            selected = (APP_E3_SCAN_ITEM)(((uint8_t)selected + 1U) % (uint8_t)APP_E3_SCAN_COUNT);
            Key_ClearAllEvents();
            App_RenderE3Menu(selected);
        }

        if (App_IsMenuUpEvent()){
            selected = (APP_E3_SCAN_ITEM)(((uint8_t)selected + (uint8_t)APP_E3_SCAN_COUNT - 1U) %
                (uint8_t)APP_E3_SCAN_COUNT);
            Key_ClearAllEvents();
            App_RenderE3Menu(selected);
        }

        if (App_IsMenuEnterEvent()){
            Key_ClearAllEvents();
            AppE_RunScanAim(selected == APP_E3_SCAN_NEGATIVE
                                ? APP_E_SCAN_DIR_NEGATIVE
                                : APP_E_SCAN_DIR_POSITIVE);
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

    if (selected == APP_MENU_AIM_TRACK){
        AppE_RunContinuousAim();
        return;
    }

    if (selected == APP_MENU_E2_AIM_2S){
        AppE_RunAimCenter2s();
        return;
    }

    if (selected == APP_MENU_E3_AUTO_AIM){
        App_RunE3Menu();
        return;
    }

    if (selected == APP_MENU_F1_LINE_AIM){
        AppE_RunLineAim(1U);
        return;
    }

    if (selected == APP_MENU_F1_LINE_AIM_SLOW){
        AppE_RunLineAimSlow(1U);
        return;
    }

    if (selected == APP_MENU_F2_LINE_AIM){
        AppE_RunLineAim(2U);
        return;
    }

    if (selected == APP_MENU_F3_LINE_CIRCLE){
        AppE_RunLineCircle(1U);
        return;
    }

    if (selected == APP_MENU_CALIBRATION){
        AppE_CalibrationTest_RunMenu();
        return;
    }

    if (selected == APP_MENU_DEVICE_CHECK){
        AppDeviceCheck_Run();
    }
}

void App_Launch(void){
    APP_MENU_ITEM selected = APP_MENU_E1_LINE;

    Key_ClearAllEvents();
    Ui_RenderStatusPage("NUEDC 2025 E", UI_STATUS_NORMAL, "K1/K4:move", "K3:enter");
    BSP_DelayMs(800U);
    App_RenderMenu(selected);

    while (1){
        if (App_IsMenuDownEvent()){
            selected = (APP_MENU_ITEM)(((uint8_t)selected + 1U) % (uint8_t)APP_MENU_COUNT);
            Key_ClearAllEvents();
            App_RenderMenu(selected);
        }

        if (App_IsMenuUpEvent()){
            selected = (APP_MENU_ITEM)(((uint8_t)selected + (uint8_t)APP_MENU_COUNT - 1U) %
                (uint8_t)APP_MENU_COUNT);
            Key_ClearAllEvents();
            App_RenderMenu(selected);
        }

        if (App_IsMenuEnterEvent()){
            Key_ClearAllEvents();
            App_RunMenuItem(selected);
            App_RenderMenu(selected);
        }

        BSP_DelayMs(APP_MENU_DELAY_MS);
    }
}
