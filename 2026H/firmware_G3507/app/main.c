/**
 * @file  main.c
 * @brief 固件入口：初始化后进入协作式调度超循环。
 *
 * 应用层为“菜单选择 → 执行选中任务 → 退回菜单”的状态机(见 app_mode)，
 * 由时间触发调度器(见 app_scheduler)驱动；任务经菜单树(见 app_menu)选择，
 * 任务生命周期契约见 app_task。
 * SysTick_Handler 定义在 app_scheduler.c（时基递增 + 按键扫描）。
 */
#include "app_init.h"
#include "app_scheduler.h"
#include "ti_msp_dl_config.h"

/*
 * MSPM0 Flash 按 64 位(8 字节)字 + ECC 编程，镜像总长若非 8 的倍数，最后不足一整字
 * 的部分会编程失败(现象: Erase OK 但 Programming Failed)。此填充经 Keil 分散加载文件
 * (board/startup/mspm0g3507.sct)以 +Last 放在镜像末尾并 8 字节对齐，强制总长为 8 的倍数。
 * 仅 Keil(armclang) 需要: 自定义段配合 .sct 的 +Last; CCS/GCC 的链接脚本另行处理，
 * 故用 __ARMCC_VERSION 限定，避免在其它工具链留下未放置段。
 */
#if defined(__ARMCC_VERSION)
__attribute__((used, aligned(8), section(".flash_tail_pad")))
static const unsigned char g_flash_tail_pad[8] = {
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU,
};
#endif

int main(void){
    App_Init();
    __enable_irq();

    while (1){
        Scheduler_Run();
    }
}
