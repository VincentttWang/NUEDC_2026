/**
 * @file  oled.c
 * @brief SSD1306 OLED 硬件 I2C 显示实现 (帧缓冲 + DMA 整帧刷新)。
 *
 * 绘制操作只写 1KB RAM 帧缓冲 (纯 CPU, 极快); OLED_Flush() 把帧缓冲整块拷进
 * DMA 发送缓冲, 用水平寻址起一次 1025 字节事务后立刻返回, 数据由 DMA 逐字节
 * 灌进 I2C 控制器 TX FIFO, 完成由 I2C 控制器 STOP 中断收尾。
 * 400kHz 下整帧约 23ms, 这段时间 CPU 完全空出来。
 *
 * 双缓冲的理由: Flush 返回后上层立刻可以继续画下一帧, 而 DMA 还在读发送缓冲;
 * 若共用一块缓冲, 这些绘制会改到正在传输的数据上, 屏幕会撕裂。
 */

#include "oled.h"
#include "oled_font.h"
#include "bsp_time.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define OLED_CMD  0U
#define OLED_DATA 1U
#define OLED_I2C_ADDR_7BIT 0x3CU
#define OLED_I2C_TIMEOUT   1000000U

/*
 * DMA 通道号来自 SysConfig: G3507.syscfg 里 OLED 的 DMA_CHANNEL_EVENT1 命名为
 * DMA_CH_OLED_TX, 触发源 OLED_INST_DMA_TRIGGER 也由生成代码给出 (DMA_I2C1_TX_TRIG)。
 * 通道的传输模式/位宽/地址递增由 SYSCFG_DL_DMA_init() 配好, 本文件只填地址和长度。
 */
#define OLED_DMA_CHANNEL DMA_CH_OLED_TX_CHAN_ID

/* 传输收尾/异常用的控制器中断: STOP 表示整帧真的发完了。使能由 syscfg 生成。 */
#define OLED_I2C_INT_MASK (DL_I2C_INTERRUPT_CONTROLLER_STOP |             \
                           DL_I2C_INTERRUPT_CONTROLLER_NACK |             \
                           DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)

/*
 * 等一帧传完的自旋上限。整帧 1025B @400kHz 约 23ms, 这里取远大于该值的圈数只做卡死兜底。
 * 不用 BSP_Time_GetMs(): SysTick 要到 Scheduler_EnableTick() 才放开, OLED_Init 阶段 ms 还不走。
 */
#define OLED_FLUSH_TIMEOUT 4000000U

/* 绘制缓冲 (page-major: page*128+col), CPU 随时可改。 */
#define OLED_FB_PIXELS (OLED_WIDTH * OLED_PAGE_COUNT)   /* 128 * 8 = 1024 */
static uint8_t oled_fb[OLED_FB_PIXELS];
#define OLED_FB (oled_fb)

/* DMA 发送缓冲: [0]=0x40 数据控制字, [1..]=像素。传输期间不得改动。 */
static uint8_t oled_txbuf[1U + OLED_FB_PIXELS];
#define OLED_FRAME_LEN ((uint16_t)(1U + OLED_FB_PIXELS))

static volatile bool oled_tx_busy;     /* 一帧 DMA 在途 */
static volatile bool oled_tx_failed;   /* 最近一帧以 NACK/仲裁丢失/超时结束 */
static bool oled_dma_ready;            /* DMA 通道与 I2C 事件已配好 */

static bool OLED_I2C_WaitIdle(void);
static bool OLED_I2C_Write(const uint8_t *data, uint16_t len);
static void OLED_WriteCmds(const uint8_t *cmds, uint8_t len);
static void OLED_WR_Byte(uint8_t dat, uint8_t mode);
static uint32_t oled_pow(uint8_t m, uint8_t n);
static void OLED_DMA_Init(void);
static bool OLED_StartFrameDMA(void);
static void OLED_AbortFrameDMA(void);
static bool OLED_FrameTransferDone(void);

static void OLED_DelayMs(uint32_t ms){
    BSP_DelayMs(ms);
}

/* ═══════════════════ 阻塞 I2C 底层 (命令 + 整帧数据) ═══════════════════════ */

static bool OLED_I2C_WaitIdle(void){
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U){
        if (timeout-- == 0U){
            return false;
        }
    }

    return true;
}

/* 阻塞一次 I2C 写事务 (填 FIFO -> 启动 -> 边发边补, 支持任意长度)。 */
static bool OLED_I2C_Write(const uint8_t *data, uint16_t len){
    uint16_t sent = 0U;
    uint32_t timeout;

    if (data == NULL || len == 0U){
        return true;
    }

    /* 总线是 OLED 独占的, 但命令/整帧共用它: 上一帧 DMA 没发完不能插事务。 */
    (void)OLED_WaitFlushDone();

    if (!OLED_I2C_WaitIdle()){
        return false;
    }

    DL_I2C_resetControllerTransfer(OLED_INST);

    while (sent < len && !DL_I2C_isControllerTXFIFOFull(OLED_INST)){
        DL_I2C_transmitControllerData(OLED_INST, data[sent++]);
    }

    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    DL_Common_delayCycles(3U);

    timeout = OLED_I2C_TIMEOUT;
    while (sent < len){
        uint32_t status = DL_I2C_getControllerStatus(OLED_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U){
            return false;
        }
        if (!DL_I2C_isControllerTXFIFOFull(OLED_INST)){
            DL_I2C_transmitControllerData(OLED_INST, data[sent++]);
            timeout = OLED_I2C_TIMEOUT;
        }
        else if (timeout-- == 0U){
            return false;
        }
    }

    timeout = OLED_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U){
        if (timeout-- == 0U){
            return false;
        }
    }

    return ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U);
}

/* 写一串命令字节: [0x00][cmd0..cmdN]。 */
static void OLED_WriteCmds(const uint8_t *cmds, uint8_t len){
    uint8_t packet[8];
    uint8_t i;

    if (cmds == NULL || len == 0U || len > (sizeof(packet) - 1U)){
        return;
    }
    packet[0] = 0x00U;
    for (i = 0U; i < len; i++){
        packet[i + 1U] = cmds[i];
    }
    (void)OLED_I2C_Write(packet, (uint16_t)(len + 1U));
}

/* 写单个命令字节 (仅命令; 数据一律写帧缓冲, 不走此路)。 */
static void OLED_WR_Byte(uint8_t dat, uint8_t mode){
    if (mode == OLED_CMD){
        uint8_t c = dat;
        OLED_WriteCmds(&c, 1U);
    }
}

/* ═══════════════════ DMA 整帧传输 ═══════════════════════════════════════════ */

/*
 * 接上 SysConfig 配好的通道: 目的地址固定指向 I2C 的 TX 数据寄存器, 再放开 NVIC。
 * SYSCFG_DL_DMA_init() 只配通道的传输模式/位宽/地址递增/触发源, 不设地址与长度;
 * enableInterrupt 与 NVIC_SetPriority 也由生成代码做, 这里只补 NVIC_EnableIRQ。
 */
static void OLED_DMA_Init(void){
    DL_DMA_setDestAddr(DMA, OLED_DMA_CHANNEL, (uint32_t)&OLED_INST->MASTER.MTXDATA);

    DL_I2C_clearInterruptStatus(OLED_INST, OLED_I2C_INT_MASK);
    NVIC_EnableIRQ(OLED_INST_INT_IRQN);

    oled_tx_busy   = false;
    oled_tx_failed = false;
    oled_dma_ready = true;
}

/* DMA 已把整帧搬进 FIFO(SIZE 归零) 且控制器把 FIFO 发完回到空闲。 */
static bool OLED_FrameTransferDone(void){
    return (DL_DMA_getTransferSize(DMA, OLED_DMA_CHANNEL) == 0U) &&
           ((DL_I2C_getControllerStatus(OLED_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U);
}

/* 异常收尾: 停通道、清事务和 FIFO, 把状态放回空闲, 避免一次故障把 OLED 永久锁死。 */
static void OLED_AbortFrameDMA(void){
    DL_DMA_disableChannel(DMA, OLED_DMA_CHANNEL);
    DL_I2C_resetControllerTransfer(OLED_INST);
    DL_I2C_flushControllerTXFIFO(OLED_INST);
    oled_tx_failed = true;
    oled_tx_busy   = false;
}

/* 起一帧: 先让 DMA 把 FIFO 填上, 再拉起始位, 这样 SCL 一走就有数据可发。 */
static bool OLED_StartFrameDMA(void){
    if (!oled_dma_ready){
        return false;
    }
    if (!OLED_I2C_WaitIdle()){
        return false;
    }

    DL_DMA_disableChannel(DMA, OLED_DMA_CHANNEL);
    DL_I2C_resetControllerTransfer(OLED_INST);
    DL_I2C_flushControllerTXFIFO(OLED_INST);
    DL_I2C_clearInterruptStatus(OLED_INST, OLED_I2C_INT_MASK);

    DL_DMA_setSrcAddr(DMA, OLED_DMA_CHANNEL, (uint32_t)&oled_txbuf[0]);
    DL_DMA_setDestAddr(DMA, OLED_DMA_CHANNEL, (uint32_t)&OLED_INST->MASTER.MTXDATA);
    DL_DMA_setTransferSize(DMA, OLED_DMA_CHANNEL, OLED_FRAME_LEN);

    oled_tx_failed = false;
    oled_tx_busy   = true;

    DL_DMA_enableChannel(DMA, OLED_DMA_CHANNEL);
    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDR_7BIT,
        DL_I2C_CONTROLLER_DIRECTION_TX, OLED_FRAME_LEN);
    DL_Common_delayCycles(3U);   /* 等 MSR.BUSY 立起来, 免得轮询兜底误判"已完成" */

    return true;
}

/*
 * I2C1 中断: 整帧传输的正常收尾(STOP) 与异常收尾(NACK/仲裁丢失)。
 * 命令包走阻塞路径, 也会打到这里, 用 oled_tx_busy 门掉。
 */
void OLED_INST_IRQHandler(void){
    switch (DL_I2C_getPendingInterrupt(OLED_INST)){
    case DL_I2C_IIDX_CONTROLLER_STOP:
        if (oled_tx_busy){
            DL_DMA_disableChannel(DMA, OLED_DMA_CHANNEL);
            oled_tx_busy = false;
        }
        break;

    case DL_I2C_IIDX_CONTROLLER_NACK:
    case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        if (oled_tx_busy){
            DL_DMA_disableChannel(DMA, OLED_DMA_CHANNEL);
            DL_I2C_resetControllerTransfer(OLED_INST);
            oled_tx_failed = true;
            oled_tx_busy   = false;
        }
        break;

    default:
        break;
    }
}

/* ═══════════════════ 整帧刷新 ═══════════════════════════════════════════════ */

bool OLED_IsFlushBusy(void){
    return oled_tx_busy;
}

bool OLED_WaitFlushDone(void){
    uint32_t timeout = OLED_FLUSH_TIMEOUT;

    while (oled_tx_busy){
        /*
         * ISR 是正常收尾路径; 这里再轮询一层硬件, 使 OLED_Init 阶段(main 还没
         * __enable_irq) 同样能判出完成, 不至于每帧都空转到超时。
         */
        if (OLED_FrameTransferDone()){
            DL_DMA_disableChannel(DMA, OLED_DMA_CHANNEL);
            oled_tx_busy = false;
            break;
        }
        /*
         * 这里不查 MSR.ERR: 该位反映"上一次操作"的结果, 事务刚起、BUSY 还没立起来的
         * 那几个周期里读到的仍是上一帧的值, 拿它判错会把好帧误杀。NACK/仲裁丢失由
         * ISR 认, 中断没开时由下面的超时兜底。
         */
        if (timeout-- == 0U){
            OLED_AbortFrameDMA();
            return false;
        }
    }

    return !oled_tx_failed;
}

/*
 * 把整帧缓冲刷到屏幕: 水平寻址窗口设为全屏(写满 128 列自动换页, 1024 字节铺满全屏),
 * 再把 [0x40][1024 像素] 交给 DMA 发送, 函数不等传输结束就返回。
 * 上一帧未发完时会先等它 —— 发送缓冲要整块换掉, 不能边发边改。
 */
void OLED_Flush(void){
    static const uint8_t win_col[3] = {0x21U, 0x00U, 0x7FU};   /* 列范围 0..127 */
    static const uint8_t win_page[3] = {0x22U, 0x00U, 0x07U};  /* 页范围 0..7   */

    (void)OLED_WaitFlushDone();

    OLED_WriteCmds(win_col, 3U);    /* 命令包很短, 仍走阻塞路径 */
    OLED_WriteCmds(win_page, 3U);

    oled_txbuf[0] = 0x40U;          /* 数据控制字 */
    memcpy(&oled_txbuf[1], oled_fb, OLED_FB_PIXELS);

    if (!OLED_StartFrameDMA()){
        /* DMA 起不来(未初始化/总线未空闲)时退回阻塞整帧发送, 不因此丢帧。 */
        (void)OLED_I2C_Write(oled_txbuf, OLED_FRAME_LEN);
    }
}

/* ═══════════════════ 绘制 (只写帧缓冲) ═══════════════════════════════════════ */

void OLED_ColorTurn(uint8_t enable){
    OLED_WR_Byte(enable ? 0xA7U : 0xA6U, OLED_CMD);   /* 反色/正常 (显示命令, 立即生效) */
}

void OLED_DisplayOn(void){
    OLED_WR_Byte(0x8DU, OLED_CMD);
    OLED_WR_Byte(0x14U, OLED_CMD);
    OLED_WR_Byte(0xAFU, OLED_CMD);
}

void OLED_Display_On(void){
    OLED_DisplayOn();
}

void OLED_DisplayOff(void){
    OLED_WR_Byte(0x8DU, OLED_CMD);
    OLED_WR_Byte(0x10U, OLED_CMD);
    OLED_WR_Byte(0xAEU, OLED_CMD);
}

void OLED_Display_Off(void){
    OLED_DisplayOff();
}

/* 清整屏 (只清帧缓冲, 由 Flush 刷屏)。 */
void OLED_Clear(void){
    memset(OLED_FB, 0, OLED_FB_PIXELS);
}

/* 清指定页。 */
void OLED_ClearLine(uint8_t page){
    if (page >= OLED_PAGE_COUNT){
        return;
    }
    memset(OLED_FB + (uint16_t)page * OLED_WIDTH, 0, OLED_WIDTH);
}

/*
 * 在帧缓冲指定位置写一个字符 (6x8 或 8x16)。
 * 8px: 6 字节写在 page 行 cols x..x+5。
 * 16px: 上半 8 字节写 page, 下半 8 字节写 page+1, 均 cols x..x+7。
 */
void OLED_ShowChar(uint8_t x, uint8_t page, uint8_t chr, uint8_t sizey){
    uint8_t c;
    uint8_t i;
    uint16_t base;

    if ((sizey != 8U && sizey != 16U) || page >= OLED_PAGE_COUNT || x >= OLED_WIDTH){
        return;
    }
    if (chr < ' ' || chr > '~'){
        chr = '?';
    }
    c = (uint8_t)(chr - ' ');

    if (sizey == 8U){
        if ((uint16_t)x + 6U > OLED_WIDTH){
            return;
        }
        base = (uint16_t)page * OLED_WIDTH + x;
        for (i = 0U; i < 6U; i++){
            OLED_FB[base + i] = asc2_0806[c][i];
        }
    } else{
        if (((uint16_t)x + 8U > OLED_WIDTH) || ((uint8_t)(page + 1U) >= OLED_PAGE_COUNT)){
            return;
        }
        base = (uint16_t)page * OLED_WIDTH + x;
        for (i = 0U; i < 8U; i++){
            OLED_FB[base + i] = asc2_1608[c][i];               /* 上半页 */
            OLED_FB[base + OLED_WIDTH + i] = asc2_1608[c][i + 8U]; /* 下半页 */
        }
    }
}

static uint32_t oled_pow(uint8_t m, uint8_t n){
    uint32_t result = 1U;
    while (n--){
        result *= m;
    }
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t page, uint32_t num, uint8_t len, uint8_t sizey){
    uint8_t t, temp, m = 0U;
    uint8_t enshow = 0U;

    if (sizey == 8U){
        m = 2U;
    }
    for (t = 0U; t < len; t++){
        temp = (uint8_t)((num / oled_pow(10U, (uint8_t)(len - t - 1U))) % 10U);
        if (enshow == 0U && t < (len - 1U)){
            if (temp == 0U){
                OLED_ShowChar((uint8_t)(x + (sizey / 2U + m) * t), page, ' ', sizey);
                continue;
            } else{
                enshow = 1U;
            }
        }
        OLED_ShowChar((uint8_t)(x + (sizey / 2U + m) * t), page, (uint8_t)(temp + '0'), sizey);
    }
}

void OLED_ShowString(uint8_t x, uint8_t page, const char *chr, uint8_t sizey){
    uint8_t j = 0U;
    uint8_t step = (sizey == 8U) ? 6U : (uint8_t)(sizey / 2U);

    if (chr == NULL || step == 0U){
        return;
    }
    while (chr[j] != '\0' && x < OLED_WIDTH){
        if ((uint16_t)x + step > OLED_WIDTH){
            break;
        }
        OLED_ShowChar(x, page, (uint8_t)chr[j++], sizey);
        x = (uint8_t)(x + step);
    }
}

void OLED_ShowStringClearLine(uint8_t x, uint8_t page, const char *chr, uint8_t sizey){
    OLED_ClearLine(page);
    OLED_ShowString(x, page, chr, sizey);
}

void OLED_DrawBMP(uint8_t x, uint8_t page, uint8_t sizex, uint8_t sizey, const uint8_t *bmp){
    uint8_t page_count;
    uint16_t bmp_index = 0U;
    uint8_t p;

    if (bmp == NULL || x >= OLED_WIDTH || page >= OLED_PAGE_COUNT || sizex == 0U || sizey == 0U){
        return;
    }
    page_count = (uint8_t)((sizey + 7U) / 8U);

    for (p = 0U; p < page_count && (uint8_t)(page + p) < OLED_PAGE_COUNT; p++){
        uint16_t base = (uint16_t)(page + p) * OLED_WIDTH + x;
        uint8_t col;
        for (col = 0U; col < sizex && (uint8_t)(x + col) < OLED_WIDTH; col++){
            OLED_FB[base + col] = bmp[bmp_index + col];
        }
        bmp_index = (uint16_t)(bmp_index + sizex);
    }
}

/* ═══════════════════ 初始化 ═══════════════════════════════════════════════ */

void OLED_Init(void){
    OLED_DelayMs(100);
    OLED_DMA_Init();                // 整帧刷新走 DMA, 须在第一次 Flush 之前配好
    OLED_WR_Byte(0xAE, OLED_CMD);   // turn off oled panel
    OLED_WR_Byte(0x00, OLED_CMD);   // set low column address
    OLED_WR_Byte(0x10, OLED_CMD);   // set high column address
    OLED_WR_Byte(0x40, OLED_CMD);   // set start line address
    OLED_WR_Byte(0x81, OLED_CMD);   // set contrast control register
    OLED_WR_Byte(0xCF, OLED_CMD);   // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1, OLED_CMD);   // Set SEG/Column Mapping
    OLED_WR_Byte(0xC8, OLED_CMD);   // Set COM/Row Scan Direction
    OLED_WR_Byte(0xA6, OLED_CMD);   // set normal display
    OLED_WR_Byte(0xA8, OLED_CMD);   // set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f, OLED_CMD);   // 1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD);   // set display offset
    OLED_WR_Byte(0x00, OLED_CMD);   // not offset
    OLED_WR_Byte(0xd5, OLED_CMD);   // set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80, OLED_CMD);   // set divide ratio
    OLED_WR_Byte(0xD9, OLED_CMD);   // set pre-charge period
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);   // set com pins hardware configuration
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);   // set vcomh
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);   // Set Memory Addressing Mode
    OLED_WR_Byte(0x00, OLED_CMD);   // 00=水平寻址 (供整帧一次刷新)
    OLED_WR_Byte(0x8D, OLED_CMD);   // set Charge Pump enable/disable
    OLED_WR_Byte(0x14, OLED_CMD);   // enable
    OLED_WR_Byte(0xA4, OLED_CMD);   // Disable Entire Display On
    OLED_WR_Byte(0xA6, OLED_CMD);   // Disable Inverse Display On
    OLED_Clear();                   // 清帧缓冲
    OLED_Flush();                   // 刷成黑屏
    OLED_WR_Byte(0xAF, OLED_CMD);   // display ON
}
