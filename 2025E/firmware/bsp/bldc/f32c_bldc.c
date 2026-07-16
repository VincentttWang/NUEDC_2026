/**
 * @file  f32c_bldc.c
 * @brief F32C 无刷电机驱动 — UART3 通信协议实现
 *
 * 合并自参考工程 DataScope_DP.C + uart_callback.c, 适配当前工程 UART3 (BLDC_INST)
 */

#include "f32c_bldc.h"
#include "ti_msp_dl_config.h"
#include "bsp_time.h"

#include <string.h>

/* ===== 全局电机数据 ===== */
volatile BLDC_MotorData_t BLDC_Motor1 = {0};
volatile BLDC_MotorData_t BLDC_Motor2 = {0};

/* ===== 接收解析器状态 ===== */
#define RX_BUF_SIZE  9U

static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t rx_index;
static uint8_t rx_state;

/* ===== 初始化标志 (保证 BLDC_Init 幂等) ===== */
static uint8_t s_bldc_init_done;

/* ===== 多圈位置软件限位 (按地址 1/2 索引) ===== */
typedef struct {
    int32_t min_x10;
    int32_t max_x10;
    uint8_t enabled;
} BLDC_AngleLimit_t;

static BLDC_AngleLimit_t s_angle_limit[2];

/* 将地址映射到限位表下标, 无效地址返回 0xFF */
static uint8_t BLDC_LimitIndex(uint8_t addr)
{
    if (addr == BLDC_ADDR_1) {
        return 0U;
    }
    if (addr == BLDC_ADDR_2) {
        return 1U;
    }
    return 0xFFU;
}

/* 按限位钳位目标角度 */
static int32_t BLDC_ClampAngle(uint8_t addr, int32_t angle_x10)
{
    uint8_t idx = BLDC_LimitIndex(addr);

    if ((idx != 0xFFU) && (s_angle_limit[idx].enabled != 0U)) {
        if (angle_x10 < s_angle_limit[idx].min_x10) {
            angle_x10 = s_angle_limit[idx].min_x10;
        } else if (angle_x10 > s_angle_limit[idx].max_x10) {
            angle_x10 = s_angle_limit[idx].max_x10;
        }
    }

    return angle_x10;
}

void BLDC_SetAngleLimit(uint8_t addr, int32_t min_x10, int32_t max_x10)
{
    uint8_t idx = BLDC_LimitIndex(addr);

    if (idx == 0xFFU) {
        return;
    }

    if (min_x10 == max_x10) {
        s_angle_limit[idx].enabled = 0U;
        return;
    }

    if (min_x10 > max_x10) {
        int32_t tmp = min_x10;
        min_x10 = max_x10;
        max_x10 = tmp;
    }

    s_angle_limit[idx].min_x10 = min_x10;
    s_angle_limit[idx].max_x10 = max_x10;
    s_angle_limit[idx].enabled = 1U;
}

/* ===== BCC 异或校验 ===== */
static uint8_t Calc_BCC(uint8_t *data, uint8_t len)
{
    uint8_t bcc = 0U;
    uint8_t i;

    for (i = 0U; i < len; i++) {
        bcc ^= data[i];
    }

    return bcc;
}

/* ===== UART 发送（阻塞式，单字节） ===== */
#ifndef BLDC_UART_TX_TIMEOUT
#define BLDC_UART_TX_TIMEOUT 100000U
#endif

static void BLDC_UartSendByte(uint8_t data)
{
    /* 等 TX FIFO 有空位; 加超时兜底, 避免 TX 异常时无限阻塞 (best-effort, 超时丢字节)。 */
    uint32_t timeout = BLDC_UART_TX_TIMEOUT;
    while (DL_UART_isTXFIFOFull(BLDC_INST)) {
        if (timeout-- == 0U) {
            return;
        }
    }
    DL_UART_Main_transmitData(BLDC_INST, data);
}

/* ===== UART 发送字节数组 ===== */
static void BLDC_UartSendArray(uint8_t *data, uint8_t len)
{
    uint8_t i;

    for (i = 0U; i < len; i++) {
        BLDC_UartSendByte(data[i]);
    }
}

/* ===== 初始化 UART3 接收中断 ===== */
void BLDC_Init(void)
{
    /* 幂等: 已初始化则直接返回 (启动流程与设备检查页可能都会调用) */
    if (s_bldc_init_done != 0U) {
        return;
    }
    s_bldc_init_done = 1U;

    /* 配置 RX 引脚上拉 */
    DL_GPIO_setDigitalInternalResistor(
        GPIO_BLDC_IOMUX_RX, DL_GPIO_RESISTOR_PULL_UP);

    /* RX FIFO 阈值: 收到 1 字节即触发中断 */
    DL_UART_Main_setRXFIFOThreshold(
        BLDC_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);

    /* 使能 RX 中断 */
    DL_UART_Main_enableInterrupt(
        BLDC_INST, DL_UART_MAIN_INTERRUPT_RX);

    /* NVIC 优先级 1（与 BlueTooth UART 同级） */
    NVIC_SetPriority(BLDC_INST_INT_IRQN, 1U);
    NVIC_ClearPendingIRQ(BLDC_INST_INT_IRQN);
    NVIC_EnableIRQ(BLDC_INST_INT_IRQN);

    /* 发送哑字节唤醒总线, 等待稳定 */
    BLDC_UartSendByte(0x00U);
    BSP_DelayMs(1500U);
}

/* ===== 发送命令帧 ===== */
void BLDC_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t tx_buf[20];
    uint8_t idx = 0U;

    if (len > (sizeof(tx_buf) - 5U)) {
        return;
    }

    tx_buf[idx++] = BLDC_HEADER;
    tx_buf[idx++] = addr;
    tx_buf[idx++] = cmd;

    if ((len > 0U) && (data != NULL)) {
        (void)memcpy(&tx_buf[idx], data, len);
        idx = (uint8_t)(idx + len);
    }

    tx_buf[idx] = Calc_BCC(tx_buf, idx);
    idx++;
    tx_buf[idx++] = BLDC_TAIL;

    BLDC_UartSendArray(tx_buf, idx);

    /* 参考工程每条命令间有 1~10ms 延时, 防止 F32C 模块来不及处理 */
    BSP_DelayMs(1U);
}

/* ===== 各命令的便捷函数 ===== */

void BLDC_Enable(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_ENABLE, NULL, 0U);
}

void BLDC_Disable(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_DISABLE, NULL, 0U);
}

void BLDC_SetSpeed(uint8_t addr, int16_t rpm)
{
    uint8_t data[2];

    data[0] = (uint8_t)(((uint16_t)rpm >> 8) & 0xFFU);
    data[1] = (uint8_t)((uint16_t)rpm & 0xFFU);
    BLDC_SendCmd(addr, CMD_SPEED, data, 2U);
}

void BLDC_SetMode(uint8_t addr, uint16_t mode)
{
    uint8_t data[2];

    data[0] = (uint8_t)((mode >> 8) & 0xFFU);
    data[1] = (uint8_t)(mode & 0xFFU);
    BLDC_SendCmd(addr, CMD_MODE, data, 2U);
}

void BLDC_SetMultiAngle(uint8_t addr, int32_t angle_x10)
{
    uint32_t raw;
    uint8_t data[4];

    /* 应用软件限位, 防止超出机械允许范围 */
    angle_x10 = BLDC_ClampAngle(addr, angle_x10);
    raw = (uint32_t)angle_x10;

    data[0] = (uint8_t)((raw >> 24) & 0xFFU);
    data[1] = (uint8_t)((raw >> 16) & 0xFFU);
    data[2] = (uint8_t)((raw >> 8) & 0xFFU);
    data[3] = (uint8_t)(raw & 0xFFU);
    BLDC_SendCmd(addr, CMD_MULTI_POS, data, 4U);
}

void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10)
{
    uint8_t data[2];

    if (angle_x10 > 3599U) {
        angle_x10 = 3599U;
    }

    data[0] = (uint8_t)((angle_x10 >> 8) & 0xFFU);
    data[1] = (uint8_t)(angle_x10 & 0xFFU);
    BLDC_SendCmd(addr, CMD_SINGLE_POS, data, 2U);
}

void BLDC_ReqFeedback(uint8_t addr, uint8_t type)
{
    uint8_t data[1] = {type};

    BLDC_SendCmd(addr, CMD_FEEDBACK, data, 1U);
}

void BLDC_SetAcc(uint8_t addr, uint16_t acc)
{
    uint8_t data[2];

    data[0] = (uint8_t)((acc >> 8) & 0xFFU);
    data[1] = (uint8_t)(acc & 0xFFU);
    BLDC_SendCmd(addr, CMD_ACC, data, 2U);
}

void BLDC_SaveParams(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_SAVE, NULL, 0U);
}

void BLDC_ClearMultiAngle(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_CLEAR_MULTI, NULL, 0U);
}

void BLDC_SetSingleAngleZero(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_SET_ZERO, NULL, 0U);
}

void BLDC_FactoryReset(uint8_t addr)
{
    BLDC_SendCmd(addr, CMD_FACTORY_RST, NULL, 0U);
}

void BLDC_SetAddress(uint8_t addr, uint8_t new_addr)
{
    uint8_t data[1] = {new_addr};

    BLDC_SendCmd(addr, CMD_SET_ADDR, data, 1U);
}

/* ===== 启动俯仰电机 ===== */
void BLDC_StartupPitch(void)
{
    /* 使能俯仰电机, 进入多圈位置闭环 */
    BLDC_Enable(BLDC_PITCH_ADDR);
    BLDC_SetMode(BLDC_PITCH_ADDR, MODE_MULTI_POS);

    /* 位置模式下必须先设定运动速度, 否则速度默认为 0, 收到目标角也不会转.
     * 速度取偏低值, 让电机温和逼近目标, 抑制到位过冲与震荡 */
    BLDC_SetSpeed(BLDC_PITCH_ADDR, BLDC_PITCH_SPEED);

    /* 限制加速度, 平滑加减速过程, 进一步减小过冲震荡 */
    BLDC_SetAcc(BLDC_PITCH_ADDR, BLDC_PITCH_ACC);

    /* 以当前物理位置为 0° 起始点 */
    BLDC_ClearMultiAngle(BLDC_PITCH_ADDR);

    /* 软件限位: 只允许 [0°, 180°] 正向范围 */
    BLDC_SetAngleLimit(BLDC_PITCH_ADDR, BLDC_PITCH_MIN_X10, BLDC_PITCH_MAX_X10);

    /* 正向转到约 120° */
    BLDC_SetMultiAngle(BLDC_PITCH_ADDR, BLDC_PITCH_INIT_X10);
}

/* ===== 接收解析器复位 ===== */
static void BLDC_ResetRxParser(void)
{
    rx_index = 0U;
    rx_state = 0U;
}

/* ===== 逐字节接收解析（在 UART ISR 中调用） ===== */
void BLDC_ParseRxByte(uint8_t rx_byte)
{
    switch (rx_state) {
    case 0:  /* 等待帧头 0x7A */
        if (rx_byte == BLDC_HEADER) {
            rx_buf[0]  = rx_byte;
            rx_index   = 1U;
            rx_state   = 1U;
        }
        break;

    case 1:  /* 地址: 仅接受 0x01 或 0x02 */
        if ((rx_byte == BLDC_ADDR_1) || (rx_byte == BLDC_ADDR_2)) {
            rx_buf[rx_index++] = rx_byte;
            rx_state = 2U;
        } else {
            BLDC_ResetRxParser();
        }
        break;

    case 2:  /* 反馈类型 */
        if (rx_byte <= FB_VOLTAGE) {
            rx_buf[rx_index++] = rx_byte;
            rx_state = 3U;
        } else {
            BLDC_ResetRxParser();
        }
        break;

    case 3:  /* 数据字节 3~6 (共 4 字节, 大端序) */
        rx_buf[rx_index++] = rx_byte;
        if (rx_index >= 7U) {
            rx_state = 4U;
        }
        break;

    case 4:  /* BCC 校验字节 */
        rx_buf[rx_index++] = rx_byte;
        rx_state = 5U;
        break;

    case 5:  /* 帧尾 */
        if (rx_byte == BLDC_TAIL) {
            rx_buf[rx_index++] = rx_byte;

            /* BCC 校验: 帧头~数据共 7 字节 XOR = BCC 字节 */
            if (Calc_BCC(rx_buf, 7U) == rx_buf[7U]) {
                uint8_t  addr = rx_buf[1U];
                uint8_t  type = rx_buf[2U];
                uint32_t raw  = ((uint32_t)rx_buf[3U] << 24U) |
                                ((uint32_t)rx_buf[4U] << 16U) |
                                ((uint32_t)rx_buf[5U] << 8U) |
                                ((uint32_t)rx_buf[6U]);
                volatile BLDC_MotorData_t *motor =
                    (addr == BLDC_ADDR_1) ? &BLDC_Motor1 : &BLDC_Motor2;

                switch (type) {
                case FB_SPEED:
                    motor->speed = (int16_t)((uint16_t)raw);
                    break;
                case FB_MULTI_ANGLE:
                    motor->multi_angle = (int32_t)raw;
                    motor->multi_angle_frame_count++;
                    break;
                case FB_SINGLE_ANGLE:
                    motor->single_angle = (uint16_t)raw;
                    break;
                case FB_ACC:
                    motor->acc = (int16_t)((uint16_t)raw);
                    break;
                case FB_VOLTAGE:
                    motor->voltage = (uint16_t)raw;
                    break;
                default:
                    break;
                }

                motor->data_ready = 1U;
            }
        }
        BLDC_ResetRxParser();
        break;

    default:
        BLDC_ResetRxParser();
        break;
    }
}

/* ===== UART3 接收中断处理 ===== */
void BLDC_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(BLDC_INST)
        == DL_UART_MAIN_IIDX_RX) {
        uint8_t rx_byte = DL_UART_Main_receiveData(BLDC_INST);
        BLDC_ParseRxByte(rx_byte);
    }
}
