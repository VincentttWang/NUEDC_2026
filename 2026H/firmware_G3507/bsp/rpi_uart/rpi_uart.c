/**
 * @file  rpi_uart.c
 * @brief Rpi_UART/UART2 收发、协议解析、测量前推与调参帧队列。
 */
#include "rpi_uart.h"

#include "bsp_time.h"
#include "ti_msp_dl_config.h"

#include <math.h>
#include <stddef.h>

#define RPI_RX_BUFFER_SIZE 128U
#define RPI_RX_BUFFER_MASK (RPI_RX_BUFFER_SIZE - 1U)
#define RPI_TX_BUFFER_SIZE 256U
#define RPI_TX_BUFFER_MASK (RPI_TX_BUFFER_SIZE - 1U)
#define RPI_CONFIG_QUEUE_SIZE 8U
#define RPI_CONFIG_QUEUE_MASK (RPI_CONFIG_QUEUE_SIZE - 1U)
#define RPI_SYNC0          0xA5U
#define RPI_SYNC1          0x5AU
#define RPI_V_RECOVERY_FRAMES 3U
#define RPI_UART_IRQ_PRIORITY  2U /* 低于编码器保护中断，高于 OLED 等后台传输中断。 */
#define BALL_ACCEL_MM_S2   7004.75f /* (5/7)×9.80665 m/s²，换算为 mm/s²。 */

#define RPI_RX_ERROR_INTERRUPTS \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | DL_UART_MAIN_INTERRUPT_BREAK_ERROR | \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR | DL_UART_MAIN_INTERRUPT_FRAMING_ERROR)

#if ((RPI_RX_BUFFER_SIZE & (RPI_RX_BUFFER_SIZE - 1U)) != 0U)
#error "RPI_RX_BUFFER_SIZE 必须是 2 的幂"
#endif

typedef struct {
    uint8_t byte;
    uint32_t arrival_ms;
} RPI_RX_ITEM;

#if ((RPI_TX_BUFFER_SIZE & (RPI_TX_BUFFER_SIZE - 1U)) != 0U)
#error "RPI_TX_BUFFER_SIZE must be a power of two"
#endif
#if ((RPI_CONFIG_QUEUE_SIZE & (RPI_CONFIG_QUEUE_SIZE - 1U)) != 0U)
#error "RPI_CONFIG_QUEUE_SIZE must be a power of two"
#endif

static RPI_RX_ITEM rx_buffer[RPI_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t uart_rx_bytes;
static volatile uint32_t uart_rx_dropped;
static volatile uint32_t uart_errors;
static uint8_t tx_buffer[RPI_TX_BUFFER_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static RPI_UART_CONFIG_REQUEST_FRAME config_queue[RPI_CONFIG_QUEUE_SIZE];
static uint8_t config_head;
static uint8_t config_tail;

static uint8_t parse_buffer[RPI_UART_FRAME_SIZE];
static uint32_t parse_time[RPI_UART_FRAME_SIZE];
static uint8_t parse_len;
static RPI_UART_MEASUREMENT latest;
static bool have_latest;
static bool last_frame_valid;
static bool have_seq;
static uint8_t last_seq;
static uint8_t velocity_block_frames;
static RPI_UART_STATS diag;
static uint32_t rate_window_start_ms;
static uint32_t rate_window_frames;

static void RpiUart_FlushHardwareRx(void){
    while (!DL_UART_Main_isRXFIFOEmpty(Rpi_UART_INST)){
        (void)DL_UART_Main_receiveData(Rpi_UART_INST);
    }
}

uint8_t RpiUart_Crc8(const uint8_t *data, uint8_t len){
    uint8_t crc = 0xFFU;
    if (data == NULL){
        return crc;
    }
    while (len-- > 0U){
        crc ^= *data++;
        for (uint8_t i = 0U; i < 8U; i++){
            crc = ((crc & 0x80U) != 0U)
                ? (uint8_t)((uint8_t)(crc << 1U) ^ 0x07U)
                : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

void RpiUart_ResetStats(void){
    diag.rx_frames = 0U;
    diag.crc_fail = 0U;
    diag.ver_fail = 0U;
    diag.seq_gap = 0U;
    diag.invalid = 0U;
    diag.max_age_ms = 0U;
    diag.frame_rate_x10 = 0U;
    uart_rx_bytes = 0U;
    uart_rx_dropped = 0U;
    uart_errors = 0U;
    rate_window_start_ms = BSP_Time_GetMs();
    rate_window_frames = 0U;
}

void RpiUart_Init(void){
    rx_head = 0U;
    rx_tail = 0U;
    tx_head = 0U;
    tx_tail = 0U;
    config_head = 0U;
    config_tail = 0U;
    parse_len = 0U;
    have_latest = false;
    last_frame_valid = false;
    have_seq = false;
    velocity_block_frames = 0U;
    RpiUart_ResetStats();

    DL_UART_Main_setTXFIFOThreshold(Rpi_UART_INST, DL_UART_TX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_disableInterrupt(Rpi_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    DL_UART_Main_setRXFIFOThreshold(Rpi_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    RpiUart_FlushHardwareRx();
    DL_UART_Main_clearInterruptStatus(Rpi_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | RPI_RX_ERROR_INTERRUPTS);
    NVIC_SetPriority(Rpi_UART_INST_INT_IRQN, RPI_UART_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(Rpi_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(Rpi_UART_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(Rpi_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | RPI_RX_ERROR_INTERRUPTS);
}

static void ParserShiftOne(void){
    for (uint8_t i = 1U; i < parse_len; i++){
        parse_buffer[i - 1U] = parse_buffer[i];
        parse_time[i - 1U] = parse_time[i];
    }
    if (parse_len > 0U){
        parse_len--;
    }
}

static void ParserSeekSync(void){
    while (parse_len >= 2U){
        if ((parse_buffer[0] == RPI_SYNC0) && (parse_buffer[1] == RPI_SYNC1)){
            return;
        }
        ParserShiftOne();
    }
    if ((parse_len == 1U) && (parse_buffer[0] != RPI_SYNC0)){
        parse_len = 0U;
    }
}

static uint16_t EffectiveAgeAt(uint8_t source_age_ms, uint32_t rx_end_ms, uint32_t now_ms){
    uint32_t age = (uint32_t)source_age_ms + (now_ms - rx_end_ms) + 1U;
    return (age > 0xFFFFU) ? 0xFFFFU : (uint16_t)age;
}

static void AcceptFrame(uint32_t rx_end_ms){
    uint32_t now = BSP_Time_GetMs();
    uint8_t seq = parse_buffer[3];
    uint8_t flags = parse_buffer[9];
    bool gap = false;

    diag.rx_frames++;
    rate_window_frames++;

    if (have_seq){
        uint8_t delta = (uint8_t)(seq - last_seq);
        if (delta != 1U){
            diag.seq_gap += (delta == 0U) ? 1U : (uint32_t)(delta - 1U);
            velocity_block_frames = RPI_V_RECOVERY_FRAMES;
            gap = true;
        }
    }
    have_seq = true;
    last_seq = seq;

    if ((flags & RPI_UART_FLAG_VALID) == 0U){
        diag.invalid++;
        last_frame_valid = false;
        return; /* 明确保留上一次有效状态，绝不能把无效帧当作 X=0。 */
    }

    if (!gap && (velocity_block_frames > 0U)){
        velocity_block_frames--;
    }

    latest.seq = seq;
    latest.age_ms = parse_buffer[4];
    latest.x_01mm = (int16_t)((uint16_t)parse_buffer[5] |
                              ((uint16_t)parse_buffer[6] << 8U));
    latest.velocity_mm_s = (int16_t)((uint16_t)parse_buffer[7] |
                                     ((uint16_t)parse_buffer[8] << 8U));
    latest.flags = flags;
    latest.rx_end_ms = rx_end_ms;
    latest.velocity_trusted = ((flags & RPI_UART_FLAG_V_VALID) != 0U) &&
                              (velocity_block_frames == 0U);
    have_latest = true;
    last_frame_valid = true;

    uint16_t age = EffectiveAgeAt(latest.age_ms, latest.rx_end_ms, now);
    if (age > diag.max_age_ms){
        diag.max_age_ms = age;
    }
}

static void AcceptConfigRequest(void){
    uint8_t next = (uint8_t)((config_head + 1U) & RPI_CONFIG_QUEUE_MASK);
    if (next == config_tail){
        return;
    }
    config_queue[config_head].seq = parse_buffer[3];
    config_queue[config_head].op = parse_buffer[4];
    config_queue[config_head].param_id =
        (uint16_t)parse_buffer[5] | ((uint16_t)parse_buffer[6] << 8U);
    config_queue[config_head].value =
        (uint16_t)parse_buffer[7] | ((uint16_t)parse_buffer[8] << 8U);
    config_head = next;
}

static void ParserPush(uint8_t byte, uint32_t arrival_ms){
    if (parse_len < RPI_UART_FRAME_SIZE){
        parse_buffer[parse_len] = byte;
        parse_time[parse_len] = arrival_ms;
        parse_len++;
    }
    ParserSeekSync();

    if (parse_len < RPI_UART_FRAME_SIZE){
        return;
    }

    if (RpiUart_Crc8(&parse_buffer[2], 8U) != parse_buffer[10]){
        diag.crc_fail++;
        ParserShiftOne();
        ParserSeekSync();
        return;
    }

    if (parse_buffer[2] == RPI_UART_PROTOCOL_VERSION){
        AcceptFrame(parse_time[10]);
    } else if (parse_buffer[2] == RPI_UART_CONFIG_REQUEST){
        AcceptConfigRequest();
    } else{
        diag.ver_fail++;
    }
    parse_len = 0U;
}

void RpiUart_Poll(void){
    while (rx_tail != rx_head){
        RPI_RX_ITEM item = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & RPI_RX_BUFFER_MASK);
        ParserPush(item.byte, item.arrival_ms);
    }

    uint32_t now = BSP_Time_GetMs();
    uint32_t elapsed = now - rate_window_start_ms;
    if (elapsed >= 1000U){
        uint32_t rate_x10 = (rate_window_frames * 10000U + (elapsed / 2U)) / elapsed;
        diag.frame_rate_x10 = (rate_x10 > 0xFFFFU) ? 0xFFFFU : (uint16_t)rate_x10;
        rate_window_frames = 0U;
        rate_window_start_ms = now;
    }
}

bool RpiUart_GetLatest(RPI_UART_MEASUREMENT *measurement){
    if ((measurement == NULL) || !have_latest){
        return false;
    }
    *measurement = latest;
    return true;
}

bool RpiUart_GetConfigRequest(RPI_UART_CONFIG_REQUEST_FRAME *request){
    if ((request == NULL) || (config_tail == config_head)){
        return false;
    }
    *request = config_queue[config_tail];
    config_tail = (uint8_t)((config_tail + 1U) & RPI_CONFIG_QUEUE_MASK);
    return true;
}

bool RpiUart_SendConfigResponse(uint8_t seq, uint8_t status,
                                uint16_t param_id, uint16_t value,
                                uint8_t priority){
    uint8_t frame[RPI_UART_FRAME_SIZE];
    uint16_t used = (uint16_t)((tx_head - tx_tail) & RPI_TX_BUFFER_MASK);
    uint16_t free_bytes = (uint16_t)(RPI_TX_BUFFER_SIZE - 1U - used);
    if (free_bytes < RPI_UART_FRAME_SIZE){
        return false;
    }

    frame[0] = RPI_SYNC0;
    frame[1] = RPI_SYNC1;
    frame[2] = RPI_UART_CONFIG_RESPONSE;
    frame[3] = seq;
    frame[4] = status;
    frame[5] = (uint8_t)param_id;
    frame[6] = (uint8_t)(param_id >> 8U);
    frame[7] = (uint8_t)value;
    frame[8] = (uint8_t)(value >> 8U);
    frame[9] = priority;
    frame[10] = RpiUart_Crc8(&frame[2], 8U);

    for (uint8_t i = 0U; i < RPI_UART_FRAME_SIZE; i++){
        tx_buffer[tx_head] = frame[i];
        tx_head = (uint16_t)((tx_head + 1U) & RPI_TX_BUFFER_MASK);
    }
    DL_UART_Main_enableInterrupt(Rpi_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    return true;
}

static void RpiUart_TxIsr(void){
    while ((tx_tail != tx_head) && !DL_UART_Main_isTXFIFOFull(Rpi_UART_INST)){
        DL_UART_Main_transmitData(Rpi_UART_INST, tx_buffer[tx_tail]);
        tx_tail = (uint16_t)((tx_tail + 1U) & RPI_TX_BUFFER_MASK);
    }
    if (tx_tail == tx_head){
        DL_UART_Main_disableInterrupt(Rpi_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

void RpiUart_GetStats(RPI_UART_STATS *stats){
    if (stats == NULL){
        return;
    }
    *stats = diag;
    stats->uart_rx_bytes = uart_rx_bytes;
    stats->uart_rx_dropped = uart_rx_dropped;
    stats->uart_errors = uart_errors;
}

bool RpiUart_Predict(float theta_rad, RPI_UART_PREDICTION *prediction){
    if ((prediction == NULL) || !have_latest){
        return false;
    }

    uint32_t now = BSP_Time_GetMs();
    float age_ms = (float)latest.age_ms + RPI_UART_XFER_MS +
                   (float)(now - latest.rx_end_ms);
    if (age_ms > (float)RPI_UART_TIMEOUT_MS){
        return false;
    }
    float dt_s = age_ms * 0.001f;
    float measured_v = latest.velocity_trusted ? (float)latest.velocity_mm_s : 0.0f;
    float accel = BALL_ACCEL_MM_S2 * sinf(theta_rad);

    prediction->measured_x_mm = (float)latest.x_01mm * 0.1f;
    prediction->measured_velocity_mm_s = (float)latest.velocity_mm_s;
    prediction->x_mm = ((float)latest.x_01mm * 0.1f) + measured_v * dt_s +
                       0.5f * accel * dt_s * dt_s;
    prediction->velocity_mm_s = measured_v + accel * dt_s;
    prediction->age_ms = age_ms;
    prediction->degraded = age_ms > (float)RPI_UART_DEGRADED_AGE_MS;
    prediction->hold_output = !last_frame_valid;
    prediction->moving = (latest.flags & RPI_UART_FLAG_MOVING) != 0U;
    prediction->edge = (latest.flags & RPI_UART_FLAG_EDGE) != 0U;
    prediction->velocity_trusted = latest.velocity_trusted;
    prediction->position_extrapolated = true;
    prediction->quality = (uint8_t)((latest.flags & RPI_UART_FLAG_QUALITY_MASK) >> 4U);
    return true;
}

bool RpiUart_Observe(RPI_UART_PREDICTION *observation){
    if ((observation == NULL) || !have_latest){
        return false;
    }

    uint32_t now = BSP_Time_GetMs();
    float age_ms = (float)latest.age_ms + RPI_UART_XFER_MS +
                   (float)(now - latest.rx_end_ms);
    if (age_ms > (float)RPI_UART_TIMEOUT_MS){
        return false;
    }

    float measured_x = (float)latest.x_01mm * 0.1f;
    float measured_v = (float)latest.velocity_mm_s;
    bool moving = (latest.flags & RPI_UART_FLAG_MOVING) != 0U;
    bool extrapolate = moving && (measured_v != 0.0f);

    observation->measured_x_mm = measured_x;
    observation->measured_velocity_mm_s = measured_v;
    observation->x_mm = measured_x;
    if (extrapolate){
        observation->x_mm += measured_v * age_ms * 0.001f;
    }
    observation->velocity_mm_s = measured_v;
    observation->age_ms = age_ms;
    observation->degraded = age_ms > (float)RPI_UART_DEGRADED_AGE_MS;
    observation->hold_output = !last_frame_valid;
    observation->moving = moving;
    observation->edge = (latest.flags & RPI_UART_FLAG_EDGE) != 0U;
    observation->velocity_trusted = latest.velocity_trusted;
    observation->position_extrapolated = extrapolate;
    observation->quality =
        (uint8_t)((latest.flags & RPI_UART_FLAG_QUALITY_MASK) >> 4U);
    return true;
}

void Rpi_UART_INST_IRQHandler(void){
    switch (DL_UART_Main_getPendingInterrupt(Rpi_UART_INST)){
        case DL_UART_MAIN_IIDX_TX:
            RpiUart_TxIsr();
            break;
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(Rpi_UART_INST)){
                RPI_RX_ITEM item;
                item.byte = DL_UART_Main_receiveData(Rpi_UART_INST);
                item.arrival_ms = BSP_Time_GetMs();
                uart_rx_bytes++;

                uint16_t next = (uint16_t)((rx_head + 1U) & RPI_RX_BUFFER_MASK);
                if (next == rx_tail){
                    uart_rx_dropped++;
                } else{
                    rx_buffer[rx_head] = item;
                    rx_head = next;
                }
            }
            break;
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_BREAK_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            RpiUart_FlushHardwareRx();
            uart_errors++;
            break;
        default:
            break;
    }
}
