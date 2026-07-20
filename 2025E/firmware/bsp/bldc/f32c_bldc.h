/**
 * @file  f32c_bldc.h
 * @brief F32C 无刷电机驱动协议定义与 API
 *
 * 通信接口: UART3 (BLDC_INST, PB3=RX / PB12=TX), 115200 8N1
 * 协议: 自定义二进制帧, 帧头 0x7A / 帧尾 0x7B, BCC 异或校验
 * 参考: WHEELTEC F32C TTL 无刷电机驱动例程 (TI MSPM0 版本)
 */

#ifndef F32C_BLDC_H
#define F32C_BLDC_H

#include <stdint.h>

/* ===== 帧分隔符 ===== */
#define BLDC_HEADER  0x7AU
#define BLDC_TAIL    0x7BU

/* ===== 电机地址 ===== */
#define BLDC_ADDR_1  0x01U
#define BLDC_ADDR_2  0x02U

/* ===== 命令码 ===== */
#define CMD_ENABLE       0x06U   /* 使能电机 */
#define CMD_DISABLE      0x05U   /* 失能电机 */
#define CMD_MODE         0x00U   /* 设置控制模式 */
#define CMD_SPEED        0x01U   /* 设置目标转速 (RPM) */
#define CMD_MULTI_POS    0x02U   /* 设置多圈目标位置 (0.1deg) */
#define CMD_SINGLE_POS   0x03U   /* 设置单圈角度 (0.1deg, 0~3599) */
#define CMD_FEEDBACK     0x0EU   /* 请求反馈数据 */
#define CMD_ACC          0x07U   /* 设置加速度 (RPM/s) */
#define CMD_SAVE         0x08U   /* 保存参数到驱动 Flash */
#define CMD_CLEAR_MULTI  0x09U   /* 清除多圈角度 */
#define CMD_SET_ZERO     0x0AU   /* 设置当前角度为单圈零点 */
#define CMD_FACTORY_RST  0x0BU   /* 恢复出厂设置 */
#define CMD_SET_ADDR     0x0DU   /* 修改电机地址 */

/* ===== 控制模式 ===== */
#define MODE_SPEED          0x0000U   /* 速度闭环 */
#define MODE_MULTI_POS      0x0001U   /* 多圈位置 */
#define MODE_SINGLE_POS     0x0002U   /* 单圈位置 */
#define MODE_MULTI_POS_L    0x0003U
#define MODE_SINGLE_POS_L   0x0004U

/* ===== 俯仰(上下)电机配置 ===== */
/* 控制上下俯仰的无刷电机地址. 若接线相反, 改为 BLDC_ADDR_2 */
#ifndef BLDC_PITCH_ADDR
#define BLDC_PITCH_ADDR   BLDC_ADDR_2
#endif
/* 位置限位与启动角, 单位 0.1° (即 *10): 起始 0°, 正向最大 180°, 启动正转 120° */
#define BLDC_PITCH_MIN_X10    0
#define BLDC_PITCH_MAX_X10    1800
#define BLDC_PITCH_INIT_X10   1500
/* 位置模式下移动到目标角的运动速度 (RPM). 位置模式必须设速度, 否则默认 0 不动.
 * 它是 pitch 能跟随的【最大速率】: 太低则大幅 pitch 修正/入弯抬升会滞后。
 * 30→50 加快 pitch 调整速度(约 1.6×); 与震荡是权衡——速度越低逼近越温和、过冲越小,
 * 若提速后到位过冲/震荡明显则回调到 40 或减小 ACC。 */
#define BLDC_PITCH_SPEED      50
/* 位置模式下的加速度限制 (转/s², 经 0x07 命令下发).
 * 20→32 配合提速让加减速更快; 值越小加减速越平滑、到位过冲越小但响应越慢。
 * 提速后若过冲明显, 先减小本值再考虑降 SPEED。请在实机上按震荡情况微调。 */
#define BLDC_PITCH_ACC        32

/* ===== 反馈类型 ===== */
#define FB_SPEED         0x00U   /* 转速 (RPM) */
#define FB_MULTI_ANGLE   0x01U   /* 多圈角度 (0.1deg) */
#define FB_SINGLE_ANGLE  0x02U   /* 单圈角度 (0.1deg) */
#define FB_ACC           0x03U   /* 加速度 */
#define FB_VOLTAGE       0x04U   /* 母线电压 (0.01V) */

/* ===== 反馈数据结构体 ===== */
typedef struct {
    int16_t  speed;         /* 转速 (RPM) */
    int32_t  multi_angle;   /* 多圈角度 (*10, 即 0.1deg/bit) */
    uint16_t single_angle;  /* 单圈角度 (*10, 0~3599) */
    int16_t  acc;           /* 加速度 (转/s²) */
    uint16_t voltage;       /* 母线电压 (*100, 即 0.01V/bit) */
    uint8_t  data_ready;    /* 数据更新标志 */
    uint32_t multi_angle_frame_count; /* 通过校验的多圈角反馈帧累计数 */
} BLDC_MotorData_t;

/* ===== 全局电机数据 ===== */
extern volatile BLDC_MotorData_t BLDC_Motor1;
extern volatile BLDC_MotorData_t BLDC_Motor2;

/* ===== API ===== */
void BLDC_Init(void);
void BLDC_SendCmd(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t len);
void BLDC_Enable(uint8_t addr);
void BLDC_Disable(uint8_t addr);
void BLDC_SetSpeed(uint8_t addr, int16_t rpm);
void BLDC_SetMode(uint8_t addr, uint16_t mode);
void BLDC_SetMultiAngle(uint8_t addr, int32_t angle_x10);
void BLDC_SetSingleAngle(uint8_t addr, uint16_t angle_x10);
void BLDC_ReqFeedback(uint8_t addr, uint8_t type);
void BLDC_SetAcc(uint8_t addr, uint16_t acc);
void BLDC_SaveParams(uint8_t addr);
void BLDC_ClearMultiAngle(uint8_t addr);
void BLDC_SetSingleAngleZero(uint8_t addr);
void BLDC_FactoryReset(uint8_t addr);
void BLDC_SetAddress(uint8_t addr, uint8_t new_addr);

/* 设置某地址多圈位置软件限位 (0.1° 单位). min==max 时取消限位 */
void BLDC_SetAngleLimit(uint8_t addr, int32_t min_x10, int32_t max_x10);
/* 启动俯仰电机: 使能→多圈位置模式→当前位置清零(0°)→设限位→正转到约120° */
void BLDC_StartupPitch(void);

/* 由 UART ISR 调用 */
void BLDC_ParseRxByte(uint8_t rx_byte);

#endif /* F32C_BLDC_H */
