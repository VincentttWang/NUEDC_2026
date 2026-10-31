/**
 * @file  app_ball_tune.h
 * @brief 滚球控制参数的串口热更新（调试用，比赛时可整块关掉）。
 *
 * 目的：整定一个参数不必重新编译烧录。整定链路本来是
 *
 *     改 .c → 编译 → 烧录 → 摆球 → 抓 100 s → 分析      （约 5 分钟一轮）
 *
 * 有了热更之后变成
 *
 *     发一行 `s kp 0.05` → 摆球 → 看遥测                （约 20 秒一轮）
 *
 * ============ 安全设计（这部分比功能本身重要）============
 *
 * 参数直接喂给一个会驱动水管的控制律，打错一个小数点就可能把摆杆甩到端点。
 * 因此每一项都**必须**带上下界，且下界/上界由物理约束推出而不是拍脑袋：
 *
 *   - kp 上界由「静摩擦残差 θ_stick/Kp」与「环路裕度」共同夹出；
 *   - 抖动幅值上界由「2πf·A < 执行器角速率上限」反推；
 *   - 角度限幅上界由查表在软限位处的物理可达角决定。
 *
 * 越界一律**拒绝并回报**，不做静默钳位——静默钳位会让人以为设进去了。
 *
 * ⚠ 本模块只改内存里的配置，**不写 flash**。掉电即恢复编译期默认值。
 *   这是刻意的：整定出来的值必须回填源码并提交，才算数。
 *
 * ============ 协议 ============
 *
 * ASCII 行，`\n` 或 `\r` 结束，不区分大小写。回显一律以 `[TUNE]` 起头，
 * 便于上位机从 `[SCV]` 遥测流里分离出来。
 *
 *   ?  | list          列出全部参数
 *   g <名>             读一个
 *   s <名> <值>        写一个
 *   d                  恢复编译期默认值
 *
 * 回显（设计成可被机器解析）：
 *
 *   [TUNE] p name=kp val=0.047110 min=0.010000 max=0.150000 unit=deg/mm
 *   [TUNE] ok name=kp val=0.050000 old=0.047110
 *   [TUNE] err name=kp reason=range min=0.010000 max=0.150000
 *   [TUNE] err reason=unknown name=kpp
 *   [TUNE] end n=24
 */
#ifndef APP_BALL_TUNE_H
#define APP_BALL_TUNE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 一条可热更的参数。 */
typedef struct {
    /** 命令里用的短名（小写，无空格）。 */
    const char *name;
    /**
     * 指向配置里的字段。为 NULL 时表示这一项不在配置结构体里，
     * 改由 getter/setter 存取（例如步进伺服增益在驱动内部）。
     */
    float *value;
    /** 允许范围，闭区间。越界拒绝而非钳位。 */
    float min_value;
    float max_value;
    /** 单位，仅用于回显。 */
    const char *unit;
    /** value == NULL 时使用的读写钩子。 */
    float (*get)(void);
    bool  (*set)(float);
} APP_BALL_TUNE_ENTRY;

/**
 * @brief 绑定参数表并快照当前值作为「编译期默认」。
 * @param table 静态存活的参数表（本模块只保存指针，不拷贝）。
 * @param count 表项数，不超过 APP_BALL_TUNE_MAX_ENTRIES。
 * @note 必须在任务 Enter 里调用；Exit 时不需要反注册。
 */
void AppBallTune_Init(const APP_BALL_TUNE_ENTRY *table, uint8_t count);

/**
 * @brief 收字节、拼行、执行命令。须在控制拍里周期调用。
 * @note 非阻塞。单拍最多处理一整行，剩余字节留到下一拍，
 *       避免有人粘贴一大段导致控制回调超时。
 */
void AppBallTune_Poll(void);

/**
 * @brief 启动完整参数表回显，上位机据此建立面板。
 * @note 非阻塞：实际回显由后续 AppBallTune_Poll() 每拍发送一项，避免挤爆 UART TX 缓冲。
 */
void AppBallTune_PrintAll(void);

/** 参数表是否仍在分拍回显；调用方可据此暂缓大带宽遥测。 */
bool AppBallTune_IsListing(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BALL_TUNE_H */
