#include "visual.h"
#include "mode.h"
#include "string.h"

/* --- 外部变量 --- */
extern uint8_t Visual_Data[25]; // 来自 main.c 的、由中断填充的最新数据帧
extern MODE mode;               // 全局状态结构体

/* --- 模块内部静态变量 --- */

// 核心状态变量：存储学习到的“初始数字”。-1 表示还未学习。
static int s_initial_digit = -1;

// 用于检测命令“下降沿”的辅助变量
static bool s_previous_sampling_command = false;

// 学习阶段变量
static int s_initial_sample_count = 0;
static uint8_t s_initial_samples[INITIAL_SAMPLING_COUNT][MAX_DIGITS_PER_FRAME];
static uint8_t s_initial_sample_lengths[INITIAL_SAMPLING_COUNT];

// 导航阶段变量
#define NAV_MAX_SAMPLES 200 // 为导航阶段的单次任务设定一个最大的样本容量
static int s_nav_sample_count = 0;
static uint8_t s_nav_samples[NAV_MAX_SAMPLES][MAX_DIGITS_PER_FRAME];
static uint8_t s_nav_sample_lengths[NAV_MAX_SAMPLES];


/* --- 内部辅助函数 --- */

/**
 * @brief 通用的序列频率分析函数，找出样本中出现次数最多的完整序列。
 * @param samples_buffer 样本序列数组
 * @param lengths_buffer 样本长度数组
 * @param count 样本总数
 * @param out_sequence 用于传出最频繁的序列
 * @param out_len 用于传出最频繁序列的长度
 * @return 最频繁序列出现的次数
 */
static int find_most_frequent_sequence(
    uint8_t samples_buffer[][MAX_DIGITS_PER_FRAME],
    const uint8_t lengths_buffer[],
    int count,
    uint8_t* out_sequence,
    uint8_t* out_len)
{
    if (count == 0) return 0;

    int max_freq = 0;
    for (int i = 0; i < count; i++) {
        if (lengths_buffer[i] == 0) continue;
        int current_freq = 0;
        for (int j = 0; j < count; j++) {
            if (lengths_buffer[i] == lengths_buffer[j] &&
                memcmp(samples_buffer[i], samples_buffer[j], lengths_buffer[i]) == 0) {
                current_freq++;
            }
        }
        if (current_freq > max_freq) {
            max_freq = current_freq;
            *out_len = lengths_buffer[i];
            memcpy(out_sequence, samples_buffer[i], *out_len);
        }
    }
    return max_freq;
}

/**
 * @brief 分析“初始学习”的稳定性。
 */
static bool analyze_initial_stability(uint8_t* out_digit)
{
    uint8_t most_frequent_sequence[MAX_DIGITS_PER_FRAME];
    uint8_t most_frequent_len = 0;
    int max_freq = find_most_frequent_sequence(s_initial_samples, s_initial_sample_lengths, s_initial_sample_count, most_frequent_sequence, &most_frequent_len);

    // 检查是否满足阈值，并且必须是单个数字(1-8)
    if (max_freq >= INITIAL_STABILITY_THRESHOLD && most_frequent_len == 1 && most_frequent_sequence[0] >= 1 && most_frequent_sequence[0] <= 8) {
        *out_digit = most_frequent_sequence[0];
        return true;
    }
    return false;
}

/**
 * @brief 分析“导航”的稳定性。
 */
static bool analyze_nav_stability(uint8_t* out_sequence, uint8_t* out_len)
{
    if (s_nav_sample_count == 0) return false;
    
    int max_freq = find_most_frequent_sequence(s_nav_samples, s_nav_sample_lengths, s_nav_sample_count, out_sequence, out_len);

    // 检查是否满足概率阈值
    if (((float)max_freq / s_nav_sample_count) >= NAV_STABILITY_PROBABILITY) {
        return true;
    }
    return false;
}


/* --- 对外公共函数实现 --- */

void visual_reset_sampling_status(void)
{
    s_initial_sample_count = 0;
    memset(s_initial_samples, 0, sizeof(s_initial_samples));
    memset(s_initial_sample_lengths, 0, sizeof(s_initial_sample_lengths));
    
    s_nav_sample_count = 0;
    memset(s_nav_samples, 0, sizeof(s_nav_samples));
    memset(s_nav_sample_lengths, 0, sizeof(s_nav_sample_lengths));
}
void visual_full_reset(void)
{
    s_initial_digit = -1;
    s_previous_sampling_command = false;
    visual_reset_sampling_status();
}


void visual_process_command(bool* sampling_command)
{
    // --- “下降沿”检测：当命令由 true -> false 时，执行“导航阶段”的最终分析 ---
    if (*sampling_command == false && s_previous_sampling_command == true)
    {
        if (s_initial_digit != -1) { // 只有在已学习过初始数字后，这个下降沿才有意义
            uint8_t stable_sequence[MAX_DIGITS_PER_FRAME];
            uint8_t stable_len;
            if (analyze_nav_stability(stable_sequence, &stable_len)) {
                bool target_found = false;
                for (int i = 0; i < stable_len; i++) {
                    if (stable_sequence[i] == s_initial_digit) {
                        mode.dir = (i < stable_len / 2) ? LEFT : RIGHT;
                        target_found = true;
                        break;
                    }
                }
                if (!target_found) mode.dir = FORWARD;
            } else {
                mode.dir = UNSTABLE;
            }
        }
    }

    // --- 数据收集阶段：当命令为 true 时执行 ---
    if (*sampling_command == true)
    {
        uint8_t current_sequence[MAX_DIGITS_PER_FRAME];
        uint8_t current_length = 0;
        if (Visual_Data[0] == FRAME_HEADER && Visual_Data[Visual_Data[1] + 2] == FRAME_TRAILER) {
            current_length = Visual_Data[1];
            if (current_length > 0 && current_length <= MAX_DIGITS_PER_FRAME) {
                memcpy(current_sequence, &Visual_Data[2], current_length);
            } else {
                current_length = 0;
            }
        }
        if (current_length == 0) { // 如果帧无效，直接更新状态并返回
             s_previous_sampling_command = *sampling_command;
             return;
        }

        // --- 根据是否已学习，分发任务 ---
        if (s_initial_digit == -1) {
            /***** 学习阶段 (自动结束) *****/
            if (s_initial_sample_count < INITIAL_SAMPLING_COUNT) {
                memcpy(s_initial_samples[s_initial_sample_count], current_sequence, current_length);
                s_initial_sample_lengths[s_initial_sample_count] = current_length;
                s_initial_sample_count++;
            }
            
            if (s_initial_sample_count >= INITIAL_SAMPLING_COUNT) {
                uint8_t learned_digit;
                if (analyze_initial_stability(&learned_digit)) {
                    s_initial_digit = learned_digit;
                    if (learned_digit == 1) mode.dir = LEFT;
                    else if (learned_digit == 2) mode.dir = RIGHT;
                    else mode.dir = FORWARD;
                } else {
                    mode.dir = UNSTABLE;
                }
                *sampling_command = false; // 任务完成，自动将命令标志位置为 false
            }
        } else {
            /***** 导航阶段 (只收集数据，等待手动结束) *****/
            if (s_nav_sample_count < NAV_MAX_SAMPLES) {
                memcpy(s_nav_samples[s_nav_sample_count], current_sequence, current_length);
                s_nav_sample_lengths[s_nav_sample_count] = current_length;
                s_nav_sample_count++;
            }
        }
    }

    // 在函数的最后，更新上一次的命令状态，为下一次检测做准备
    s_previous_sampling_command = *sampling_command;
}