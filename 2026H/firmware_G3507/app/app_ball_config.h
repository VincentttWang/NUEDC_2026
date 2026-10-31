/**
 * @file  app_ball_config.h
 * @brief 水管滚球任务的实机标定参数。
 */
#ifndef APP_BALL_CONFIG_H
#define APP_BALL_CONFIG_H

/*
 * pygame_linkage_visualizer.py 的精确几何模型元数据。
 * 固件使用由该模型离线生成的单调查表，不在 Cortex-M0+ 上实时求 asin/atan2。
 * 几何尺寸和曲线形状已确认；当前水平相位按用户指定值标定为 180 cnt。
 */
#define APP_BALL_LINKAGE_MODEL_LEVEL_MIN_COUNTS 180.0f
#define APP_BALL_LINKAGE_MODEL_LEVEL_COUNTS 180.0f
#define APP_BALL_LINKAGE_MODEL_LEVEL_MAX_COUNTS 180.0f
#define APP_BALL_LINKAGE_MODEL_TABLE_STEP_COUNTS 20
#define APP_BALL_LINKAGE_MODEL_MAX_INTERP_ERROR_DEG 0.006127f
#define APP_BALL_LINKAGE_MODEL_GEOMETRY_VALIDATED 1U
#define APP_BALL_LINKAGE_MODEL_LEVEL_IS_NOMINAL 0U

/*
 * ===== H3 可调物理参数 =====
 * 几何表只描述名义曲线形状；测量误差通过 scale/offset 集中修正，禁止在表内散改点。
 * 实际角 = 名义查表角 * SCALE + OFFSET；反查计数时执行对应逆变换。
 */
#define APP_BALL_LINKAGE_ANGLE_SCALE_X10000      10000
#define APP_BALL_LINKAGE_ANGLE_SCALE \
    ((float)APP_BALL_LINKAGE_ANGLE_SCALE_X10000 * 0.0001f)
#define APP_BALL_LINKAGE_ANGLE_OFFSET_DEG        0.0000f

/* 纯滚动实心球名义值 (5/7)g；管身扭转、球体结构和滑动均可能使其变化。 */
#define APP_BALL_ROLLING_GAIN_MM_S2_PER_RAD      7004.75f
/* 动力学水平点相对修正后几何 0° 的初始先验，由守点数据继续学习。 */
#define APP_BALL_INITIAL_ZERO_TRIM_DEG            (-0.40f)

/* 在线终端约束轨迹参数，集中预留以便串口数据整定。 */
#define APP_BALL_PROFILE_MAX_VELOCITY_MM_S        90.0f
#define APP_BALL_PROFILE_MAX_ACCELERATION_MM_S2  280.0f
#define APP_BALL_PROFILE_MIN_DURATION_S            0.35f
#define APP_BALL_PROFILE_MAX_DURATION_S            3.00f
#define APP_BALL_PROFILE_LOOKAHEAD_S                0.12f

#if (APP_BALL_LINKAGE_ANGLE_SCALE_X10000 <= 0)
#error "APP_BALL_LINKAGE_ANGLE_SCALE_X10000 必须为正"
#endif

/* 已确认：cnt 增大时，水管向使球正向加速的方向倾斜。 */
#define APP_BALL_COUNT_POSITIVE_ACCEL_POSITIVE 1U

#if (APP_BALL_COUNT_POSITIVE_ACCEL_POSITIVE != 1U)
#error "滚球控制律按 cnt 增大=>球正向加速设计，机械方向不符时须显式修改控制符号"
#endif

#endif /* APP_BALL_CONFIG_H */
