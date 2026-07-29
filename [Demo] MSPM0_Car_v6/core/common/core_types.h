/**
 * @file  core_types.h
 * @brief Core 层公共基础数据类型。
 */
#ifndef CORE_TYPES_H
#define CORE_TYPES_H

/**
 * @brief 二维浮点坐标点。
 */
typedef struct {
    /** X 坐标。 */
    float x;
    /** Y 坐标。 */
    float y;
} CORE_POINT2F;

/**
 * @brief 二维云台/平面姿态角。
 */
typedef struct {
    /** yaw 角，单位 deg。 */
    float yaw;
    /** pitch 角，单位 deg。 */
    float pitch;
} CORE_ATTITUDE2F;

#endif /* CORE_TYPES_H */
