/**
 * @file  core_types.h
 * @brief Core 层公共基础数据类型。
 *
 * @note 预留：CORE_POINT2F 当前工程暂无调用者，保留供后续二维几何/坐标映射复用。
 */
#ifndef CORE_TYPES_H
#define CORE_TYPES_H

/**
 * @brief 二维浮点坐标点。
 * @note 预留：当前工程暂无调用者。
 */
typedef struct {
    /** X 坐标。 */
    float x;
    /** Y 坐标。 */
    float y;
} CORE_POINT2F;

#endif /* CORE_TYPES_H */
