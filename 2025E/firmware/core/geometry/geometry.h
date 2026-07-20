/**
 * @file  geometry.h
 * @brief Core 层二维几何映射和插值工具。
 */
#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "common/core_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 按左上、右上、右下、左下顺序描述的二维四边形。
 */
typedef struct {
    CORE_POINT2F top_left;
    CORE_POINT2F top_right;
    CORE_POINT2F bottom_right;
    CORE_POINT2F bottom_left;
} GEOMETRY_RECT2F;

/**
 * @brief 二维点线性插值。
 * @param start 起点。
 * @param end 终点。
 * @param ratio 插值比例，0 表示起点，1 表示终点。
 * @return 插值结果。
 */
CORE_POINT2F Geometry_Lerp2D(CORE_POINT2F start,
                             CORE_POINT2F end,
                             float ratio);

/**
 * @brief 在四边形内部做双线性插值。
 * @param rect 四边形描述。
 * @param u 水平方向归一化坐标，通常范围 [0, 1]。
 * @param v 垂直方向归一化坐标，通常范围 [0, 1]。
 * @return 插值得到的二维点；rect 为 NULL 时返回零点。
 */
CORE_POINT2F Geometry_RectBilinearInterpolate(const GEOMETRY_RECT2F *rect,
                                              float u,
                                              float v);

/**
 * @brief 将纸面坐标映射到图像/空间四边形坐标。
 * @param paper_point 纸面坐标点。
 * @param paper_width 纸面宽度。
 * @param paper_height 纸面高度。
 * @param rect 目标四边形。
 * @return 映射后的二维点。
 */
CORE_POINT2F Geometry_PaperToRectPoint(CORE_POINT2F paper_point,
                                       float paper_width,
                                       float paper_height,
                                       const GEOMETRY_RECT2F *rect);

/**
 * @brief 计算圆上指定角度对应的点。
 * @param center 圆心。
 * @param radius 半径。
 * @param angle_deg 角度，单位 deg。
 * @return 圆上点坐标。
 */
CORE_POINT2F Geometry_CirclePointDeg(CORE_POINT2F center,
                                     float radius,
                                     float angle_deg);

/**
 * @brief 将 CanMV 矩形数组转换为 GEOMETRY_RECT2F。
 * @param rect_data 8 个无符号整数，顺序为 x0,y0,x1,y1,x2,y2,x3,y3。
 * @param rect 输出四边形。
 */
void Geometry_RectFromArray(const uint16_t rect_data[8], GEOMETRY_RECT2F *rect);

#ifdef __cplusplus
}
#endif

#endif /* GEOMETRY_H */
