/**
 * @file  rotation.h
 * @brief Core 层三维欧拉角、旋转矩阵和向量旋转工具。
 */
#ifndef ROTATION_H
#define ROTATION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief yaw-pitch-roll 欧拉角，单位 deg。
 */
typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
} ROTATION_EULER;

/**
 * @brief 3x3 旋转矩阵。
 */
typedef struct {
    float value[3][3];
} ROTATION_MATRIX;

/**
 * @brief 设置为单位矩阵。
 * @param matrix 待写入矩阵。
 */
void Rotation_MatrixIdentity(ROTATION_MATRIX *matrix);

/**
 * @brief 将欧拉角转换为旋转矩阵。
 * @param euler 输入欧拉角。
 * @param matrix 输出旋转矩阵。
 */
void Rotation_EulerToMatrix(const ROTATION_EULER *euler, ROTATION_MATRIX *matrix);

/**
 * @brief 将旋转矩阵转换为欧拉角。
 * @param matrix 输入旋转矩阵。
 * @param euler 输出欧拉角。
 */
void Rotation_MatrixToEuler(const ROTATION_MATRIX *matrix, ROTATION_EULER *euler);

/**
 * @brief 矩阵乘法 out = left * right。
 */
void Rotation_MatrixMultiply(const ROTATION_MATRIX *left,
                             const ROTATION_MATRIX *right,
                             ROTATION_MATRIX *out);

/**
 * @brief 计算旋转矩阵转置。
 */
void Rotation_MatrixTranspose(const ROTATION_MATRIX *matrix, ROTATION_MATRIX *out);

/**
 * @brief 将旋转矩阵作用到三维向量上。
 * @param matrix 旋转矩阵。
 * @param input 输入向量，长度为 3。
 * @param output 输出向量，长度为 3。
 */
void Rotation_VectorApply(const ROTATION_MATRIX *matrix,
                          const float input[3],
                          float output[3]);

#ifdef __cplusplus
}
#endif

#endif /* ROTATION_H */
