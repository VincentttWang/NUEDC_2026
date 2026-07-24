#include "rotation.h"

#include "kinematics/kinematics.h"

#include <math.h>
#include <stddef.h>

void Rotation_MatrixIdentity(ROTATION_MATRIX *matrix){
    if (matrix == NULL){
        return;
    }

    for (int row = 0; row < 3; row++){
        for (int col = 0; col < 3; col++){
            matrix->value[row][col] = (row == col) ? 1.0f : 0.0f;
        }
    }
}

void Rotation_EulerToMatrix(const ROTATION_EULER *euler, ROTATION_MATRIX *matrix){
    if ((euler == NULL) || (matrix == NULL)){
        return;
    }

    float cy = cosf(KINEMATICS_DEG_TO_RAD(euler->yaw_deg));
    float sy = sinf(KINEMATICS_DEG_TO_RAD(euler->yaw_deg));
    float cp = cosf(KINEMATICS_DEG_TO_RAD(euler->pitch_deg));
    float sp = sinf(KINEMATICS_DEG_TO_RAD(euler->pitch_deg));
    float cr = cosf(KINEMATICS_DEG_TO_RAD(euler->roll_deg));
    float sr = sinf(KINEMATICS_DEG_TO_RAD(euler->roll_deg));

    matrix->value[0][0] = cy * cp;
    matrix->value[0][1] = cy * sp * sr - sy * cr;
    matrix->value[0][2] = cy * sp * cr + sy * sr;

    matrix->value[1][0] = sy * cp;
    matrix->value[1][1] = sy * sp * sr + cy * cr;
    matrix->value[1][2] = sy * sp * cr - cy * sr;

    matrix->value[2][0] = -sp;
    matrix->value[2][1] = cp * sr;
    matrix->value[2][2] = cp * cr;
}

void Rotation_MatrixToEuler(const ROTATION_MATRIX *matrix, ROTATION_EULER *euler){
    if ((matrix == NULL) || (euler == NULL)){
        return;
    }

    euler->yaw_deg = KINEMATICS_RAD_TO_DEG(atan2f(matrix->value[1][0],
                                                 matrix->value[0][0]));
    euler->pitch_deg = KINEMATICS_RAD_TO_DEG(
        atan2f(-matrix->value[2][0],
               sqrtf(matrix->value[2][1] * matrix->value[2][1] +
                     matrix->value[2][2] * matrix->value[2][2])));
    euler->roll_deg = KINEMATICS_RAD_TO_DEG(atan2f(matrix->value[2][1],
                                                   matrix->value[2][2]));
}

void Rotation_MatrixMultiply(const ROTATION_MATRIX *left,
                             const ROTATION_MATRIX *right,
                             ROTATION_MATRIX *out){
    if ((left == NULL) || (right == NULL) || (out == NULL)){
        return;
    }

    ROTATION_MATRIX result;

    for (int row = 0; row < 3; row++){
        for (int col = 0; col < 3; col++){
            result.value[row][col] = 0.0f;
            for (int idx = 0; idx < 3; idx++){
                result.value[row][col] += left->value[row][idx] * right->value[idx][col];
            }
        }
    }

    *out = result;
}

void Rotation_MatrixTranspose(const ROTATION_MATRIX *matrix, ROTATION_MATRIX *out){
    if ((matrix == NULL) || (out == NULL)){
        return;
    }

    ROTATION_MATRIX result;

    for (int row = 0; row < 3; row++){
        for (int col = 0; col < 3; col++){
            result.value[row][col] = matrix->value[col][row];
        }
    }

    *out = result;
}

void Rotation_VectorApply(const ROTATION_MATRIX *matrix,
                          const float input[3],
                          float output[3]){
    if ((matrix == NULL) || (input == NULL) || (output == NULL)){
        return;
    }

    float result[3] = {0.0f, 0.0f, 0.0f};

    for (int row = 0; row < 3; row++){
        for (int col = 0; col < 3; col++){
            result[row] += matrix->value[row][col] * input[col];
        }
    }

    output[0] = result[0];
    output[1] = result[1];
    output[2] = result[2];
}
