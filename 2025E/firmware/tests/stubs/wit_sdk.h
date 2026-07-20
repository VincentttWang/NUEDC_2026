#ifndef TEST_STUB_WIT_SDK_H
#define TEST_STUB_WIT_SDK_H

#define WIT_HAL_OK 0

typedef struct {
    float x;
    float y;
    float z;
} WIT_VECTOR3F;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} WIT_ATTITUDE;

typedef struct {
    WIT_VECTOR3F acc_g;
    WIT_VECTOR3F gyro_deg_s;
    WIT_ATTITUDE attitude_deg;
    float temperature_c;
} WIT_IMU_DATA;

int WitGetData(WIT_IMU_DATA *out);

#endif
