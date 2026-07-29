# core/rotation 接口说明

## 模块职责

`core/rotation` 提供欧拉角、旋转矩阵和三维向量旋转计算能力，属于纯数学模块。

该模块不读取传感器、不控制外设、不保存全局状态。调用方负责提供输入数据并使用输出结果。

## 数据结构

```c
typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
} ROTATION_EULER;
```

欧拉角，单位为度。

```c
typedef struct {
    float value[3][3];
} ROTATION_MATRIX;
```

三阶旋转矩阵。矩阵元素按 `value[row][col]` 访问。

## 接口

### `void Rotation_MatrixIdentity(ROTATION_MATRIX *matrix)`

将矩阵设置为单位矩阵。传入空指针时不执行任何操作。

### `void Rotation_EulerToMatrix(const ROTATION_EULER *euler, ROTATION_MATRIX *matrix)`

根据 `yaw_deg`、`pitch_deg`、`roll_deg` 生成旋转矩阵。输入角度单位为度。

### `void Rotation_MatrixToEuler(const ROTATION_MATRIX *matrix, ROTATION_EULER *euler)`

从旋转矩阵提取欧拉角。输出角度单位为度。

### `void Rotation_MatrixMultiply(const ROTATION_MATRIX *left, const ROTATION_MATRIX *right, ROTATION_MATRIX *out)`

计算三阶矩阵乘法：

```text
out = left * right
```

函数内部使用临时矩阵，因此 `out` 可以和 `left` 或 `right` 指向同一个对象。

### `void Rotation_MatrixTranspose(const ROTATION_MATRIX *matrix, ROTATION_MATRIX *out)`

计算三阶矩阵转置。函数内部使用临时矩阵，因此 `out` 可以和 `matrix` 指向同一个对象。

### `void Rotation_VectorApply(const ROTATION_MATRIX *matrix, const float input[3], float output[3])`

将旋转矩阵应用到三维向量：

```text
output = matrix * input
```

函数内部使用临时数组，因此 `output` 可以和 `input` 指向同一个数组。

## 迁移说明

旧接口 `rotation_matrix()`、`matrix_multiplication()`、`matrix_transpose()` 和 `matrix_to_angles()` 已移除。当前源码中没有外部模块调用这些旧接口，后续需要旋转计算时应使用 `Rotation_*` 接口。
