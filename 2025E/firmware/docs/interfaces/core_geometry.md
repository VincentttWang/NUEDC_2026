# core/geometry 接口说明

## 模块职责

`core/geometry` 提供二维平面映射计算能力，属于纯算法模块。

该模块只处理点、矩形、插值和圆点等二维平面映射，不处理 CanMV 数据获取、灰度巡线、云台目标选择或 PID 控制。二维距离等通用运动学工具继续由 `core/kinematics` 提供。

## 数据结构

```c
typedef struct {
    CORE_POINT2F top_left;
    CORE_POINT2F top_right;
    CORE_POINT2F bottom_right;
    CORE_POINT2F bottom_left;
} GEOMETRY_RECT2F;
```

二维四边形矩形角点。角点顺序显式命名，避免继续扩散旧 `Rect_Loc[8]` 的隐式下标语义。

## 接口

### `CORE_POINT2F Geometry_Lerp2D(CORE_POINT2F start, CORE_POINT2F end, float ratio)`

对两个二维点做线性插值。

### `CORE_POINT2F Geometry_RectBilinearInterpolate(const GEOMETRY_RECT2F *rect, float u, float v)`

在矩形区域内做双线性插值。`u` 表示从左到右的归一化位置，`v` 表示从上到下的归一化位置。

### `CORE_POINT2F Geometry_PaperToRectPoint(CORE_POINT2F paper_point, float paper_width, float paper_height, const GEOMETRY_RECT2F *rect)`

将纸面坐标按宽高归一化后映射到矩形区域。该函数只做几何映射，不决定目标点来源。

### `CORE_POINT2F Geometry_CirclePointDeg(CORE_POINT2F center, float radius, float angle_deg)`

计算二维圆上指定角度的点。角度单位为度。

### `void Geometry_RectFromArray(const uint16_t rect_data[8], GEOMETRY_RECT2F *rect)`

将旧矩形数组转换为显式角点结构。数组下标约定如下：

```text
rect_data[0..1] = bottom_left
rect_data[2..3] = bottom_right
rect_data[4..5] = top_right
rect_data[6..7] = top_left
```

该函数只做数据布局转换，不读取 CanMV，也不判断识别状态。

## 边界说明

- 灰度传感器角度、巡线偏差、空线/半线/十字判断不属于本模块，后续由 `middleware/line_follow` 和 `middleware/line_tracking` 处理。
- 视觉目标选择、激光点跟踪、云台 PID 不属于本模块，后续由 `middleware/gimbal_tracking` 处理。
- 三维姿态和旋转矩阵不属于本模块，由 `core/rotation` 处理。
