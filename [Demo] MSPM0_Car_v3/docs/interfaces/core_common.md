# core/common 接口说明

## 模块职责

`core/common` 保存 core 层内部可复用的基础数据类型。

该模块不依赖 BSP、middleware 或 app。BSP 层也不应包含该头文件，避免底层反向依赖上层。

## 数据结构

### `CORE_POINT2F`

```c
typedef struct {
    float x;
    float y;
} CORE_POINT2F;
```

二维浮点坐标。当前用于纸面坐标、图像平面坐标和二维几何计算。

### `CORE_ATTITUDE2F`

```c
typedef struct {
    float yaw;
    float pitch;
} CORE_ATTITUDE2F;
```

二维姿态或二维角速度修正量。当前用于旧云台跟踪模块的 yaw/pitch 修正量，后续会随 `core/gimbal_tracking` 重写继续收敛语义。
