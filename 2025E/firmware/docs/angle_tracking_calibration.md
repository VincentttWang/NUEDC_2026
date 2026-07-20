# K230 角度追踪标定方法

本流程对应 `k230/aim_track_angle.py`，其测量与 `rect11_lcd.py` 一致：先检测矩形，再从标定瞄准点的像素位置计算相对相机光轴的 yaw/pitch 角，发送有符号角度误差。

## 0. 固件前提

脚本发送 9 字节帧：`A5 5A status seq yaw_hi yaw_lo pitch_hi pitch_lo checksum`。其中 yaw/pitch 是 `deg × 100` 的 `int16` 大端数，`status` 表示有效、未找到或丢失，`seq` 是滚动帧序号，校验是前八字节之和的低八位。

固件的 `bsp/canmv` 已支持本帧，并在 `GimbalTracking_UpdateLaserCenter()` 收到第一帧后自动切换到角度模式。该模式使用独立的 yaw/pitch 角度 PID，原 `0x12 ... 0x5B` 坐标帧仍保留给矩形扫描和旧脚本。

## 0.5 蓝紫色光点的人工标定

脚本以 RGB565 图像检测蓝紫色光点，但光点只用于 LCD 叠加核对，**不会**被用于控制误差、参数学习或自动覆盖标定值。

1. 让激光实际命中靶心，读取 LCD 上品红色光点的像素坐标。
2. 将该坐标人工填写到 `MANUAL_LASER_REF_X_PX/Y_PX`；它是固定参考距离下激光相对相机的像素参考点。
3. 在同一距离复测残余角误差，按第 4 节填写 `YAW_STATIC_OFFSET_DEG/PITCH_STATIC_OFFSET_DEG`。
4. 换距离时测得的残差，按第 5 节手动拟合并填写视差系数。

`LASER_PURPLE_THRESHOLD` 仅影响 LCD 上是否能画出光点，不会改变云台控制；可按实际光照用 IDE 的色块工具调节。

## 1. 固定坐标和目标定义

1. 固定相机、云台和靶纸，确认 `MIRROR_180`。之后所有标定都必须使用相同值。
2. 用卡尺测量黑色矩形的实际高度，填写 `TARGET_HEIGHT_MM`。
3. 明确激光实际应命中的位置。靶心填 `CALIB_OFFSET_Y_MM = 0`；若要求在靶心上方 12 mm，先填 `12.0`。
4. 将靶纸正对相机、放在常用距离，先令 `IDEAL_YAW_DEG/IDEAL_PITCH_DEG` 与静态偏置均为零。

脚本沿用 `rect11_lcd.py` 的近似：`偏移像素 = 矩形像素高 × 物理偏移 / 靶高`。靶纸正对相机时准确度最好；若靶纸倾斜明显，后续应由四角单应性映射替换该近似。

## 2. 标定光学中心 `CX/CY`

保持云台不动，移动靶纸，使实际瞄准点分别出现在画面左、右、上、下。记录每次“激光恰好命中瞄准点”时脚本打印的 `yaw_err/pitch_err`。

- 调整 `CAMERA_CX_PX`，使左、右两点的 yaw 误差大小相等、符号相反。
- 调整 `CAMERA_CY_PX`，使上、下两点的 pitch 误差大小相等、符号相反。

不要直接假定光学中心一定是 `(160, 120)`；镜头安装偏心和裁剪都会改变它。

## 3. 标定焦距 `FX/FY`（推荐）

令云台在机械零位，靶纸平面大致正对镜头。用已知角度让靶的瞄准点偏离光轴，记录像素坐标 `u/v` 和真实角度 `a`。

\[
f_x = \frac{u-c_x}{\tan(a_{yaw})}, \qquad
f_y = \frac{v-c_y}{\tan(a_{pitch})}
\]

在左右、上下各取至少 5 个点，去掉异常值后取均值，分别填入 `CAMERA_FX_PX/FY_PX`。若暂时无法提供真实转角，可将它们保持为 `0`，脚本会使用：

\[
f_x=\frac{W/2}{\tan(HFOV/2)},\qquad f_y=\frac{H/2}{\tan(VFOV/2)}
\]

并通过调整 `H_FOV_DEG/V_FOV_DEG` 获得近似值；这只适合作为初值。

## 4. 标定静态零偏

在参考距离 `REFERENCE_DISTANCE_MM`，让激光真实命中瞄准点，保持云台稳定，记录 30 帧 `yaw_err/pitch_err`，取平均值 `e_yaw/e_pitch`。

将平均值填入：

```python
YAW_STATIC_OFFSET_DEG = e_yaw
PITCH_STATIC_OFFSET_DEG = e_pitch
```

重启后，同一姿态下误差应接近 `0°`。若误差变大，说明符号填反；将该偏置改为负值。

## 5. 标定相机—激光视差（近中远必做）

相机与激光、云台旋转中心并不共点，故零偏随距离变化。分别在例如 `500 mm / 1000 mm / 2000 mm` 距离，重复步骤 4，得到校正前误差 `e(d)`。

脚本使用：

\[
e_{parallax}=K\left(\frac{1}{d}-\frac{1}{d_{ref}}\right)
\]

对每一轴由两个距离估算：

\[
K=\frac{e(d_1)-e(d_2)}{1/d_1-1/d_2}
\]

将结果填入 `YAW_PARALLAX_K_DEG_MM/PITCH_PARALLAX_K_DEG_MM`。然后复测第三个距离；若残差仍呈明显非线性，应检查靶纸倾斜、距离估计和激光安装，而不是直接提高 PID 增益。

## 6. 验证方向与 PID

在 MCU 仍限制低速的条件下，人工将靶点移到画面右侧、下侧，确认角度帧符号与电机动作能让误差减小。若某轴越追越远，优先翻转 MCU 的 `yaw_output_sign` 或 `pitch_output_sign`，不要篡改相机角度定义。

最后以固定距离、相同初始偏差做阶跃测试：先只调 `Kp` 到响应足够快且不持续摆动，再加很小的 `Kd` 抑制振荡；视觉闭环通常不建议一开始加入 `Ki`，以免丢帧和静态安装偏差造成积分累积。
