# 工程结构

## 顶层结构

当前工程采用“分层源码目录 + IDE 工程目录 + 板级配置目录”的组织方式：

- `app/`：应用入口和任务流程
- `core/`：控制算法和数据处理
- `middleware/`：系统组合能力与共享运行时状态
- `bsp/`：板级外设驱动
- `board/`：SysConfig、启动和链接资源
- `project/`：CCS 和 Keil 工程文件
- `docs/`：工程文档
- `tools/`：调试和烧录脚本

## 运行路径

典型运行路径如下：

1. `app/main.c` 完成系统初始化。
2. `app/app_launcher.c` 显示启动页。
3. 用户选择 E 题前三项任务、E1 的 1 到 5 圈圈数或设备检查页面。
4. 任务流程调用 `core` 中的控制算法。
5. `core` 通过 `middleware` 的组合接口和 `bsp` 的底层驱动控制硬件。

## app

`app` 是顶层应用层，负责：

- 主入口和中断分发
- 启动页
- 顶层任务菜单
- E 题基本要求 1、2、3 的任务流程，其中要求 1 提供 1 到 5 圈选择入口
- 设备自检页面

该层可以直接调用下层公开接口，但不应把纯算法或底层驱动细节继续塞入应用流程。

## core

`core` 放置和硬件无关或弱硬件相关的控制逻辑：

- `common/`：core 层基础数据类型。
- `pid/`：位置式和增量式 PID 控制器。
- `kinematics/`：角度、位姿、差速混控和二维几何计算。
- `rotation/`：欧拉角、旋转矩阵和三维向量旋转计算。
- `geometry/`：二维矩形插值、纸面到矩形映射和圆点计算。
- `line_tracking/`：巡线偏差计算、PID 修正和底盘输出。
- `gimbal_tracking/`：基于 CanMV 目标和 PID 的云台视觉跟踪控制。

该层可以使用 `middleware` 提供的组合能力，也可以调用必要的 `bsp` 数据接口，但不能包含 `app` 头文件。

## middleware

`middleware` 放置多个 BSP 外设组合后的系统能力：

- `chassis/`：底盘组合服务
- `gimbal/`：云台组合服务
- `line_follow/`：巡线运行状态服务
- `ui/`：轻量 OLED UI 渲染层
- `fault/`：系统故障处理服务

当前包括底盘服务、云台服务、巡线运行状态、轻量 UI 渲染和系统故障处理。旧 `runtime` 兼容头已在应用层重写后移除。

## bsp

`bsp` 是最低层，直接面对板级外设：

- `common/`
- `canmv/`
- `imu/`
- `key/`
- `motor/`
- `oled/`
- `step_motor/`
- `time/`
- `grayscale_sensor/`

该层禁止包含 `app`、`core`、`middleware` 头文件。需要基础时间或阻塞延时能力时，统一调用 `bsp/time`。

## board

`board` 保存板级配置和启动资源：

- `sys_config/`
- `startup/`

其中 SysConfig 生成文件内容不手改。

## project

`project` 保存 IDE 工程入口：

- `ccs/`
- `keil/`

源码路径重构后必须同步维护这两个目录中的工程文件。
