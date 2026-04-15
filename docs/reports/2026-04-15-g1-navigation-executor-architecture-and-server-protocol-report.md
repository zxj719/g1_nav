# G1 导航执行端架构与服务端协议报告

## 1. 背景与目的

本文档用于说明当前 `g1_nav` 中这套“云端服务端 + 本机导航执行端”架构的设计意图、模块边界、协议约束与排障入口，重点服务于以下场景：

- 合并代码时快速判断改动是否破坏了原有导航链路
- 服务端联调时明确消息格式与职责归属
- 出现“能连上但不动”“POI 标记成功但后续导航异常”“SLAM 回环后 POI 漂移”等问题时，快速定位问题层级

当前方案的核心设计原则是：

- 服务端只负责语义层，不持有机器人侧的几何真值
- 执行端持有本地 POI 几何信息，并直接对接 SLAM、TF、Nav2
- WebSocket 协议尽量沿用现有导航消息格式，新增 `mark_current_poi` 能力
- SLAM 后端回环引起 `map->odom` 变化时，由执行端本地重投影 POI，而不是要求服务端回写地图坐标

## 2. 总体架构

当前运行链路由四部分组成：

1. 云端服务端
   负责接收大脑模块的抽象意图，向机器人下发语义导航命令或 POI 标记命令，并接收执行结果。
2. 机器人本机 `navigation_executor.py`
   作为 WebSocket 执行端，承接服务端消息，完成协议解析、状态管理、POI 本地存储与 Nav2 桥接。
3. 机器人本机导航栈
   包括 Lightning SLAM、TF、Nav2、前沿探索节点、Realsense 扫描桥等。
4. 机器人本机 POI 存储
   执行端以本地文件形式保存 POI 的几何信息与会话信息，使其能随 SLAM 回环更新。

可以把职责分成两层：

- 语义层
  服务端只知道“POI_006 叫会议室门口”“去 POI_006”“把当前位置标成 POI_008”。
- 几何执行层
  执行端知道“POI_006 当前在 map 下的姿态是什么”“它对应的 odom 锚点是什么”“SLAM 回环后它需要如何修正”。

这意味着服务端不应该成为地图坐标真值源，否则一旦机器人本地地图因为回环发生修正，服务端存下来的旧坐标就会和现场实际地图脱节。

## 3. 本机启动链路

当前一键启动入口是 [start_navigation_headless.sh](/home/unitree/ros2_ws/src/g1_nav/start_navigation_headless.sh)。

它统一拉起以下进程：

1. `map -> odom` 静态 TF
2. Lightning SLAM
3. `g1_auto_explore.launch.py`
   其中包含 Nav2、可选 Realsense scan bridge、可选 frontier exploration、可选 RViz
4. `navigation_executor.py`

脚本默认行为：

- 默认服务端地址为 `ws://172.16.21.205:8100/ws/navigation/executor`
- 默认启用 frontier exploration
- 默认启用 Realsense depth-to-scan bridge
- 默认不启用 RViz
- 统一在当前 shell 中托管子进程，`Ctrl+C` 可整体退出

脚本在启动时会打印：

- `ROS_DOMAIN_ID`
- RViz 开关状态
- Realsense 开关状态
- frontier exploration 开关状态
- executor 目标 WebSocket URI

脚本还会在启动前做两个重要预检：

- 校验 `python3-yaml`
- 校验 `python3-websockets`

这两个依赖缺失会直接导致执行端无法运行。此前真实遇到的一个故障就是执行端启动后立即退出，根因并不是网络不通，而是 Python 运行环境缺少 `websockets` 模块。

## 4. 执行端内部组件分工

### 4.1 入口进程

[navigation_executor.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor.py) 是执行端主入口，负责：

- 解析命令行参数
- 建立 ROS 2 节点
- 初始化 TF、POI 存储、Nav2 桥接
- 建立 WebSocket 长连接
- 发送心跳
- 监听 SLAM 回环引起的 `map->odom` 变化

它默认连接：

- `ws://172.16.21.205:8100/ws/navigation/executor`

当前已经显式打印连接生命周期日志：

- `connecting to ...`
- `connected`
- `connection failed: ...`
- `connection closed: ...`

这对联调很关键，因为它能区分：

- 进程根本没起来
- 进程起来了但依赖缺失
- 进程起来了但连不上服务端
- 进程已连上但后续被服务端或网络断开

### 4.2 协议状态机

[navigation_executor_core.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor_core.py) 负责协议层状态管理，当前支持的动作为：

- `update_poi_list`
- `navigate_to`
- `abort_navigation`
- `mark_current_poi`

状态机含义：

- `IDLE` 空闲
- `STARTING` 已接收导航请求，正在发起 Nav2 goal
- `NAVIGATING` 导航中
- `ERROR` 最近一次导航失败

关键约束：

- 新的 `navigate_to` 会抢占当前导航任务
- `abort_navigation` 只取消匹配 `request_id` 的活动任务
- `update_poi_list` 只更新语义目录，不应打断导航
- `mark_current_poi` 允许在导航过程中执行，不应取消当前导航

### 4.3 Nav2 桥接层

[nav2_action_bridge.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/nav2_action_bridge.py) 把协议层动作转换成 `/navigate_to_pose` action 调用。

职责包括：

- 等待 Nav2 action server 就绪
- 把本地 POI 的 `map_pose` 转成 `NavigateToPose.Goal`
- 转发 Nav2 feedback 为 `on_progress`
- 在成功、取消、失败时回调协议层

当前桥接层不负责解释业务语义，只负责“把一个几何目标送进 Nav2 并把结果吐回来”。

### 4.4 位姿采集层

[pose_provider.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/pose_provider.py) 负责提供 POI 标记和回环更新所需的几何信息。

它同时维护两类位姿：

- `map` 系下的机器人当前位姿
- `odom` 系下的机器人当前位姿

`mark_current_poi` 之所以要同时保存这两份，不是冗余，而是为了支持 SLAM 回环后的 POI 重投影：

- `map_pose` 用于当前导航与对外回传
- `odom_pose` 用于在 `map->odom` 更新后重新推算新的 `map_pose`

### 4.5 本地 POI 存储

[poi_store.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/poi_store.py) 是执行端本地真值存储，当前保存：

- `poi_id`
- `name`
- `map_pose`
- `odom_pose`
- `slam_session_id`

这表示 POI 不只是一个名字或一个静态坐标，而是一个带有“几何锚点 + 所属 SLAM 会话”的本地对象。

## 5. 服务端与执行端的职责边界

这是本次架构最关键的设计点。

### 5.1 服务端负责什么

服务端应该负责：

- 接收大脑的自然语言或任务编排结果
- 生成语义命令并下发给执行端
- 管理 `request_id`、`sub_id`
- 管理语义 POI 目录
- 向用户反馈导航结果或 POI 标记结果

服务端可以知道：

- `POI_006` 这个语义标识存在
- 它的展示名称是“会议室门口”
- 用户要求去这个点，或新增这个点

### 5.2 服务端不应该负责什么

服务端不应该成为以下信息的权威来源：

- 机器人侧地图中的 POI 几何坐标
- `map->odom` 变化后的 POI 修正结果
- 由本机 TF 和 odometry 推导出的最新 POI 位置

原因很直接：

- 地图真值在机器人本地
- 回环修正在机器人本地发生
- 服务端看不到足够实时且一致的 TF/odom/SLAM 内部状态

如果让服务端持有 POI 几何真值，会出现两个典型问题：

1. 服务端拿到的是旧地图坐标，回环后坐标过时
2. 服务端和机器人对同一个 POI 的理解不一致，导致“能看到 POI，但导航去错地方”

### 5.3 推荐的边界定义

建议将 POI 一分为二：

- 服务端语义目录
  只维护 `id`、`name`、可选业务标签
- 执行端几何目录
  维护本地导航所需的坐标、姿态、锚点、会话信息

这样服务端保持抽象层，大脑也只处理任务语义，不被底层地图漂移耦合。

## 6. 当前协议格式

### 6.1 一个必须明确的事实

当前代码里的实际协议格式，是沿用现有导航链路的 `flat schema`，不是最初示例里那种统一包在 `data` 字段下的结构。

也就是说，当前执行端实际解析的是：

```json
{
  "action": "navigate_to",
  "request_id": "req_001",
  "sub_id": 1,
  "target_id": "POI_001"
}
```

而不是：

```json
{
  "action": "navigate_to",
  "data": {
    "request_id": "req_001"
  }
}
```

合并或服务端联调时，这一点必须先对齐，否则最常见结果就是 WebSocket 已连接，但执行端收到消息后直接解析失败。

### 6.2 服务端 -> 执行端消息

#### 6.2.1 `update_poi_list`

```json
{
  "action": "update_poi_list",
  "version": 3,
  "poi_list": [
    {
      "id": "POI_001",
      "name": "前台"
    },
    {
      "id": "POI_002",
      "name": "会议室门口"
    }
  ]
}
```

当前语义：

- 用于同步服务端维护的语义 POI 目录
- 默认只更新已有 POI 的名称，不覆盖本地几何坐标
- 不应作为地图坐标下发表使用

#### 6.2.2 `navigate_to`

```json
{
  "action": "navigate_to",
  "request_id": "req_nav_001",
  "sub_id": 1,
  "target_id": "POI_002"
}
```

当前语义：

- 服务端只给目标 POI 的语义 ID
- 执行端在本地 POI store 中查找其几何信息
- 找到后转给 Nav2 的 `/navigate_to_pose`

#### 6.2.3 `abort_navigation`

```json
{
  "action": "abort_navigation",
  "request_id": "req_nav_001"
}
```

当前语义：

- 取消匹配 `request_id` 的活动导航任务
- 若当前无活动任务或 `request_id` 不匹配，则忽略

#### 6.2.4 `mark_current_poi`

```json
{
  "action": "mark_current_poi",
  "request_id": "req_poi_001",
  "sub_id": 2,
  "poi": {
    "id": "POI_006",
    "name": "会议室门口"
  }
}
```

当前语义：

- 服务端下发一个语义 POI 定义
- 执行端立刻回 `ack`
- 执行端读取机器人当前 `map_pose` 和 `odom_pose`
- 执行端本地落盘
- 执行端返回成功或失败结果

### 6.3 执行端 -> 服务端消息

#### 6.3.1 `on_progress`

```json
{
  "event_type": "on_progress",
  "request_id": "req_nav_001",
  "sub_id": 1,
  "remaining_distance": 2.347,
  "status": "running"
}
```

#### 6.3.2 `on_arrived`

```json
{
  "event_type": "on_arrived",
  "request_id": "req_nav_001",
  "sub_id": 1
}
```

#### 6.3.3 `on_error`

```json
{
  "event_type": "on_error",
  "request_id": "req_nav_001",
  "sub_id": 1,
  "error_message": "Nav2 rejected the goal"
}
```

#### 6.3.4 `on_mark_poi_ack`

```json
{
  "event_type": "on_mark_poi_ack",
  "request_id": "req_poi_001",
  "sub_id": 2,
  "status": "accepted"
}
```

#### 6.3.5 `on_mark_poi_success`

```json
{
  "event_type": "on_mark_poi_success",
  "request_id": "req_poi_001",
  "sub_id": 2,
  "poi": {
    "id": "POI_006",
    "name": "会议室门口",
    "frame_id": "map",
    "position": {
      "x": 1.2,
      "y": -0.8,
      "z": 0.0
    },
    "yaw": 1.57,
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.707,
      "w": 0.707
    }
  }
}
```

需要注意：

- 当前成功回包比最初草案更丰富
- 它不仅回 `id` 与 `name`，还回了当前 `map` 下的几何姿态

如果服务端坚持“只做抽象层”，这份几何回包可以仅作为日志、展示或调试信息，而不应变成服务端的长期几何真值。

#### 6.3.6 `on_mark_poi_error`

```json
{
  "event_type": "on_mark_poi_error",
  "request_id": "req_poi_001",
  "sub_id": 2,
  "error_message": "当前位置位姿数据过期"
}
```

## 7. POI 标记与回环更新机制

### 7.1 为什么 POI 要同时保存 `map_pose` 和 `odom_pose`

如果只保存 `map_pose`，在 SLAM 后端发生回环后，地图整体可能会发生轻微甚至明显修正，旧的 `map` 坐标就不再可靠。

当前实现的思路是：

1. 标记 POI 时保存当时的 `map_pose`
2. 同时保存当时的 `odom_pose`
3. 持续监听当前 `map->odom`
4. 一旦 `map->odom` 发生变化，用新的 `map_from_odom` 重新把旧 `odom_pose` 投影回 `map`

这使 POI 能跟随 SLAM 后端修正，而不是永远钉死在旧地图坐标。

### 7.2 当前重投影触发点

在 [navigation_executor.py](/home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor.py) 中，执行端会周期性检查 `pose_provider.current_map_from_odom()`。

当发现 `map->odom` 发生变化时：

- 调用 `poi_store.reproject_session(...)`
- 对匹配 `slam_session_id` 的 POI 做重投影
- 若位移或角度超过阈值，则更新本地 POI 文件

当前阈值为：

- 平移阈值 `0.05 m`
- 航向阈值 `0.03 rad`

### 7.3 这对服务端意味着什么

这意味着服务端不需要也不应该在回环后主动“修 POI 坐标”。

服务端只需要：

- 继续持有 POI 的语义标识
- 在导航时引用 POI ID
- 在需要展示时读取执行端反馈或独立获取机器人状态

## 8. 典型时序

### 8.1 启动连接时序

1. 机器人执行 [start_navigation_headless.sh](/home/unitree/ros2_ws/src/g1_nav/start_navigation_headless.sh)
2. 本机拉起 TF、SLAM、Nav2、exploration、executor
3. executor 连接 `/ws/navigation/executor`
4. 服务端向执行端同步 `update_poi_list`

### 8.2 导航时序

1. 服务端下发 `navigate_to`
2. 执行端在本地 POI store 中找到目标几何位置
3. 执行端把目标转成 Nav2 goal
4. Nav2 feedback 被桥接成 `on_progress`
5. 成功时返回 `on_arrived`
6. 失败时返回 `on_error`

### 8.3 POI 标记时序

1. 服务端下发 `mark_current_poi`
2. 执行端返回 `on_mark_poi_ack`
3. 执行端采集当前 `map_pose` 与 `odom_pose`
4. 执行端落盘到本地 POI store
5. 成功则返回 `on_mark_poi_success`
6. 失败则返回 `on_mark_poi_error`

### 8.4 回环修正时序

1. SLAM 后端发生回环
2. `map->odom` 更新
3. executor 轮询检测到变更
4. 本地 POI store 按 `odom_pose` 重投影
5. 本地 POI 几何信息被更新

## 9. 合并时需要重点核对的协议风险

以下几项是最容易在合并时引入隐性故障的点。

### 9.1 消息结构是否仍为 flat schema

当前代码要求字段在顶层：

- `action`
- `request_id`
- `sub_id`
- `target_id`
- `poi`

如果服务端改回 `data` 包裹式结构，执行端会解析失败。

### 9.2 `sub_id` 是否持续保留

当前执行端在：

- `on_progress`
- `on_arrived`
- `on_error`
- `on_mark_poi_ack`
- `on_mark_poi_success`
- `on_mark_poi_error`

都会携带 `sub_id`。服务端如果在新版本中省略、忽略或改名，容易造成多步骤任务流无法正确关联。

### 9.3 `update_poi_list` 是否被误当成几何同步

当前实现中，`update_poi_list` 只做语义目录更新。如果服务端在这个消息里下发坐标，并期望执行端全量覆盖本地几何，当前代码不会按这个逻辑工作。

### 9.4 服务端是否错误地把 `on_mark_poi_success` 当成几何真值回写源

当前成功回包里包含几何信息，但它更适合：

- 调试
- 观测
- UI 展示

不建议服务端把这份几何长期保存为自己的全局真值，否则又会把语义层和几何层耦合回去。

### 9.5 `mark_current_poi` 是否允许在移动中执行

当前实现明确支持边走边标记。如果服务端或上层业务把它限制为“必须静止时才能发”，就和当前执行端能力不一致。

## 10. 排障定位建议

### 10.1 WebSocket 连不上

先看执行端日志：

- 是否打印 `connecting to ...`
- 是否打印 `connected`
- 是否打印 `connection failed: ...`

若连不上，优先排查：

- 服务端地址是否正确
- 8100 端口是否监听
- 机器人到服务端网络是否可达
- 服务端路径是否为 `/ws/navigation/executor`

### 10.2 执行端进程秒退

优先排查：

- `python3-websockets` 是否安装
- ROS 环境是否已 `source`
- `ros2 run g1_nav navigation_executor.py --help` 是否可执行

历史上已经出现过“看起来像网络问题，实际是 Python 模块缺失”的案例。

### 10.3 已连接但收到命令无反应

优先排查协议字段是否匹配当前实现：

- 顶层是否有 `action`
- `navigate_to` 是否带 `request_id`、`sub_id`、`target_id`
- `mark_current_poi` 是否带顶层 `poi`
- 是否错误使用了 `data` 包裹格式

### 10.4 `navigate_to` 收到后不动

分层排查：

1. 协议层
   是否真的收到了 `navigate_to`
2. POI 层
   本地 POI store 中是否存在对应 `target_id`
3. Nav2 层
   `/navigate_to_pose` action server 是否 ready
4. 控制层
   Nav2 是否在出 `/cmd_vel_nav`
5. 安全层
   `collision_monitor` 是否把速度清零
6. 本体层
   `g1_move` 是否真的将安全速度发到底盘

### 10.5 `mark_current_poi` 失败

优先排查：

- 当前是否有有效 `map->base_link` TF
- `/lightning/odometry` 是否持续更新
- 位姿时间戳是否过期

当前实现里，若 `map_pose` 时间戳相对当前时间超过阈值，会返回：

- `当前位置位姿数据过期`

### 10.6 回环后 POI 漂移或跳变

优先排查：

- 当前是否真的发生了 `map->odom` 更新
- 本地 POI 是否保存了 `odom_pose`
- `slam_session_id` 是否一致
- 重投影阈值是否过大或过小

如果服务端同时也在“修坐标”，则要先排除双写冲突。

## 11. 当前实现的优点与已知约束

### 11.1 优点

- 服务端保持抽象层，避免与 SLAM 几何强耦合
- 执行端直接掌握 TF、odom、Nav2，上下文完整
- 支持边走边做视觉大模型标记
- 支持 SLAM 回环后的本地 POI 修正
- 启动脚本统一托管所有关键进程，便于现场使用

### 11.2 已知约束

- 当前 `update_poi_list` 更偏“重命名/语义同步”，不是完整 POI 双向同步协议
- 当前 `slam_session_id` 默认仍是固定值 `live_session`，后续如果需要多地图或多次建图会话隔离，还需要继续细化
- 当前 WebSocket 长连接侧已经有心跳，但服务端超时与重连策略仍需联调确认
- 当前成功回包包含几何信息，服务端需要明确只把它当调试信息还是展示信息，避免语义层重新几何化

## 12. 合并建议

建议在合并这套方案时，把以下三点作为强制检查项：

1. 协议兼容性检查
   服务端发送与接收的 JSON 字段必须和当前 `navigation_protocol.py` 保持一致，特别是 `flat schema`、`sub_id`、`poi` 结构。
2. 边界检查
   服务端只维护语义目录，不接管执行端几何真值。
3. 回环一致性检查
   本地 POI 更新逻辑必须继续保留 `odom_pose + map->odom` 重投影机制，不能在后续重构里退化成只保存静态 `map_pose`。

如果后续要继续演进协议，推荐方向是：

- 明确区分“语义目录同步协议”和“调试观测协议”
- 给 `update_poi_list` 增加版本冲突与增删改语义定义
- 给回环重投影增加可选上报事件，便于服务端做观测而不是做真值维护

## 13. 结论

当前这套架构的本质，不是把服务端做成地图数据库，而是把它保持为“语义调度与任务反馈中心”。

真正与地图、SLAM、TF、Nav2 强相关的几何执行逻辑，都应尽量留在机器人本机执行端。这样才能同时满足：

- POI 标记时的实时性
- 边走边标记的能力
- SLAM 回环后的局部真值修正
- 服务端业务层的稳定抽象边界

对于后续合并与联调，最值得优先核对的并不是算法细节，而是：

- 服务端是否使用了当前实际协议格式
- 是否仍然遵守“服务端语义、执行端几何”的边界
- 是否保留了回环后的本地 POI 重投影机制

这三点一旦对齐，后续问题定位会清晰很多。
