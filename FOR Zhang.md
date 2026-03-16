# g1_nav，写给 Zhang

`g1_nav` 这个项目最容易被误解的地方是：名字里虽然有 “nav”，但它真正做的不是某一个“导航算法”，而是把一群彼此并不完全兼容的模块接成一条能跑的导航链。

如果把机器人系统想成一支乐队：

- Lightning SLAM 负责“我在哪、地图长什么样”
- Nav2 负责“下一步往哪走”
- `g1_cmd/g1_move` 负责“把速度真的执行到机器人身上”
- `g1_nav` 则更像指挥和编曲师

它最值钱的地方，不是某个单点算法有多炫，而是它知道：

- 哪些 topic 要喂给 Nav2
- 哪些 TF / odom 需要补齐
- 哪些消息虽然“存在”，但 QoS 和 frame 不对，实际上根本不能用
- 自主探索应该在什么时候启动，才不会和 Nav2 bringup 打架

## 1. 这个项目在做什么

一句话版本：

`g1_nav` 把外部 SLAM、Nav2、自主探索和机器人执行链路粘在一起，让 G1 从“有地图”变成“会自己导航、会自己找新区域”。

更具体一点：

1. 它启动 Nav2。
2. 它把 3D 点云转换成 Nav2 更稳定的 `/scan`。
3. 它把外部位姿整理成标准 `/odom`。
4. 它给 volatile 的 SLAM 地图加一层缓存桥，稳定转发到 `/map`。
5. 它支持 frontier exploration，而且这次已经演进出了 C++ 版 explorer，默认更轻量。
6. 它还能带 RViz，把地图、costmap、机器人姿态和探索标记一起显示出来。

## 2. 技术架构，用“城市导航系统”理解

可以把整套系统理解成一座城市的自动导航中枢：

- `lightning run_slam_online`
  像测绘局，负责生成地图和位姿。
- `tf2_ros static_transform_publisher map odom`
  像坐标翻译公证处，告诉系统 map 和 odom 的关系。
- `livox_ros_driver2`
  像传感器入口，负责把 Livox 数据送进来。
- `pointcloud_to_laserscan`
  像翻译官，把 3D 点云翻成 Nav2 最熟的 `LaserScan`。
- `map_cache_bridge.py`
  像缓存服务器，把 Lightning 的 volatile 栅格地图桥接成稳定的 `/map`。
- `tf_to_odom.py`
  像协议适配器，把外部 odometry/TF 整理成 Nav2 需要的 `/odom` 与 `odom -> base`。
- `Nav2`
  像市政调度中心，里面有 planner、controller、costmap、behavior tree。
- `frontier_explorer`
  像巡逻队长，不等人手动点目标，自己挑下一块值得探索的未知边界。
- `g1_move`
  像执行队，把 `cmd_vel` 真正下发到机器人。

所以这不是一个“算法仓库”，而是一条完整流水线：

`点云 / 位姿 -> 地图 / TF / odom -> Nav2 -> cmd_vel -> G1运动`

## 3. 代码库结构

### `launch/`

这是系统的“总导演区”。

- `launch/g1_auto_explore.launch.py`
  你最关心的入口。它会在 2D Nav2 bringup 之上打开自主探索，并支持选择 C++ / Python explorer。
- `launch/2d_g1_nav2_bringup.launch.py`
  现在是最关键的一层。外部 SLAM 由你单独启动，这个 launch 只负责接上 Nav2、桥接节点、explorer 和可视化。
- `launch/g1_nav2_bringup.launch.py`
  是仓库里保留的另一套入口，偏向历史上的“一揽子 bringup”。

### `config/`

这是系统的“性格配置区”。

- `config/nav2_params_2d.yaml`
  决定 planner、controller、costmap、BT 插件、速度平滑等行为。
- `config/frontier_explorer_params.yaml`
  决定 frontier explorer 的地图来源、TF frame、黑名单、进度超时、GVD 可视化等。

### `g1_nav/`

这是 Python 功能节点区。

- `g1_nav/tf_to_odom.py`
  位姿桥接节点。
- `g1_nav/map_cache_bridge.py`
  地图缓存桥。
- `g1_nav/nav2_precheck.py`
  启动前诊断工具。
- `g1_nav/frontier_explorer.py`
  老的 Python explorer，实现还在，主要作为回退方案。

### `src/`

这里是新的 C++ explorer。

- `src/frontier_explorer.cpp`
  这次 `git pull` 后最重要的新增内容。默认 explorer 已切到 C++，更轻、更适合真机长期跑。

## 4. 这次版本更新，核心变化是什么

这次更新里最重要的一条，不是 launch，而是：

`frontier_explorer` 从 Python 主实现变成了 C++ 主实现。

这背后的技术决策非常合理：

- Python 写 frontier 原型很快，调试方便
- 真机长期运行时，C++ 版更稳，性能和资源占用更可控
- 保留 Python 版作为 fallback，也方便对照行为是否回归

所以当前仓库的思路其实很成熟：

- 默认用 `cpp`
- 仍保留 `python`
- 通过 `explorer_impl` 参数选择

这不是“推倒重来”，而是典型的工程演进路线：

先用 Python 验证思路，再把稳定的核心迁成 C++。

## 5. 这次我做的移植，为什么是这些改动

你已经明确了真实运行方式：

```bash
ros2 run lightning run_slam_online --config ./config/default_livox.yaml
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

这句话其实把边界划得很清楚了：

- SLAM 由外部起
- `map -> odom` 由外部提供
- Livox 驱动由外部起
- `g1_nav` 只负责“吃现成的 SLAM 结果并导航”

所以这次移植里，我刻意只在原来的 `g1_auto_explore -> 2d_g1_nav2_bringup` 链上补东西，没有再把职责扩散出去。

### 5.1 加 `map_cache_bridge.py`

这是因为 Lightning 的地图话题是 volatile 风格，Nav2 / RViz 这种“后启动的消费者”很容易错过第一帧地图。

于是现在加了一层桥：

- 订阅 `/lightning/grid_map`
- 转发到 `/map`
- 按 `transient_local` 方式缓存最后一张图
- 同时落盘，防止重启后地图一时半会儿收不到
- 发布时把 frame 改成 `map`，配合你外部起的 `map -> odom` 静态 TF

这层桥非常典型，属于机器人系统里“别抱怨上游不完美，先把边界适配平”的工程思路。

### 5.2 升级 `tf_to_odom.py`

外部 odometry 有一个真机里特别常见的问题：

“不是没消息，而是不连续，不一定稳定按固定频率发。”

老版本只会从 TF 查 `odom -> base`，或者依赖持续不断的源数据。这样一旦上游 odometry 不是持续流，Nav2 就会觉得 `/odom` 断了。

现在这个桥做了三件事：

- 优先吃 `/lightning/odometry`
- 发布标准 `/odom`
- 即使源 odometry 暂时没刷新，也会持续重发最后一帧，保证 Nav2 不因为静止时“没新消息”而失联

这就是优秀工程师会特别在意的点：

系统静止时也要稳定，不是只有机器人在动的时候才“看起来正常”。

### 5.3 保留原来的 2D bringup 结构，只补 RViz 和 robot model

你说“不能这么改”，这个判断是对的。

在机器人项目里，大改入口层最容易引入你暂时看不见的副作用：

- 新旧 launch 的参数来源不一致
- explorer 启动时序变了
- 其他人已有脚本失效

所以这次处理方式是：

- `g1_auto_explore.launch.py` 继续直接套 `2d_g1_nav2_bringup.launch.py`
- 只给它多加 `use_rviz` / `rviz_config` / `publish_robot_state`
- 不改变你外部起 SLAM 的职责划分

这就是“在原来的基础上改”，而不是“换一套我觉得更整洁的新结构”。

## 6. 这套系统现在是怎么连起来的

你外部先起：

```bash
ros2 run lightning run_slam_online --config ./config/default_livox.yaml
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

然后再起：

```bash
ros2 launch g1_nav g1_auto_explore.launch.py
```

此时链路应该是：

1. Livox 驱动给出点云
2. `pointcloud_to_laserscan` 转成 `/scan`
3. Lightning 给 `/lightning/grid_map`
4. `map_cache_bridge` 转成带缓存的 `/map`
5. Lightning 给 `/lightning/odometry`
6. `tf_to_odom` 转成稳定 `/odom`
7. 外部静态 TF 给出 `map -> odom`
8. Nav2 使用 `/map`、`/odom`、`/scan`
9. C++ frontier explorer 发布探索目标
10. RViz 显示地图、costmap、机器人模型、frontier 和 GVD 标记

## 7. 你可以从这次改动学到什么

### 经验 1：机器人里最重要的常常不是算法，而是边界适配

算法论文看起来最耀眼，但真机项目里，最容易决定成败的通常是：

- topic 名对不对
- QoS 对不对
- frame 对不对
- 启动顺序对不对
- 静止时消息还稳不稳

### 经验 2：外部组件已经有明确职责时，不要随便“重新设计架构”

这是这次你提醒得非常对的一点。

当你已经确定：

- SLAM 外部起
- static TF 外部起
- Livox 驱动外部起

那 `g1_nav` 就应该乖乖只做导航适配。

优秀工程师不是“总想重构成更优雅的结构”，而是会尊重真实运行方式，优先保证系统边界稳定。

### 经验 3：C++ 迁移不是为了“高级”，而是为了长期运行

这次 frontier 切 C++，非常值得你记住：

- Python 适合快速验证
- C++ 适合稳定落地
- 最成熟的路线不是二选一，而是保留 fallback、逐步切主实现

这就是一个很典型的工程升级姿势。

## 8. 潜在坑点

### 坑 1：外部 `map -> odom` 没起

如果你忘了起：

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

那 Nav2 很容易表现成：

- `/map` 有
- `/odom` 有
- costmap 就是不更新
- RViz 里地图和机器人像分属两个宇宙

### 坑 2：SLAM 已经先起了，Nav2 后起，结果没收到第一帧地图

这正是 `map_cache_bridge` 存在的原因。

如果没有这层桥，volatile map 非常容易造成：

“topic list 里有地图话题，但 Nav2 / RViz 看起来像没地图。”

### 坑 3：机器人静止时 `/lightning/odometry` 不连续

如果桥接节点只会“有新消息才发”，你会得到一种很迷惑的系统：

- 机器人一动，一切正常
- 机器人一停，Nav2 报 TF/odom stale

所以这次 `tf_to_odom.py` 的持续重发是非常关键的稳态修复。

## 9. 最值得你继续读的文件

- `launch/2d_g1_nav2_bringup.launch.py`
  这是当前最核心的系统入口。
- `src/frontier_explorer.cpp`
  这是这次 C++ 迁移的主角。
- `g1_nav/map_cache_bridge.py`
  这是很典型的“工程适配层”。
- `g1_nav/tf_to_odom.py`
  这是另一个非常实战的桥接文件。
- `config/nav2_params_2d.yaml`
  这份文件决定了 Nav2 的真实行为。

## 最后一句

`g1_nav` 最像一个经验丰富的系统集成人员。

它不负责发明 SLAM，也不负责发明 Nav2，但它知道怎样把两边接到同一张地图、同一条 TF 树、同一套启动顺序里。真正能把机器人从 “demo 能跑” 变成 “现场能用” 的，通常就是这种项目。
