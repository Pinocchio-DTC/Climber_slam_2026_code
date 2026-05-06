# 开源行为树决策运行逻辑（COD_Behavior）

> 目标：用“运行时视角”解释这个开源决策是怎么工作的，方便你快速定位可改点与可复用点。

---

## 1. 总体架构（一句话）

这个工程是 **BehaviorTree.CPP + ROS2 + Nav2**：
- 用 `cod_behavior.cpp` 启动行为树循环；
- 用自定义 BT 节点读取串口/裁判系统数据写入黑板；
- 树里根据条件切换“回家补血 / 巡逻 / 占点”等行为；
- 动作节点通过 `goal_pose` 话题或 Nav2 Action 下发导航任务。

---

## 2. 启动后发生了什么（主流程）

主入口：`COD_Behavior/src/cod_behavior.cpp`

启动顺序可理解为 8 步：

1. `rclcpp::init` 初始化 ROS2。
2. 创建全局节点 `cod_behavior`。
3. 配置多个 `RosNodeParams`（分别对应：
   - `navigate_to_pose`
   - `follow_waypoints`
   - `navigate_through_poses`
   - `goal_pose` 话题发布）
4. 在 `BehaviorTreeFactory` 注册所有自定义节点（条件节点、动作节点、黑板写入节点等）。
5. 读取参数 `cod_bt_path`，加载 XML 行为树。
6. 初始化黑板：站位点、巡逻点、血量、状态布尔量、航点索引等。
7. 启动 Groot2Publisher（可视化调试）。
8. 进入循环：
   - `rclcpp::spin_some(global_node_)`
   - `tree.tickOnce()`
   - `sleep(10ms)`

> 这意味着：**ROS 回调和行为树 tick 在同一主循环里被持续驱动**。

---

## 3. 运行时数据从哪里来（黑板输入链路）

关键节点：`WriteToBlackboard`（在 `include/cod_behavior/action.h`）

它做的事：
- 订阅 `/SerialReceiveData`（消息类型 `rm_interfaces::msg::SerialReceiveData`）。
- 从消息里抽取：
  - `hp`
  - `zone_status`
  - `is_defence`
  - `is_attack`
  - `self_status`
  - `is_recover`
- 在 `tick()` 时写入行为树黑板输出端口。

行为树中的条件节点（在 `include/cod_behavior/condition.h`）再读取这些黑板变量判断分支：
- `HpCondition`：血量阈值判断（示例中是 `<210`）
- `ZonePatrolCondition`：增益区状态判断
- `DefencePatrolConditioin` / `AttackPatrolCondition`：姿态标志判断
- 其他组合条件

---

## 4. 行为树是怎么决策的（以 singlenav_tree.xml 为例）

参考：`COD_Behavior/cod_bt/singlenav_tree.xml`

顶层结构可以简化为：

1. 先发一次 `main_position` 作为初始目标。
2. `KeepRunningUntilFailure` 持续运行一个 `ReactiveFallback`。
3. `ReactiveFallback` 从上到下尝试分支（谁先 SUCCESS/RUNNING 就执行谁）：
   - 分支A：更新黑板（`WriteToBlackboard`）
   - 分支B：低血量则回家并等待恢复（`HpCondition + StayHome`）
   - 分支C：满足占区巡逻条件时，加载巡逻 CSV 并循环航点
   - 分支D：默认预加载巡逻（兜底）
   - 分支E：`AlwaysSuccess` 防止整棵树失败退出

你可以把它理解成“高优先级行为抢占低优先级行为”的反应式树。

---

## 5. 导航动作是怎么发出去的

动作实现主要在 `include/cod_behavior/action.h`：

- `PubNav2Goal`
  - 从黑板读取 `goal_pose`
  - 发布到 `goal_pose` 话题
  - 适合单点实时目标

- `SendNav2Goal`（Action）
  - 对接 `navigate_to_pose`
  - 支持 result/feedback/failure 回调

- `FollowWaypointsAction`（Action）
  - 从 CSV 加载航点
  - 调用 `follow_waypoints`

- `NavigateThroughPosesAction`（Action）
  - 从 CSV 加载航点
  - 调用 `navigate_through_poses`（穿越式，不一定每点停留）

- 巡逻辅助节点：
  - `LoadWaypoints`
  - `GetCurrentWaypoint`
  - `WaitUntilReached`（TF 判断到点）
  - `WaitDuration`
  - `NextWaypoint`

---

## 6. 一次典型运行周期（按时间线）

1. 主循环 tick。
2. `WriteToBlackboard` 刷新最新裁判/串口状态到黑板。
3. 树先检查“是否低血量”。
4. 若低血量，切到 `home_position` 并保持恢复逻辑。
5. 否则检查“是否满足巡逻/占区条件”。
6. 若满足则进入 CSV 航点循环：取当前点 -> 发目标 -> 等到达 -> 等待 -> 下一个点。
7. 若都不满足，执行兜底分支，保证树继续运行。
8. 下一个 tick 再次全量重评估（Reactive）。

---

## 7. 这个开源方案的特点（优点/注意点）

### 优点
- 决策结构可视化（XML + Groot），比纯 `if/else` 更易调参与沟通。
- 高优先级行为天然可抢占。
- 导航动作节点封装完整，可复用性高。

### 注意点
- XML 内有硬编码绝对路径（如 `/home/.../patrol.csv`），移植时必须参数化。
- 黑板变量命名与消息字段耦合较深，接口变化会连锁影响条件节点。
- 部分阈值写死在条件节点里（如 `HpCondition`），最好改为参数。

---

## 8. 你在读代码时建议按这个顺序

1. `src/cod_behavior.cpp`：先看节点注册与主循环。
2. `cod_bt/*.xml`：看分支优先级和执行图。
3. `include/cod_behavior/condition.h`：看“为什么走某分支”。
4. `include/cod_behavior/action.h`：看“分支具体做了什么”。
5. `launch/*.py + launch/*.yaml`：确认参数与树文件路径。

---

## 9. 速记版（30 秒回忆）

- **输入**：`/SerialReceiveData` -> `WriteToBlackboard`。
- **决策**：ReactiveFallback 按优先级抢占执行。
- **执行**：发 `goal_pose` 或 Nav2 Action。
- **循环**：10ms tick，一直重评估。
- **调试**：Groot2 可视化树状态。

---

如果你愿意，我可以下一步再给你整理一份“战术树各分支的行为映射表”，用于后续继续拆分和优化。
