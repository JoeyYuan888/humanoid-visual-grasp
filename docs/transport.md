# Box Transport Stage

运输阶段负责双手搬运箱子。当前默认使用 FoundationPose + CAD 模型识别箱子 6D pose，并输出左右抓取点；原 FastSAM/颜色方案保留为 legacy 回退。

## 阶段边界

```text
input : 头部相机 RGB/depth、camera_info、箱子 CAD 模型
logic : FoundationPose 6D pose、CAD 盒口投影、左右短边中心抓取点
output: 相机系左右抓取点，再转换为 BASE
```

不和塑料袋单手抓取流程混写：

```text
apps/grasp/      塑料袋抓取
apps/transport/ 蓝色盒子识别、双手搬运
```

## 当前默认识别方案

```text
FoundationPose RGB-D 位姿估计
-> CAD 盒口模型投影
-> 左右短边中心作为双手抓取点
-> 输出 camera frame JSON
-> lock_box_grasp_target.py 使用 CAM2HEAD + TF 转 BASE
```

代码位置：

```text
apps/transport/run_foundationpose_box_grasp_point.py
third_party/foundationpose_crate/
```

旧方案备份：

```text
FastSAM 分割候选盒体
-> 蓝色候选筛选
-> depth-rim + side-mid 几何
-> 红线拟合盒子前沿
-> 绿线用后沿多数边界鲁棒拟合，忽略盒内物体局部凸起
-> 左右侧边中点作为双手抓取点
-> 抓取点如落到 mask 外，自动回退到最近 mask 内有效侧边点
```

## 运行入口

默认 FoundationPose：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --execute
```

默认主流程会继续到箱子两侧靠近点：

```text
低头识别
-> 最近 5 帧 valid left/right 抓取点分别取平均
-> 相机系左右抓取点转 BASE
-> 设置 MPC running mode
-> 到 transport_pregrasp_dual
-> 按往身体 17cm、左右各向外 10cm、z=抓取点+0.38m 生成外扩等待点
-> 设置 MPC running mode
-> 到外扩等待点
-> 开启导纳
-> 用 /wa/points_seq_tracking_with_admittance 单段夹紧到 clamp_offset
-> 关闭导纳
```

默认靠箱参数：

```text
outside_offset = 0.10 m
clamp_offset   = 0.03 m
clamp_offsets  = ""      # 默认不用分段；仅调试时手动传入
clamp_control  = admittance
body_offset    = -0.17 m   # BASE x 负方向，往身体 17cm；相比上一版向身体反方向外移 2cm
side_z_offset  = 0.38 m    # 当前夹紧点上移 38cm；相比上一版下调 2cm
motion_duration = 5.0 s
side_motion_duration = 10.0 s   # 预抓取到两侧靠近点降速 50%
carry_lift = 0.15 m             # 夹紧/手指补夹后抬高 15cm
carry_waist_joints = [-0.3, 0.3, 0.0, 0.0]  # 回收后设置 3-6 四个身体关节
carry_pullback = -0.20 m        # 抬高后 BASE x 负方向回收 20cm
```

FoundationPose 抓取点高度策略：

```text
先对最近 5 帧 valid left/right 相机系抓取点分别求平均
-> selection_policy = average_recent_valid_frames

只要 left/right 都 valid
-> 两侧 z_mm 统一为两侧 z_mm 的平均值
-> height_source = average_valid_sides
```

这样可以降低单帧 FoundationPose/深度抖动；即使无遮挡时左右深度略有差异，后续双手目标高度也保持一致。

### 搬运到位后的放箱/回收

当前保存的放箱点：

```text
data/poses/transport/transport_place_dual.json
```

该文件是实际放置/夹紧点，已经从最新识别到的箱子两侧目标中去掉左右外扩偏移：

```text
left  y = left_grasp_y
right y = right_grasp_y
removed_outside_offset = 0.10 m
```

放箱并回收双臂：

```bash
python apps/transport/run_transport_place_return.py \
  --ws-url ws://192.168.20.102:9091 \
  --execute
```

执行顺序：

```text
当前搬运姿态
-> transport_place_dual 实际放置点
-> 双手复位/松开
-> transport_box_side_approach_latest 外扩退开点
-> transport_pregrasp_dual 中间点
-> transport_home_dual home 点
```

不要把 `transport_place_dual.json` 再当外扩点使用；外扩退开点仍然使用 `data/runtime/transport_box_side_approach_latest.json`。

当前箱子搬运手型：

```text
left_hand_q  = [0.2, 0.9, 0.35, 0.45, 0.56, 0.65]
right_hand_q = [0.2, 0.9, 0.35, 0.45, 0.56, 0.65]
```

当前 `run_transport_flow.py` 默认已自动执行手指补夹、抬高 15cm、往身体回收 20cm、调整腰部搬运姿态。调试时可用 `--skip-hand-adjust`、`--skip-carry-lift`、`--skip-carry-pullback` 单独跳过。

默认放置步骤位置：

```text
2026-08-20 当前测试箱子所在位置，记录为后续默认放置步骤的放置点。
导航目标名: transport_place_area
map pose:
position = [-9.41538026566515, 3.786190846269941, -0.0018881712990937022]
orientation = [-0.002042785013786434, -0.002546765999154717, -0.6442150921871242, 0.7648374049500116]

注意：
place_area / shelf_place_area 是塑料袋放置阶段的地图点。
transport_place_area 是箱子搬运阶段的默认放置地图点。

搬运导航点：
start_goal = transport_start_area     # 占位，尚未测量，不可直接运行
end_goal   = transport_place_area     # 已测量，当前默认终点

相关文件：
configs/navigation.yaml
configs/transport.yaml
data/runtime/transport_box_grasp_target_latest.json
data/runtime/transport_box_side_approach_latest.json
data/runtime/transport_box_clamp_latest.json
data/runtime/transport_box_lift_10cm_latest.json
```

如果需要回退普通 points 夹紧：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --clamp-control points \
  --execute
```

## 厂商 Demo 结论

厂商给的搬运 demo 没有使用 `/wa/points_seq_collaborative_tracking`。它实际使用：

```text
/wa/points_seq_tracking
/wa/points_seq_tracking_with_admittance
/wa/points_seq_tracking_with_joints
/zj_humanoid/upperlimb/movej/whole_body
```

因此厂商 demo 的搬运思路不是“严格双臂协同接口”，而是：

```text
普通任务空间轨迹
+ 导纳模式接触/夹紧/拉出/推入
+ joints 参考姿态引导腰身
+ 初始化姿态使用 whole_body movej
```

当前项目原则仍然是：业务自动流程优先 MPC，不恢复 SDK 手臂路线。厂商 demo 里的 `/zj_humanoid/upperlimb/movej/whole_body` 只能作为参考，不直接迁入主流程。

### 1. 普通移动

厂商普通移动调用：

```text
/wa/points_seq_tracking
```

demo 内部对左右手目标都做了：

```text
target_pose.position.z += 0.35
```

这与本项目实测 `z offset = 0.35m` 一致。注意后续代码不能重复补偿。

适合使用普通 `PointsSeqTracking` 的阶段：

```text
home -> transport_pregrasp_dual
transport_pregrasp_dual -> 外扩等待点
短距离抬起/后拉测试
原路返回
```

不适合直接依赖普通 `PointsSeqTracking` 的阶段：

```text
夹紧后长距离搬起
夹紧后长距离拉回身体
导航中保持箱子
任何要求严格保持左右手相对位姿的动作
```

### 2. 导纳接触

厂商接触类动作调用：

```text
/wa/admittance_mode_setting
/wa/points_seq_tracking_with_admittance
```

典型使用位置：

```text
往里收夹紧箱子
机械臂下沉接触
向外抽离箱子
撤回身体旁边
推箱子进货架
```

推荐测试顺序：

```text
1. 确认服务存在。
2. 在当前外扩/夹紧点附近做 1cm 级别小动作。
3. 开导纳 -> 发送小位移 -> 观察接触是否柔顺 -> 关导纳。
4. 稳定后再用于拉出/推入。
```

现场确认命令：

```bash
rosservice list | grep -E "admittance|points_seq_tracking_with_admittance"
```

如果当前机器人没有导纳服务，运输阶段继续使用普通 `PointsSeqTracking` 分段小步调试，不做大位移搬运。

### 3. 身体姿态约束

厂商撤回/恢复身体姿态时调用：

```text
/wa/points_seq_tracking_with_joints
```

并传入 `reference_joints`：

```text
joint_num = 23
states = reference_joints
```

用途：

```text
避免只满足末端 pose 时腰部前倾/后仰
让撤回身体附近、恢复 home、收手等动作有身体姿态参考
```

本项目后续运输阶段需要记录并维护：

```text
data/poses/transport/transport_home_dual.json
data/poses/transport/transport_pregrasp_dual.json
data/poses/transport/transport_pullback_ref_dual.json
```

其中需要包含：

```text
left/right pose
mpc_state
joint_num=23
```

### 4. 抓取姿态重建

厂商 demo 不直接相信视觉返回的抓取点四元数，而是用左右抓取点连线重建箱子 yaw，再设置固定 pitch：

```text
dx = left.x - right.x
dy = left.y - right.y
box_yaw = atan2(-dx, dy)
orientation = yaw(box_yaw) + fixed_pitch
```

demo 使用的固定 pitch：

```text
pitch = -49.8 deg
```

本项目后续推荐：

```text
1. FoundationPose 负责输出左右抓取点。
2. 用左右抓取点连线计算 box_yaw。
3. 使用现场调好的固定 pitch。
4. 左右手使用同一个重建 orientation。
```

这样比直接使用模型输出姿态更稳定，尤其适合箱子有轻微偏角时。

### 5. Planar Offset

厂商 demo 使用水平面 offset，避免 TCP pitch 把“前后移动”错误投到 z 方向：

```text
dx: 沿箱子法线前后
dy: 沿箱子切线左右
dz: 全局 Z 上下
```

推荐替换当前简单 BASE x/y/z offset 的运输动作：

```text
外扩
靠近
下沉
夹紧
拉回
推入
撤手
```

实现原则：

```text
只使用 yaw 计算 XY 平面偏移
Z 永远单独作为全局高度控制
```

## 推荐控制路线

当前运输阶段建议按以下路线推进：

```text
1. FoundationPose 识别箱子左右抓取点。
2. 相机系抓取点通过 CAM2HEAD + TF 转 BASE。
3. 用左右点连线重建 box_yaw 和左右手 orientation。
4. 使用 planar offset 生成：
   - 预抓取点
   - 外扩等待点
   - 导纳夹紧点
   - 抬起/后拉试探点
5. 普通无接触运动使用 /wa/points_seq_tracking。
6. 接触、夹紧、拉出、推入使用 /wa/points_seq_tracking_with_admittance。
7. 撤回、home、腰身恢复使用 /wa/points_seq_tracking_with_joints。
8. 如果厂商开放 /wa/points_seq_collaborative_tracking，再把夹紧后的长距离搬运切到协同接口。
```

当前不直接使用 collaborative 的原因：

```text
reference 文档写明 collaborative 适用于双臂搬运并保持相对位姿，
但现场 rosservice list 暂未看到 /wa/points_seq_collaborative_tracking。
```

## 导纳安装后的融合流程

导纳默认安装完成后，运输阶段采用“我们的识别 + 厂商控制结构”的融合方案：

```text
FoundationPose 负责箱子抓取点
厂商 demo 的 points/admittance/with_joints 结构负责控制
```

核心原则：

```text
1. 自由空间运动使用 /wa/points_seq_tracking。
2. 接触、夹紧、拉出、推入使用 /wa/points_seq_tracking_with_admittance。
3. 腰身恢复、撤手、home 使用 /wa/points_seq_tracking_with_joints。
4. 不恢复 SDK 手臂路线。
5. 手掌参数不直接照搬厂商 demo，继续使用现场实测参数。
```

### 阶段 A：识别和锁存

```text
1. 设置 MPC running mode true。
2. MPC neck 低头。
3. FoundationPose 识别箱子左右抓取点。
4. camera -> HEAD -> BASE。
5. 保存 BASE 左右抓取点。
6. MPC neck 抬头。
```

### 阶段 B：姿态和路径生成

```text
1. 用左右抓取点连线计算 box_yaw。
2. 使用现场固定 pitch 生成左右手统一 orientation。
3. 使用 planar offset 生成：
   - pregrasp
   - outside 15cm
   - clamp 8cm / 4cm / 3cm / 2cm
   - lift
   - pull-back
```

姿态生成逻辑：

```text
dx = left.x - right.x
dy = left.y - right.y
box_yaw = atan2(-dx, dy)
orientation = yaw(box_yaw) + fixed_pitch
```

offset 逻辑：

```text
dx: 沿箱子法线前后
dy: 沿箱子切线左右
dz: 全局 Z 上下
```

### 阶段 C：靠近

```text
1. with_joints 到 transport_pregrasp_dual。
2. points_seq_tracking 到 outside 15cm。
```

### 阶段 D：接触夹紧

```text
1. 开启导纳。
2. 导纳从 outside_offset 单段收到 clamp_offset。
3. 关闭导纳。
```

### 阶段 E：搬起和拉回

```text
1. 普通 points 小幅抬高 3-5cm。
2. 开启导纳。
3. 导纳往身体方向拉回，初始从小位移开始测。
4. 关闭导纳。
```

拉回距离不要直接照搬厂商 demo 的 45cm。当前项目需要现场重新测：

```text
先 5cm
再 10cm
再 20cm
最后根据箱子位置决定是否继续增加
```

### 阶段 F：导航准备

```text
1. with_joints 到 transport carry reference。
2. 确认腰身恢复到可导航姿态。
3. 进入 navigation 阶段。
```

## 导纳最小验证步骤

导纳服务装好后，先做最小闭环验证，不直接跑完整搬运。当前代码默认夹紧使用导纳，业务流程默认单段收到 `clamp_offset`。如需安全调试，可临时传 `--clamp-offsets` 做 1cm 小步。

```text
1. 机器人到 outside 15cm 点。
2. 开启导纳。
3. 从 15cm 收到 14cm。
4. 观察末端是否柔顺，有无明显硬顶。
5. 关闭导纳。
6. 原路返回 outside 15cm 或 transport_pregrasp_dual。
```

如果机器人已经停在外扩等待点，可以只执行夹紧段，不重新识别、不回预抓取：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --clamp-only \
  --clamp-control admittance \
  --clamp-offsets 0.09 \
  --execute
```

`--clamp-only` 只执行导纳夹紧。它复用 `data/runtime/transport_box_grasp_target_latest.json`，不会重新生成箱子目标。

单步验证通过后，业务流程使用默认单段夹紧：

```text
outside_offset -> clamp_offset
```

最后再测试：

```text
夹紧 -> 抬高 3-5cm -> 往身体方向拉回
```

导纳验证前禁止：

```text
1. 一次性大位移拉箱。
2. 一次性抬高超过 5cm。
3. 使用普通 PointsSeqTracking 做接触阶段大位移。
```

只停在预抓取点：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --skip-side-approach \
  --execute
```

只跑 FoundationPose 识别：

```bash
python apps/transport/run_foundationpose_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window
```

旧 FastSAM 备份方案：

```bash
python apps/transport/run_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --backend fastsam \
  --geometry depth-rim \
  --rim-fit-mode side-mid \
  --show-window
```

当前头部姿态已合适时：

```bash
python apps/transport/run_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --backend fastsam \
  --geometry depth-rim \
  --rim-fit-mode side-mid \
  --skip-neck-down \
  --skip-neck-home \
  --show-window
```

## 数据输出

```text
data/transport/box_grasp_target_latest.json
data/transport/box_grasp_target_latest.png
data/transport/box_grasp_debug_latest/
data/transport/foundationpose_box_grasp_debug_latest/
data/runtime/transport_box_grasp_camera_latest.json
data/runtime/transport_box_grasp_target_latest.json
data/runtime/transport_box_side_approach_latest.json
```

核心字段：

```text
objects                         left/right 抓取点，相机系
source                          foundationpose / legacy backend
rim_corners                     盒口四角
rim_meta.front_line             前沿/后沿拟合信息
rim_meta.handle_point_adjustment
                                抓取点回退到 mask 内时记录 from/to
```

## 后续接入

```text
1. 使用 FoundationPose 识别左右抓取点。
2. 将相机系左右抓取点转换到 BASE。
3. 用左右点连线重建 yaw + 固定 pitch。
4. 使用 planar offset 生成预抓取、外扩、夹紧、拉回点。
5. 复现双手预抓取姿态。
6. 从预抓取点移动到箱子两侧外扩靠近点。
7. 分段靠近并检测左右手压力。
8. 确认导纳服务后，将接触/拉出/推入切到 admittance。
9. 撤回和恢复姿态使用 with_joints。
```

## 风险点

1. FoundationPose 依赖 CAD 尺寸、相机内参和第一帧 mask；箱子型号变化时要重新建模或换配置。
2. 双手路径不能分开执行，否则容易推盒子。
3. 相机系点不能直接发送 MPC，必须先通过 CAM2HEAD + TF 锁存到 BASE。
4. 旧 FastSAM/颜色方案保留用于现场回退，不作为默认主流程。
5. 厂商 demo 里 `z += 0.35` 是 SDK/MPC 坐标补偿逻辑；如果目标已经是 MPC BASE TCP，不要再次加 0.35。
6. 导纳模式依赖机器人侧服务和配置，未验证前只允许 1cm 级小步测试。
7. 普通 `PointsSeqTracking` 不保证左右手相对位姿固定，夹紧后大位移搬运必须谨慎。

## 2026-08-19 导纳夹紧测试记录

现场状态：

```text
机器人从 home 出发。
FoundationPose 识别箱子可用。
相机系左右抓取点已成功转换到 BASE。
机器人已能到 transport_pregrasp_dual 和箱子两侧外扩点。
/wa/admittance_mode_setting 已存在。
/wa/points_seq_tracking_with_admittance 已存在。
```

本次测试参数：

```text
ws_url = ws://192.168.20.102:9091
backend = foundationpose
body_offset = -0.17 m
outside_offset = 0.10 m
side_z_offset = 0.38 m
clamp_offset = 0.03 m
clamp_control = admittance
left_hand_q = [0.2, 0.9, 0.35, 0.45, 0.56, 0.65]
right_hand_q = [0.2, 0.9, 0.35, 0.45, 0.56, 0.65]
```

已验证动作：

```text
1. 外扩点 -> 导纳夹紧到 4cm。
2. 4cm -> 导纳夹紧到 3cm。
3. 3cm 夹紧点 -> 双手同步抬高 10cm。
```

压力记录：

```text
4cm:
left  abs=0.100
right abs=0.200

3cm:
left  abs=0.100
right abs=1.000
```

结论：

```text
导纳模式开关正常。
夹紧轨迹成功走 /wa/points_seq_tracking_with_admittance。
双手抬高 10cm 成功，箱子可以被举起。
左右压力不均，右手明显先受力；当前先接受该现象，抓取姿态明天继续调整。
```

本次生成/使用的关键文件：

```text
data/runtime/transport_box_grasp_target_latest.json
data/runtime/transport_box_side_approach_latest.json
data/runtime/transport_box_clamp_latest.json
data/runtime/transport_clamp_03_pressure_uneven.json
data/runtime/transport_box_lift_10cm_latest.json
```

关键命令：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --backend foundationpose \
  --show-window \
  --clamp-control admittance \
  --clamp-offsets 0.09 \
  --execute
```

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --clamp-only \
  --clamp-control admittance \
  --clamp-offset 0.03 \
  --execute
```

```bash
python apps/transport/run_dual_pose_path.py \
  --ws-url ws://192.168.20.102:9091 \
  --target data/runtime/transport_box_lift_10cm_latest.json \
  --max-motion 2.0 \
  --duration 6.0 \
  --execute-delay 2.0 \
  --execute
```

后续继续：

```text
1. 当前左右手型已写入 transport flow 自动执行。
2. 抬高 15cm、往身体方向回收 20cm、腰部姿态调整已写入 transport flow，后续继续优化搬运直腰姿态。
3. 记录稳定 carry pose。
4. 后续再接导航阶段。
```
