# 手眼对齐正式方案（2026-07-21）

这份文档只保留课程手册方案。之前的固定脖子 `camera -> MPC` 点对拟合属于实验简化方案，已经废弃，后续不再作为抓取坐标转换依据。

## 结论

正式路线采用课程手册方案：

```text
物体相机坐标
  -> CAM2HEAD 手眼标定矩阵
  -> HEAD->BASE 实时 tf
  -> 物体 BASE 位姿
  -> 抓取偏移 GRASP_OFFSET
  -> TCP/MPC 目标位姿
```

## 课程手册标准方案

课程手册给出的链路是：

```text
相机坐标系
  -> CAM2HEAD
  -> HEAD 坐标系
  -> HEAD2BASE(tf)
  -> BASE 坐标系
  -> 抓取偏移
  -> TCP 目标位姿
```

对应项目里的数据来源：

| 数据 | 来源 |
|---|---|
| RGB 图像 | `/zj_humanoid/sensor/realsense_head/color/image_raw/compressed` |
| aligned depth | `/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth` |
| 相机内参 | `/zj_humanoid/sensor/realsense_head/color/camera_info` |
| 视觉 3D 点 | `robot_grasp/vision/depth_utils.py` 反投影输出 |
| MPC 末端位姿 | `/DualArmMobile/currentEEPose/FrameR` / `FrameL` |
| SDK TCP 位姿 | `/zj_humanoid/upperlimb/tcp_pose/right_arm` / `left_arm` |
| HEAD->BASE | 需要从 tf 或机器人接口确认 |
| CAM2HEAD | 需要手眼标定得到 |
| GRASP_OFFSET | 需要根据手掌、塑料袋抓取策略设定 |

优点：

```text
和课程手册一致
脖子角度变化时可通过 HEAD->BASE(tf) 处理
后续 SDK/MPC 都能复用 BASE 下物体位姿
适合正式交付
```

当前缺口：

```text
已看到 RealSense 头部相机内部静态 tf
已看到机器人身体链路可到 HEAD
还没看到 HEAD 到 realsense_head_link 的安装外参
还没确认参数里的 base2cam 是否就是可用外参
还没定义塑料袋抓取偏移 GRASP_OFFSET
```

## 2026-07-22 当前现场输出判断

`/tf_static` 已确认以下 RealSense 内部 frame：

```text
realsense_head_link -> realsense_head_depth_frame
realsense_head_depth_frame -> realsense_head_depth_optical_frame
realsense_head_link -> realsense_head_color_frame
realsense_head_link -> realsense_head_aligned_depth_to_color_frame
realsense_head_aligned_depth_to_color_frame -> realsense_head_color_optical_frame
```

其中视觉检测使用的是 aligned depth to color，所以后续相机坐标 frame 应优先按：

```text
realsense_head_color_optical_frame
```

但当前 `/tf_static` 只包含 RealSense 内部外参，没有看到：

```text
HEAD -> realsense_head_link
BASE -> HEAD
BASE -> realsense_head_color_optical_frame
```

`/tf` 当前只采到：

```text
/body_norm -> /imu
```

这说明课程手册链路还差机器人本体的 HEAD/BASE tf，或者这些变换没有发布到当前 rosbridge 环境。

`rosparam list` 中发现大量疑似已有标定参数：

```text
/jzhw/calib/camera/up/*_base2cam
/jzhw/calib/camera/down/*_base2cam
/jzhw/camera/info/up/base/calib_param/*_base2cam
/jzhw/camera/info/down/base/calib_param/*_base2cam
/j1/jzhw/model/camera/*/*_base2cam
/j3/jzhw/model/camera/*/*_base2cam
```

下一步要读取这些参数的具体值，判断哪一路对应头部 RealSense。

已读取到的 `up/down` 参数分别对应：

```text
/cam_top_docking
/cam_bottom_docking
```

这两个不是当前视觉使用的 `/zj_humanoid/sensor/realsense_head/...`，因此不能直接作为头部 RealSense 的 `CAM2HEAD` 或 `BASE->CAM` 外参。继续找真正的 `realsense_head` 到 HEAD/BASE 的 tf 或参数。

10 秒动态 `/tf` 采样中已经看到身体链路：

```text
root -> BASE
BASE -> ANKLE
... -> WAIST_YAW -> WAIST_PITCH -> NECK -> HEAD
```

这说明 `HEAD->BASE` 可以从 tf 树获得。当前真正缺的是：

```text
HEAD -> realsense_head_link
```

如果这段安装外参不存在于 tf/rosparam 中，就必须按课程手册做 `CAM2HEAD` 标定。

新版 `debug_handeye_sources.py` 的路径检查已确认：

```text
[✓] BASE -> HEAD
[✓] root -> HEAD
[✗] BASE -> realsense_head_link
[✗] HEAD -> realsense_head_link
[✗] BASE -> realsense_head_color_optical_frame
[✗] HEAD -> realsense_head_color_optical_frame
```

最终判断：

```text
HEAD->BASE: 已确认可由 tf 获得
RealSense 内部光学 frame: 已确认可由 tf_static 获得
CAM2HEAD / HEAD->realsense_head: 当前系统没有发布
```

RViz 观察结果也支持这个判断：

```text
Fixed Frame = BASE: 能看到机器人身体 tf 树
Fixed Frame = realsense_head_link: 只能看到 RealSense 内部 frame
两边没有连成同一棵 tf 树
```

因此下一步不是继续查 `up/down` 标定参数，而是二选一：

```text
1. 找厂家/课程材料要 HEAD -> realsense_head_link 安装外参
2. 自己按课程手册做头部 RealSense 的 CAM2HEAD 标定
```

## CAM2HEAD 标定执行方案

厂商已提供和示例图一致的手背标定板工装，当前优先使用厂家工装路线。工装方案比手点 TCP 标记更可靠，原因是：

```text
工装几何尺寸/点位已知
点位可覆盖多个深度和图像区域
不依赖手点 TCP 尖端的点击精度
更接近课程手册正式标定流程
```

### 厂家手背标定板路线（当前主路线）

该路线假设标定板固定在右手手背，且厂家给出的 `T/R` 表示 TCP 到标定板坐标系的固定变换：

```text
BASE_T_TCP * TCP_T_BOARD
=
BASE_T_HEAD * HEAD_T_CAM * CAM_T_BOARD
```

因此每个采样姿态可直接得到：

```text
HEAD_T_CAM = inv(BASE_T_HEAD) * BASE_T_TCP * TCP_T_BOARD * inv(CAM_T_BOARD)
```

当前默认配置文件：

```text
data/calibration/vendor_handback_board_20260729.json
```

其中 `tcp_to_board.translation_m/rpy_deg` 已按厂家示例图先填入：

```text
T(mm) = [2.351, -1.143, 5.623]
R(deg) = [0.573, -1.146, 0.286]
```

但正式使用前必须核对：

```text
1. marker dictionary 是否正确，例如 DICT_4X4_50。
2. marker_size_m 是否等于实物边长。
3. markers 里 id=0/1/2/3 的布局是否和实物一致。
4. 厂家 T/R 的方向是否确实是 TCP -> BOARD。
```

采集命令：

```bash
conda activate detect
python tools/calibration/collect_vendor_board_cam2head.py \
  --ws-url ws://192.168.20.102:9091 \
  --arm right
```

操作：

```text
1. 进入试教模式，把右手手背工装移动到头部 RealSense 可见位置。
2. 画面识别到至少 2 个 marker 后，按 c 记录当前姿态。
3. 移动到下一个位置，停稳 1-2 秒，再按 c。
4. 采 8-12 组，覆盖画面中心、左、右、上、下，以及近/远不同深度。
5. 按 q 退出，CSV 会保存在 data/samples/。
```

求解命令：

```bash
python tools/calibration/solve_vendor_board_cam2head.py \
  data/samples/vendor_board_cam2head_samples_xxx.csv
```

输出：

```text
data/calibration/cam2head_vendor_board_xxx.json
```

判断标准：

```text
translation max error < 20-30mm：可以进入实物验证
translation max error > 30mm：优先检查 marker 尺寸、布局、TCP_T_BOARD 方向
```

### 手动点击 TCP 点路线（备用）

如果厂家板识别不稳定，仍可退回手动点击点对路线。无论工装形式是什么，最终都要得到同一批点对：

```text
cam_x_m, cam_y_m, cam_z_m
head_x_m, head_y_m, head_z_m
```

然后用：

```bash
python tools/calibration/solve_cam2head.py data/samples/cam2head_pairs_xxx.csv
```

解算并保存：

```text
data/calibration/cam2head_xxx.json
```

如果工装能直接输出/提供 `HEAD -> realsense_head_link` 或 `HEAD -> realsense_head_color_optical_frame`，则不需要采点解算，直接把该外参转成 4x4 `CAM2HEAD` 即可。

## 2026-07-24 厂商候选矩阵

厂商提供了一份别人标出来的候选 `CAM2HEAD`：

```text
data/calibration/cam2head_vendor_20260724.json
```

矩阵方向暂按厂商说明理解为：

```text
p_head = CAM2HEAD @ p_camera
```

已做的离线检查：

```text
rotation det = 1.000000
orthogonality error ~= 9e-9
translation ~= [0.0656, 0.0329, 0.1259] m
```

这说明矩阵数学上是一个正常刚体变换，但它来自“别人标出来的”结果，不能直接作为本机最终外参。当前只能用于 dry-run 和尺子/工装对照验证，不能直接发运动命令。

测试命令：

```bash
python tools/calibration/test_cam2head_candidate.py --latest-csv
python tools/calibration/test_cam2head_candidate.py --point-mm 100 150 1200
```

验证建议：

```text
1. 把同一个物体放在桌面不同左右位置，观察 HEAD 坐标横向轴是否单调变化。
2. 把同一个物体前后移动，观察 HEAD 坐标前向/深度相关轴是否单调变化。
3. 用厂商工装或尺子量一个点在 HEAD/BASE 下的大概位置，比较误差。
4. 如果误差超过抓取容忍范围，必须重新做本机 CAM2HEAD 标定。
```

当前仓库也提供了一个无工装备选采集脚本：

```bash
python tools/calibration/collect_cam2head_pairs.py
```

它通过“点击 TCP 尖端/固定小标记 + 读取右臂 TCP pose + tf 转 HEAD”生成点对。该方法只作为备选，使用时必须保证点击点就是 TCP 参考点或已知固定偏移点。

## 当前推荐执行顺序

### 第 1 步：确认标准链路需要的 tf

先查 tf frame：

```bash
rostopic echo -n 1 /tf
rostopic echo -n 1 /tf_static
```

如果环境有 tf 工具：

```bash
rosrun tf view_frames
rosrun tf tf_echo base_link head_link
```

实际 frame 名称要以现场输出为准，可能不叫 `base_link/head_link`。

### 第 2 步：确认是否已有相机外参

查找 RealSense 到 head/base 的静态 tf 或参数：

```bash
rosparam list | grep -i camera
rosparam list | grep -i realsense
rosparam list | grep -i head
rostopic echo -n 1 /tf_static
```

如果已有 `camera_color_optical_frame -> head` 或类似静态变换，优先使用已有外参，不重复做手动拟合。

为了少手敲参数，可以运行只读检查脚本：

```bash
python tools/calibration/debug_handeye_sources.py
```

### 第 3 步：没有外参时，再做 CAM2HEAD 标定

标准 CAM2HEAD 标定需要明确的相机坐标点和 HEAD/BASE 下对应点。可用标定板、AprilTag、Aruco、TCP 触碰点等方法，最终输出一个 4x4 矩阵：

```text
CAM2HEAD
```

### 第 4 步：定义抓取偏移

塑料袋不是刚性规则物体，抓取点不一定是 bbox 中心。第一版可以用经验偏移：

```text
物体中心/候选抓取点 -> TCP 目标位姿
```

后面再根据抓取成功率调整。

## 当前状态

```text
MPC points_seq_tracking: 请求结构已 dry-run 通过，但 rosbridge 缺 mpc_target，不能直接 execute
joints_seq_tracking: 暂停，等待 currenState/ocs2_msgs 问题解决
手眼标准方案: 需要先确认 tf frame 和是否已有 CAM2HEAD/相机外参
废弃方案: collect_handeye_pairs.py / solve_handeye_alignment.py 不再作为正式路线
```
