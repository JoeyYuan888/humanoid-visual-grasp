"""
全局配置
"""

# ==================== 网络 ====================

WS_URL = "ws://192.168.20.98:9090"

# ==================== ROS 话题 ====================

TOPIC_RGB = "/zj_humanoid/sensor/realsense_head/color/image_raw/compressed"
TOPIC_RGB_RAW = "/zj_humanoid/sensor/realsense_head/color/image_raw"
TOPIC_DEPTH = "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw"
TOPIC_DEPTH_COMPRESSED = "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth"
TOPIC_CAMERA_INFO = "/zj_humanoid/sensor/realsense_head/color/camera_info"
SERVICE_REALSENSE_RESTART = "/zj_humanoid/sensor/realsense_head/restart"

# ==================== YOLO ====================

YOLO_MODEL = "models/best.pt"   # 当前塑料袋模型；tools/test_yolo.py 已验证可识别塑料袋
YOLO_CONF = 0.25                # 模型推理阈值；塑料袋先保证召回，再由 OBJECT_MIN_CONF 做业务过滤
YOLO_IOU = 0.45                 # NMS IoU 阈值
YOLO_DEDUP_CENTER_THRESH = 80   # 同类别 bbox 中心小于该像素距离时视为重复框
YOLO_TARGET_CLASSES = []        # models/best.pt 自定义类别暂不按名称过滤，直接使用模型输出
YOLO_IMGSZ = 512                # 推理输入尺寸；小二维码/小物体阶段优先保证 ROI 召回
OBJECT_MIN_CONF = 0.30          # 业务输出最低置信度；过滤很低置信度的误检框
REQUIRE_CUDA = True             # 本项目不允许静默退回 CPU；detect 环境必须能看到 GPU
YOLO_HALF = True                # CUDA 上使用 FP16，加速推理
DETECT_EVERY_N_FRAMES = 1       # 当前 YOLO 很快，先每帧检测以提高 detect_fps
LOG_DETECTIONS = False          # 长时间实时测试时关闭逐帧日志，按 s/退出仍会保存点击记录
PERF_LOG_INTERVAL_SEC = 0.5     # 每隔多久记录一条性能数据

# ==================== 二维码 ====================

ENABLE_QR = False               # 桌面阶段只做塑料袋定位；二维码抓取后移到相机前再近距离识别
QR_DECODE_INTERVAL_SEC = 0.8    # 后台二维码解码间隔；避免 QR 线程抢占深度/RGB
QR_EXPECTED_COUNT = 3           # 当前测试桌面期望二维码数量；正式任务按实际目标数量调整
QR_ACTIVE_FULL_RESCAN_INTERVAL_SEC = 5.0 # 未扫齐时低频全量补扫，主路径仍优先扫未绑定目标
QR_LOCKED_FULL_RESCAN_INTERVAL_SEC = 12.0 # 扫齐后更低频补扫，降低对 FPS 的影响
QR_MAX_ROIS_PER_SCAN = 2        # 每次 QR 任务最多扫几个 ROI，防止全量扫描拖慢深度流
QR_PRIORITY_CLASSES = []        # 抓后 QR 检查阶段再设置目标 ROI/类别优先级
QR_MEMORY_TTL_SEC = 60.0        # 静态任务中二维码识别到一次后长时间锁存，避免重复扫码拖慢
QR_RAW_RGB_THROTTLE_MS = 1000   # 抓后静态 QR 扫码约 1Hz 订阅 raw RGB，兼顾质量和 rosbridge 负载
QR_MAX_RAW_AGE_MS = 4500        # 静态场景允许使用较旧 raw，提高小二维码识别机会
USE_SEPARATE_QR_CLIENT = True   # raw RGB 独立 WebSocket，避免大包堵塞 compressed RGB

# ==================== 深度 ====================

ENABLE_DEPTH = True             # 使用 compressedDepth，避免 raw depth over rosbridge 造成卡顿
DEPTH_TRANSPORT = "compressedDepth"  # compressedDepth / raw
DEPTH_MIN_MM = 100              # 最小有效深度 (mm)
DEPTH_MAX_MM = 5000             # 最大有效深度 (mm)
SHOW_DEPTH_OVERLAYS = False     # 十字准星/点击实时测距会访问深度；调试坐标时再打开
DEPTH_MAX_AGE_MS = 1000         # 静态抓取允许较低深度频率，但太旧则不输出抓取点
GRASP_POINT_X_RATIO = 0.50      # bbox 内抓取点 u 位置比例：0=左边，1=右边
GRASP_POINT_Y_RATIO = 0.333     # bbox 上方 2/3 区域的中点；0=上边，1=下边
DEPTH_ROI_X1_RATIO = 0.00       # bbox 内深度采样 ROI 左边界比例
DEPTH_ROI_X2_RATIO = 1.00       # bbox 内深度采样 ROI 右边界比例
DEPTH_ROI_Y1_RATIO = 0.00       # bbox 内深度采样 ROI 上边界比例；当前取全 bbox
DEPTH_ROI_Y2_RATIO = 1.00       # bbox 内深度采样 ROI 下边界比例
TARGET_STABLE_FRAMES = 2        # 静态抓取下 2 帧稳定即可开始采样
TARGET_STABLE_PIXEL_THRESH = 180 # 静态抓取下 bbox 抖动较大，中心点在该范围内视为同一目标
BBOX_SMOOTH_ALPHA = 0.20        # bbox 指数平滑系数，越小越稳
STATIC_GRASP_SAMPLES = 2        # 深度约 2Hz，累计 2 个有效点即可输出静态抓取点
TARGET_LOST_GRACE_FRAMES = 15   # YOLO 短暂漏检时保留状态和已采样深度

# ==================== 显示 ====================

WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
INFO_BAR_HEIGHT = 70

# ==================== ROS bridge 订阅 ====================

ROS_QUEUE_LENGTH = 1            # rosbridge 端只保留最新消息，减少积压延迟
USE_SEPARATE_DEPTH_CLIENT = True # RGB/depth 分开 WebSocket，避免深度回调拖慢 RGB
ROS_RGB_THROTTLE_MS = 0         # 0=不节流；需要降带宽可设 66(约15fps) / 100(约10fps)
ROS_DEPTH_THROTTLE_MS = 0       # depth 单独连接，不在 rosbridge 端节流；客户端再按需解码
DEPTH_DECODE_INTERVAL_SEC = 0.2 # 本地最多 5Hz 解码深度，保护 RGB 流畅度

# ==================== 连接超时 ====================

CONNECT_TIMEOUT = 10.0          # 连接超时 (秒)
CAMERA_GUARD_ENABLED = True     # 相机任务启动前先检查 RealSense 图像流，必要时自动 restart
CAMERA_GUARD_SAMPLE_TIMEOUT = 3.0
CAMERA_GUARD_RESTART_WAIT = 10.0
