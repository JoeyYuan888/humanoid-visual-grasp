"""
抓取业务编排的纯逻辑部分。

这里不直接发机器人运动命令，便于先用 CSV 或视觉输出做 debug。
后续接 SDK/MPC 时，让运动客户端实现对应方法，再由更高层调用。
"""

from __future__ import annotations

from . import config


def object_conf(obj: dict) -> float:
    """兼容实时对象的 confidence 字段和 CSV 摘要里的 conf 字段。"""
    return float(obj.get("confidence", obj.get("conf", 0.0)) or 0.0)


def select_grasp_target(object_results: list[dict], preferred_label: str | None = None) -> dict | None:
    """从视觉 object_results 中选择一个可抓目标。

    当前新业务逻辑是“先抓取，后把物体放到相机前识别 QR”，因此抓取前
    不要求 qr_text 非空，只要求 3D 坐标有效。
    """
    candidates = [
        obj for obj in object_results
        if obj.get("valid")
        and object_conf(obj) >= config.OBJECT_MIN_CONF
        and obj.get("x_mm") != ""
        and obj.get("y_mm") != ""
        and obj.get("z_mm") != ""
    ]
    if preferred_label:
        candidates = [obj for obj in candidates if obj.get("label") == preferred_label]
    if not candidates:
        return None
    return max(candidates, key=object_conf)


def bind_qr_after_grasp(grasped_object: dict, qr_text: str) -> dict:
    """抓后近距离识别到 QR 后，绑定到已抓物体。"""
    bound = dict(grasped_object)
    bound["qr_text"] = qr_text
    bound["qr_source"] = "post_grasp_camera_check"
    return bound


def summarize_target(target: dict | None) -> str:
    if target is None:
        return "no valid target"
    return (
        f"#{target.get('idx')} {target.get('label')} "
        f"conf={object_conf(target):.3f} "
        f"qr={target.get('qr_text', '')} "
        f"xyz=({target.get('x_mm', '')}, {target.get('y_mm', '')}, {target.get('z_mm', '')}) "
        f"status={target.get('status', '')}"
    )
