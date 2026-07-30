"""
手掌关节限位和安全姿态。

q 顺序:
[THUMB_MP, THUMB_CMC, INDEX, MIDDLE, RING, LITTLE]
"""

HAND_LIMITS = [
    (-0.7854, 0.7854),  # THUMB_MP
    (-0.3491, 1.5708),  # THUMB_CMC
    (0.0, 1.3963),      # INDEX
    (0.0, 1.3963),      # MIDDLE
    (0.0, 1.3963),      # RING
    (0.0, 1.3963),      # LITTLE
]

HAND_OPEN = [-0.5, 1.2, 0.0, 0.0, 0.0, 0.0]
HAND_TEST_CLOSE = [0.2, 1.2, 0.6, 0.6, 0.6, 0.6]
HAND_CLOSE = [0.4, 1.2, 1.2, 1.2, 0.4, 0.4]


def clamp_hand_q(q):
    """将手掌 q 裁剪到机械限位内。"""
    if len(q) != len(HAND_LIMITS):
        raise ValueError(f"hand q must have {len(HAND_LIMITS)} values, got {len(q)}")
    return [
        max(low, min(high, float(value)))
        for value, (low, high) in zip(q, HAND_LIMITS)
    ]
