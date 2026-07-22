import sys
from pathlib import Path

# 把项目根目录（当前文件的父目录）临时加入搜索路径
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import threading

import numpy as np
import cv2

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from simulate_python.unitree_go_image import Image_

TOPIC_IMAGE_DEPTH = "/depth/image_raw"

_lock = threading.Lock()
_latest = None
_frame_count = 0


def depth_callback(msg: Image_):
    global _frame_count
    _frame_count += 1

    depth_mm = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape((msg.height, msg.width))
    depth_m = depth_mm.astype(np.float32) / 1000.0

    valid = depth_m > 0
    display = np.zeros_like(depth_m, dtype=np.uint8)

    if np.any(valid):
        vmin, vmax = np.percentile(depth_m[valid], [2, 98])
        if vmax <= vmin:
            vmax = vmin + 1e-6
        clipped = np.clip(depth_m, vmin, vmax)
        display = ((clipped - vmin) / (vmax - vmin) * 255).astype(np.uint8)

    depth_colored = cv2.applyColorMap(display, cv2.COLORMAP_JET)
    depth_colored[~valid] = 0

    with _lock:
        global _latest
        _latest = depth_colored

    if _frame_count <= 1:
        print(
            f"[{_frame_count}] w={msg.width} h={msg.height} data_len={len(msg.data)} "
            f"valid={np.sum(valid)} depth_m=[{depth_m[valid].min():.3f}, {depth_m[valid].max():.3f}]"
        )


if __name__ == "__main__":
    ChannelFactoryInitialize(0, "lo")

    sub = ChannelSubscriber(TOPIC_IMAGE_DEPTH, Image_)
    sub.Init(depth_callback, 10)

    cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Depth Image", 1280, 960)

    print("Waiting for depth images... Press 'q' to quit.")
    try:
        while True:
            with _lock:
                frame = _latest.copy() if _latest is not None else None
            if frame is not None:
                cv2.imshow("Depth Image", frame)
            if cv2.waitKey(33) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
