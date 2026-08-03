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

_latest = None
_frame_count = 0


def depth_callback(msg: Image_):
    global _frame_count
    _frame_count += 1

    depth_mm = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape((msg.height, msg.width))
    depth_m = depth_mm.astype(np.float32) / 1000.0

    valid = depth_m > 0
    vmin, vmax = 0.0, 5.0  # 固定范围
    clipped = np.clip(depth_m, vmin, vmax)
    display = ((clipped - vmin) / (vmax - vmin) * 255).astype(np.uint8)

    global _latest
    _latest = display

    if _frame_count <= 1:
        print(
            f"[{_frame_count}] w={msg.width} h={msg.height} data_len={len(msg.data)} "
            f"valid={np.sum(valid)} depth_m=[{depth_m[valid].min():.3f}, {depth_m[valid].max():.3f}]"
        )


if __name__ == "__main__":
    ChannelFactoryInitialize(0, "lo")

    sub = ChannelSubscriber(TOPIC_IMAGE_DEPTH, Image_)
    sub.Init(depth_callback, 1)

    cv2.namedWindow("Depth Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Depth Image", 640, 480)

    print("Waiting for depth images... Press 'q' to quit.")
    try:
        while True:
            gray = _latest
            if gray is not None:
                colored = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
                colored_resized = cv2.resize(colored, (640, 480), interpolation=cv2.INTER_NEAREST)
                cv2.imshow("Depth Image", colored_resized)

            if cv2.waitKey(33) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
