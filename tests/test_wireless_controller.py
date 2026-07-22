import sys
from pathlib import Path

# 把项目根目录（当前文件的父目录）临时加入搜索路径
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulate_python.wireless_controller import *
from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
import os
import torch


from rich.live import Live
from rich.progress import Progress
from rich.table import Table
from rich.console import Group


if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0, "lo")

    wireless_controller = WirelessController(queue_length=10)

    live = Live(auto_refresh=True)
    live.start()

    while True:
        table = Table()
        table.add_column("key name")
        table.add_column("on press")
        table.add_column("pressed")
        table.add_column("on release")
        for key in KeyIdx:
            table.add_row(
                key.name,
                f"{'[green]True' if wireless_controller.key_down(key) else '[red]False'}",
                f"{'[green]True' if wireless_controller.key_pressed(key) else '[red]False'}",
                f"{'[green]True' if wireless_controller.key_up(key) else '[red]False'}",
            )
        table.add_row(
            "left joystick",
            f"{wireless_controller.lx():.2f}",
            f"{wireless_controller.ly():.2f}",
            "-",
        )
        table.add_row(
            "right joystick",
            f"{wireless_controller.rx():.2f}",
            f"{wireless_controller.ry():.2f}",
            "-",
        )

        live.update(table)

        if wireless_controller.key_pressed(KeyIdx.L1) and wireless_controller.key_pressed(KeyIdx.L2):
            break

    live.stop()
