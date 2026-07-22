import sys
from pathlib import Path

# 把项目根目录（当前文件的父目录）临时加入搜索路径
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import pygame


pygame.init()
# 【关键】必须创建一个窗口，否则事件循环不会工作
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption("手柄测试")

pygame.joystick.init()
count = pygame.joystick.get_count()
print(f"检测到手柄数量：{count}")

if count == 0:
    print("没有手柄，请检查连接")
    sys.exit()

# 尝试索引 0 或 1，因为你的设备是 js1，可能索引是 1
for i in range(count):
    joy = pygame.joystick.Joystick(i)
    joy.init()
    print(f"手柄 {i} 名称：{joy.get_name()}")

# 这里我们假设只有一个手柄，但索引可能是1，你打印出来看名称
# 通常如果有多个，手柄会按顺序分配，但如果 js0 不存在，js1 可能被映射为索引0，所以要遍历打印。

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.JOYBUTTONDOWN:
            print(f"按钮 {event.button} 按下")
        if event.type == pygame.JOYAXISMOTION:
            print(f"轴 {event.axis} 值：{event.value}")
        if event.type == pygame.JOYHATMOTION:
            print(f"苦力帽 {event.hat} 方向：{event.value}")
    pygame.display.flip()

pygame.quit()
