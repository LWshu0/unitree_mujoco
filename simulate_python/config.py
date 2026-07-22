ROBOT = "go2"  # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene_empty.xml"  # Robot scene
DOMAIN_ID = 0  # Domain id
INTERFACE = "lo"  # Interface

USE_JOYSTICK = 0  # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox"  # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0  # Joystick number

PRINT_SCENE_INFORMATION = True  # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False  # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

# Camera
CAMERA_ENABLED = True
CAMERA_TYPE = "realsense_d435i"
CAMERA_NAME = "d435i_camera"
CAMERA_FPS = 30

# Depth randomization
DEPTH_CLIP_NEAR = 0.1
DEPTH_CLIP_FAR = 10.0
DEPTH_GAUSSIAN_NOISE_STD = 0.001
DEPTH_DROPOUT_RATE = 0.01
DEPTH_QUANTIZATION = 0.001
DEPTH_EDGE_DISTORTION = True
DEPTH_EDGE_SHIFT_STD = 0.02
DEPTH_EDGE_KERNEL_SIZE = 5
DEPTH_SYSTEMATIC_MISSING_RATE = 0.005
DEPTH_MISSING_PATCH_SIZE = (2, 10)
