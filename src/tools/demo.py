import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from config_path import DEFAULT_URDF_PATH, PINOCCHIO_PACKAGE_DIRS

urdf_path = str(DEFAULT_URDF_PATH)
package_dirs = PINOCCHIO_PACKAGE_DIRS
robot = RobotWrapper.BuildFromURDF(urdf_path, package_dirs=package_dirs)

geom_model = robot.collision_model
geom_data = robot.collision_data

# 设置关节配置（中性位置）
q = pin.neutral(robot.model)

# 计算碰撞（stop_at_first_collision=False）
pin.computeCollisions(robot.model, robot.data, geom_model, geom_data, q, False)

# 检查是否有碰撞
collision_detected = False
for k, pair in enumerate(geom_model.collisionPairs):
    if geom_data.collisionResults[k].isCollision():
        print(f"检测到碰撞: {pair}")
        collision_detected = True

if not collision_detected:
    print("未检测到碰撞")
