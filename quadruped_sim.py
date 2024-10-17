import pybullet as p
import pybullet_data
import time
import math
import random

def create_quadruped():
    # Create a simple quadruped robot
    quadruped = p.createMultiBody(
        baseMass=1,
        basePosition=[0, 0, 0.5],
        baseOrientation=[0, 0, 0, 1],
        baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.1, 0.05]),
        baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.1, 0.05], rgbaColor=[0.8, 0.3, 0.3, 1]),
        linkMasses=[0.1] * 4,
        linkCollisionShapeIndices=[p.createCollisionShape(p.GEOM_CYLINDER, radius=0.02, height=0.2)] * 4,
        linkVisualShapeIndices=[p.createVisualShape(p.GEOM_CYLINDER, radius=0.02, length=0.2, rgbaColor=[0.3, 0.3, 0.8, 1])] * 4,
        linkPositions=[[0.15, 0.1, 0], [0.15, -0.1, 0], [-0.15, 0.1, 0], [-0.15, -0.1, 0]],
        linkOrientations=[[0, 0, 0, 1]] * 4,
        linkInertialFramePositions=[[0, 0, 0]] * 4,
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
        linkParentIndices=[0] * 4,
        linkJointTypes=[p.JOINT_REVOLUTE] * 4,
        linkJointAxis=[[0, 1, 0]] * 4
    )

    # Set joint limits and forces
    for i in range(p.getNumJoints(quadruped)):
        p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL, force=10)
        p.changeDynamics(quadruped, i, maxJointVelocity=10)
        p.setJointMotorControl2(quadruped, i, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(quadruped, i, p.TORQUE_CONTROL, force=0)
        p.changeDynamics(quadruped, i, jointLowerLimit=-0.5, jointUpperLimit=0.5, jointLimitForce=100)

    return quadruped

def apply_gait(quadruped, t):
    # Simple alternating diagonal gait
    frequency = 1.0  # Hz
    amplitude = 0.2  # radians

    # Calculate joint angles for each leg
    angles = [
        amplitude * math.sin(2 * math.pi * frequency * t),
        amplitude * math.sin(2 * math.pi * frequency * t + math.pi),
        amplitude * math.sin(2 * math.pi * frequency * t + math.pi),
        amplitude * math.sin(2 * math.pi * frequency * t)
    ]

    # Apply the calculated angles to each joint
    for i in range(4):
        p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL, targetPosition=angles[i], force=10)

def create_obstacles(num_obstacles=5):
    obstacles = []
    for _ in range(num_obstacles):
        x = random.uniform(-2, 2)
        y = random.uniform(-2, 2)
        size = random.uniform(0.1, 0.3)
        obstacle = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[size/2, size/2, size/2]),
            basePosition=[x, y, size/2],
            baseOrientation=[0, 0, 0, 1]
        )
        obstacles.append(obstacle)
    return obstacles

def check_obstacle_in_view(quadruped, obstacles, max_distance=1.0, fov=60):
    robot_pos, robot_orn = p.getBasePositionAndOrientation(quadruped)
    robot_orn = p.getEulerFromQuaternion(robot_orn)
    
    for angle in range(-fov//2, fov//2 + 1, 5):  # Check every 5 degrees within FOV
        ray_angle = math.radians(robot_orn[2] + angle)
        ray_end = [
            robot_pos[0] + max_distance * math.cos(ray_angle),
            robot_pos[1] + max_distance * math.sin(ray_angle),
            robot_pos[2]
        ]
        
        ray_test = p.rayTest(robot_pos, ray_end)[0]
        if ray_test[0] in obstacles:
            hit_position = ray_test[3]
            distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(robot_pos, hit_position)))
            print(f"Obstacle detected at distance: {distance:.2f} meters, angle: {angle} degrees")
            return True
    
    return False

def main():
    # Initialize PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Create quadruped robot
    quadruped = create_quadruped()

    # Create obstacles
    obstacles = create_obstacles()

    # Set up camera
    p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

    # Simulation loop
    t = 0
    dt = 1/240.
    robot_stopped = False
    for _ in range(10000):
        if not robot_stopped:
            apply_gait(quadruped, t)
        
        p.stepSimulation()

        # Check for obstacle in view
        if check_obstacle_in_view(quadruped, obstacles):
            print("Robot detected an obstacle in view! Stopping.")
            robot_stopped = True

        # Update camera position to follow the robot
        robot_pos, _ = p.getBasePositionAndOrientation(quadruped)
        p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=robot_pos)

        time.sleep(dt)
        t += dt

    p.disconnect()

if __name__ == "__main__":
    main()
