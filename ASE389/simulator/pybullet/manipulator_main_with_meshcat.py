import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math
from collections import OrderedDict

import pybullet as p
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import pinocchio as pin

from util import pybullet_util
from config.manipulator_config import ManipulatorConfig
from pnc.manipulator_pnc.manipulator_interface import ManipulatorInterface

if __name__ == "__main__":

    # Environment Setup
    p.connect(p.DIRECT)
    # p.resetDebugVisualizerCamera(cameraDistance=4.0,
    # cameraYaw=0,
    # cameraPitch=-45,
    # cameraTargetPosition=[1.5, 0., 0.])
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(fixedTimeStep=ManipulatorConfig.DT,
                                numSubSteps=ManipulatorConfig.N_SUBSTEP)
    if ManipulatorConfig.VIDEO_RECORD:
        if not os.path.exists('video'):
            os.makedirs('video')
        for f in os.listdir('video'):
            os.remove('video/' + f)
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "video/atlas.mp4")

    # Create Robot, Ground
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = p.loadURDF(cwd +
                       "/robot_model/manipulator/three_link_manipulator.urdf",
                       useFixedBase=True)

    # Create Robot for Meshcat Visualization
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        cwd + "/robot_model/manipulator/three_link_manipulator.urdf",
        cwd + "/robot_model/manipulator")
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    try:
        viz.initViewer(open=True)
    except ImportError as err:
        print(
            "Error while initializing the viewer. It seems you should install Python meshcat"
        )
        print(err)
        sys.exit(0)
    viz.loadViewerModel()
    vis_q = pin.neutral(model)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot)

    # Set Initial Config
    p.resetJointState(robot, 0, -np.pi / 6., 0.)
    p.resetJointState(robot, 1, np.pi / 6., 0.)
    p.resetJointState(robot, 2, np.pi / 3., 0.)

    # Joint Friction
    pybullet_util.set_joint_friction(robot, joint_id, 0.1)

    # Construct Interface
    interface = ManipulatorInterface()

    # Run Sim
    t = 0
    dt = ManipulatorConfig.DT
    count = 0
    while (1):

        # Get SensorData
        sensor_data = pybullet_util.get_sensor_data(robot, joint_id, link_id,
                                                    pos_basejoint_to_basecom,
                                                    rot_basejoint_to_basecom)

        # Compute Command
        command = interface.get_command(sensor_data)

        # Apply Trq
        pybullet_util.set_motor_trq(robot, joint_id, command)

        # Visualize via Meshcat
        for i, (k, v) in enumerate(sensor_data['joint_pos'].items()):
            vis_q[i] = v
        viz.display(vis_q)

        p.stepSimulation()

        time.sleep(dt)
        t += dt
        count += 1
