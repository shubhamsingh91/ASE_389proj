import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math
from collections import OrderedDict

import pybullet as p
import numpy as np
import matplotlib.pyplot as pyplot

from util import pybullet_util
from config.manipulator_config import ManipulatorConfig
from pnc.manipulator_pnc.manipulator_interface import ManipulatorInterface

if __name__ == "__main__":

    # Environment Setup
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=4.0,
                                 cameraYaw=0,
                                 cameraPitch=-45,
                                 cameraTargetPosition=[1.5, 0., 0.])
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
    #for problem 2
    q1 = []
    q2 = []
    q3 = []
    x = []
    y =[]
    theta = []
    x1 = []
    x2q1 = []
    x2q2 = []
    x2q3 = []
    t_array = []
    while (t<2.0):

        # Get SensorData
        sensor_data = pybullet_util.get_sensor_data(robot, joint_id, link_id,
                                                    pos_basejoint_to_basecom,
                                                    rot_basejoint_to_basecom)

        # Compute Command
        command = interface.get_command(sensor_data)

        # Apply Trq
        pybullet_util.set_motor_trq(robot, joint_id, command)

        p.stepSimulation()

        time.sleep(dt)
        t += dt
        count += 1
        
        #for problem 2
        q = interface.currentJointPos()
        q1.append(q[0])
        q2.append(q[1])
        q3.append(q[2])
        t_array.append(count)
        
        """
        #for problem 3
        pos = interface.currentEEPos()
        x.append(pos[0])
        y.append(pos[1])
        theta.append(np.arctan(pos[1]/pos[0]))
        t_array.append(count)
        """
        """
        #for problem 4
        theta = interface.currentx1Pos()
        x2 = interface.currentx2Pos()
        x1.append(theta)
        x2q1.append(x2[0])
        x2q2.append(x2[1])
        x2q3.append(x2[2])
        t_array.append(count)
        """

    
    ###Problem 2####
    #Plotting q1,q2,q3 as a function of time as it approaches q_des
    pyplot.figure(1)
    pyplot.subplot(221)
    pyplot.plot(t_array, q1)
    pyplot.ylabel('q1 (rad)')
    pyplot.xlabel('time steps') 
    pyplot.title('q1 vs t where q1_des = 0')
    pyplot.grid(True)

    pyplot.subplot(222)
    pyplot.plot(t_array, q2)
    pyplot.ylabel('q2 (rad)')  
    pyplot.xlabel('time steps') 
    pyplot.title('q2 vs t where q2_des = pi/4')
    pyplot.grid(True)

    pyplot.subplot(223)
    pyplot.plot(t_array, q3)
    pyplot.ylabel('q3 (rad)')  
    pyplot.xlabel('time steps') 
    pyplot.title('q3 vs t where q3_des = pi/4')
    pyplot.grid(True) 
    pyplot.show()
    
    """
    ###Problem 3####
    #Plotting x,y,theta as a function of time as it approaches x_des
    pyplot.figure(1)
    pyplot.subplot(221)
    pyplot.plot(t_array, x)
    pyplot.ylabel('x')
    pyplot.xlabel('time steps') 
    pyplot.title('x vs t where x_des = 0.5')
    pyplot.grid(True)

    pyplot.subplot(222)
    pyplot.plot(t_array, y)
    pyplot.ylabel('y')  
    pyplot.xlabel('time steps') 
    pyplot.title('y vs t where y_des = 0.5')
    pyplot.grid(True)

    pyplot.subplot(223)
    pyplot.plot(t_array, theta)
    pyplot.ylabel('theta (rad)')  
    pyplot.xlabel('time steps') 
    pyplot.title('theta vs t where theta_des = pi/4')
    pyplot.grid(True) 
    pyplot.show()
    """
    """
    ###Problem 4####
    #Plotting x1 and x2 as a function of time
    pyplot.figure(1)
    pyplot.subplot(221)
    pyplot.plot(t_array, x1)
    pyplot.ylabel('x1 (rad)')
    pyplot.xlabel('time steps') 
    pyplot.title('x1 vs t where x1_des = pi/4')
    pyplot.grid(True)

    pyplot.subplot(222)
    pyplot.plot(t_array, x2q1)
    pyplot.ylabel('x2 q1')  
    pyplot.xlabel('time steps') 
    pyplot.title('x2 q1 vs t where x2 q1_des = 0')
    pyplot.grid(True)

    pyplot.subplot(223)
    pyplot.plot(t_array, x2q2)
    pyplot.ylabel('x2 q2')  
    pyplot.xlabel('time steps') 
    pyplot.title('x2 q2 vs t where x2 q2_des = 0')
    pyplot.grid(True)

    pyplot.subplot(224)
    pyplot.plot(t_array, x2q3)
    pyplot.ylabel('x2 q3')  
    pyplot.xlabel('time steps') 
    pyplot.title('x2 q3 vs t where x2 q3_des = 0')
    pyplot.grid(True) 
    pyplot.show()
    """
    