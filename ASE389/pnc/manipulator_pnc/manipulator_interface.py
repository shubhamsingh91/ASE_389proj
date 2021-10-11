import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math

import numpy as np

from pnc.interface import Interface
from config.manipulator_config import ManipulatorConfig
from pnc.robot_system.pinocchio_robot_system import PinocchioRobotSystem


class ManipulatorInterface(Interface):
    def __init__(self):
        super(ManipulatorInterface, self).__init__()

        self._robot = PinocchioRobotSystem(
            cwd + "/robot_model/manipulator/three_link_manipulator.urdf",
            cwd + "/robot_model/manipulator", True,
            ManipulatorConfig.PRINT_ROBOT_INFO)

    def get_command(self, sensor_data):
        # Update Robot
        self._robot.update_system(
            sensor_data["base_com_pos"], sensor_data["base_com_quat"],
            sensor_data["base_com_lin_vel"], sensor_data["base_com_ang_vel"],
            sensor_data["base_joint_pos"], sensor_data["base_joint_quat"],
            sensor_data["base_joint_lin_vel"],
            sensor_data["base_joint_ang_vel"], sensor_data["joint_pos"],
            sensor_data["joint_vel"])

        # Operational Space Control
        jtrq_cmd = self._compute_osc_command()
        jpos_cmd = np.zeros_like(jtrq_cmd)
        jvel_cmd = np.zeros_like(jtrq_cmd)

        # Compute Cmd
        command = self._robot.create_cmd_ordered_dict(jpos_cmd, jvel_cmd,
                                                      jtrq_cmd)

        # Increase time variables
        self._count += 1
        self._running_time += ManipulatorConfig.DT

        return command

    def _compute_osc_command(self):
        jtrq = np.zeros(self._robot.n_a)
        
        ###Problem 2####
        #desired joint position vector q_des
        q_des = [0, np.pi/4, np.pi/4]
        #use function Jpos() to calulcate joint torque command for given q_des
        jtrq = self.Jpos(q_des)
        
        """
        ###Problem 3####
        #desired end effector position vector x_des
        x_des = [0.5, 0.5, np.pi/4]
        #use function Jpos() to calulcate joint torque command for given x_des
        jtrq = self.OpSpace(x_des)
        """
        """
        ###Problem 4####
        #desired end effector orientation x1_des
        x1_des = np.pi/4
        #desired joint position vector x2_des
        x2_des = [0, 0, 0]
        #use function Jpos() to calulcate joint torque command for given q_des
        jtrq = self.prioritizeTask(x1_des, x2_des)
        """
        return jtrq

    def Jpos(self, q_des):
        
        ###Problem 2####
        #identify the simulated joint position vector q and simulated q_dot
        q = self._robot.get_q()
        q_dot = self._robot.get_q_dot()
        KP = 400
        KD = 40
        #PD controller
        a_ref = (-1 * KP * (q - q_des)) - (KD * q_dot)
        #the joint accelerations are equal to our PD controller
        qddot_ref = a_ref
        #identify mass matrix A, coriolis vector b, and gravity g
        A = self._robot.get_mass_matrix()
        b = self._robot.get_coriolis()
        g = self._robot.get_gravity()
        #calculate torques using joint space equations of motion
        torque = np.dot(A, qddot_ref) + b + g
        
        return torque

    #For problem 2 plotting
    def currentJointPos(self):

        q = self._robot.get_q()
        
        return q

    def OpSpace(self, x_des):
        
        ###Problem 3####
        jac = self._robot.get_link_jacobian('ee')[3:6, :]
        #jacdot = self._robot.get_link_jacobian_dot('ee')[3:6, :]
        jacdot = self._robot.get_link_jacobian_dot_times_qdot('ee')[3:6]
        pos = self._robot.get_link_iso('ee')[0:3, 3]
        vel = self._robot.get_link_vel('ee')[3:6]
        q = self._robot.get_q()
        q_dot = self._robot.get_q_dot()
        A = self._robot.get_mass_matrix()
        b = self._robot.get_coriolis()
        g = self._robot.get_gravity()
        M = np.linalg.pinv(np.matmul(np.matmul(jac, np.linalg.inv(A)), np.transpose(jac)), rcond=1e-3)
        KP = 10
        KD = 10
        #PD controller
        a_ref = (-1 * KP * (pos - x_des)) - (KD * vel)
        #ot = np.matmul(-1*jacdot, q) + np.matmul(np.linalg.inv(A), (b+g))
        ot = -1*jacdot + np.matmul(np.linalg.inv(A), (b+g))
        F = np.dot(M, (a_ref + ot))
        torque = np.matmul(np.transpose(jac), F)

        return torque

    #For problem 3 plotting
    def currentEEPos(self):

        pos = self._robot.get_link_iso('ee')[0:3, 3]
        
        return pos

    def prioritizeTask(self, x1_des, x2_des):

        ###Problem 4####
        
        J1 = [1, 1, 1]
        J2 = self._robot.get_link_jacobian('ee')[3:6, :]
    
        J1dot = self._robot.get_link_jacobian_dot_times_qdot('ee')[3:6]
        
        #identify the simulated joint position vector q and simulated q_dot
        x2 = self._robot.get_q()
        x2_dot = self._robot.get_q_dot()
        x1 = x2[0] + x2[1] + x2[2]
        x1_dot = np.dot(J1, np.transpose([x2_dot[0], x2_dot[1], x2_dot[2]]))

        A = self._robot.get_mass_matrix()
        Ainv = np.linalg.inv(A)
        b = self._robot.get_coriolis()
        g = self._robot.get_gravity()
        I = np.identity(3)
        X = np.matmul(J1, Ainv)
        Y = np.dot(X, np.transpose(J1))
        #M1 = np.linalg.pinv(Y, rcond=1e-3)
        M1 = 1/Y
        #J1_bar = np.matmul(np.matmul(Ainv, np.transpose(J1)), M1) 
        J1_bar = np.dot(Ainv, np.transpose(J1)) * M1
        
        N1 = I - np.matmul(J1_bar, J1)
        
        KP = 10
        KD = 10
        #PD controller for x1
        a1_ref = (-1 * KP * (x1 - x1_des)) - (KD * x1_dot)
        #PD controller for x2
        a2_ref = (-1 * KP * (x2 - x2_des)) - (KD * x2_dot)

        #ot is 3x1
        ot = -1*J1dot + np.matmul(Ainv, (b+g))

        F1 = M1 * (a1_ref + ot)
        #J21 is 3x3
        J21 = np.matmul(J2, N1)
        M21 = np.linalg.pinv(np.matmul(np.matmul(J21, Ainv), np.transpose(J21)), rcond=1e-3)
        F2 = np.matmul(M21, a2_ref) + ot
        J1_trans = np.transpose(J1)
        J21_trans = np.transpose(J21)

        torque = np.matmul(J1_trans, F1) + np.matmul(J21_trans, F2)

        return torque

    def currentx1Pos(self):

        x1 = self._robot.get_link_iso('ee')[0:3, 3]
        theta = np.arctan(x1[1]/x1[0])
    
        return theta

    def currentx2Pos(self):

        x2 = self._robot.get_q()
    
        return x2