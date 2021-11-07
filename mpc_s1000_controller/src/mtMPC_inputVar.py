#!/usr/bin/env python3
# ROS python API
import rospy
import numpy as np
from numpy import array, dot
from scipy import io
from scipy.sparse import csc_matrix
from threading import Semaphore

import qpsolvers as qp
from qpsolvers import solve_qp
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Vector3Stamped, Point
from mpc_s1000_controller.msg import PolytopeMatricesStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import UInt8
import time
from math import sqrt

# from dji_sdk.msg import
# DA MODIFICARE
filepath="/root/mtMPC_ws/src/mpc_s1000_controller/src/data_inputvar.mat"
data = io.loadmat(filepath)
######
A = data['A']
A_ct = data['A_ct']
Abar = data['Abar']
Abarbar = data['Abarbar']
Abarbar2 = data['Abarbar2']
B = data['B']
B_ct = data['B_ct']
Bbar = data['Bbar']
Bbar2 = data['Bbar2']
Bbarbar = data['Bbarbar']
Bbarbar2 = data['Bbarbar2']
Bbarbar3 = data['Bbarbar3']
C = data['C']
C2 = data['C2']
C_ct = data['C_ct']
C_vel = data['C_vel']
Cbar_pos = data['Cbar_pos']
Cbar_pos2 = data['Cbar_pos2']
Cbar_pos_2 = data['Cbar_pos_2']
Cbar_pos_xy = data['Cbar_pos_xy']
Cbar_vel = data['Cbar_vel']
Cbar_vel2 = data['Cbar_vel2']
Cbarbar_ex = data['Cbarbar_ex']
Cbarbar_ex_t = data['Cbarbar_ex_t']
Cbarbar_pos = data['Cbarbar_pos']
Cbarbar_pos_xy = data['Cbarbar_pos_xy']
Cxy = data['Cxy']
D = data['D'].item()
Gamma_xi = data['Gamma_xi']
#Gamma_y = data['Gamma_y']
Gamma_ye = data['Gamma_ye']
Gamma_ys = data['Gamma_ys']
#k1 = data['k1'].item()
#k2 = data['k2'].item()
Lambda_xi = data['Lambda_xi']
#Lambda_y = data['Lambda_y']
Lambda_ye = data['Lambda_ye']
Lambda_ys = data['Lambda_ys']
max_acc = data['max_acc'].item()
N = data['N'].item()
nu = data['nu'].item()
nx = data['nx'].item()
nxi = data['nxi'].item()
ny = data['ny'].item()
ps = data['ps'].item()
Q = data['Q']
Qbar = data['Qbar']
R = data['R']
Rbar = data['Rbar']
Ts = data['Ts'].item()
v_max = data['v_max'].item()
Aeq = data['Aeq'].astype(np.double)
beqwithoutx0 = data['beqwithoutx0'].astype(np.double)
Aacc = data['Aacc'].astype(np.double)
Aineq2row = data['Aineq2row'].astype(np.double)
Aineq3row = data['Aineq3row'].astype(np.double)
bacc1temp = data['bacc1temp'].astype(np.double)
bacc2temp = data['bacc2temp'].astype(np.double)

bineq2rowtemp = data['bineq2rowtemp'].astype(np.double)
bineq3rowtemp = data['bineq3rowtemp'].astype(np.double)
f1temp = data['f1temp'].astype(np.double)
f2temp = data['f2temp'].astype(np.double)
f3temp = data['f3temp'].astype(np.double)
f4temp = data['f4temp'].astype(np.double)
H = data['H'].astype(np.double)
H = csc_matrix(H)
rho =data['rho'].astype(np.double)

Aineq4row = Aacc
Aeq = csc_matrix(Aeq)
Yref = np.zeros(((N+1)*nxi,1))

class mtMPC():
    def __init__(self):
        #### DA MODIFICARE
        rospy.Subscriber('/dji_sdk/local_position',PointStamped,self.callback_loc_pos)
        self.position_sem = Semaphore(0)
        rospy.Subscriber('/dji_sdk/velocity',Vector3Stamped,self.callback_vel)
        self.velocity_sem = Semaphore(0)
        rospy.Subscriber('/mtMPC/temp_target',Point,self.callback_target)
        self.target_sem = Semaphore(0)
        rospy.Subscriber('/free_polytope',PolytopeMatricesStamped,self.callback_polytope)
        self.polytope_sem = Semaphore(0)
        #####
        self.local_pos = PoseStamped()
        self.vel = TwistStamped()
        self.setpoint = PoseStamped()
        self.frame = 'odom'
        self.predTraj_explo = Path()
        self.predTraj_safe = Path()
        self.x_goal_mark = Marker()
        self.x_goal_mark.header.frame_id = self.frame
        self.x_goal_mark.color.a = 1
        self.x_goal_mark.color.r = 0
        self.x_goal_mark.color.g = 1
        self.x_goal_mark.color.b = 0
        self.x_goal_mark.type = 2
        self.x_goal_mark.scale.x = 1
        self.x_goal_mark.scale.y = 1
        self.x_goal_mark.scale.z = 1
        self.x_goal = np.array([[0],[0],[3],[0],[0],[0]])
        self.x_goal_mark.pose.position.x=self.x_goal[0].item()
        self.x_goal_mark.pose.position.y=self.x_goal[1].item()
        self.x_goal_mark.pose.position.z=self.x_goal[2].item()
        self.setpt_pub = rospy.Publisher('/mtMPC/mtMPC_pos_ref', PoseStamped, queue_size=1)
        self.traj_explo_pub = rospy.Publisher('/mtMPC/pred_trajectory_exploit',Path,queue_size=1)
        self.traj_safe_pub = rospy.Publisher('/mtMPC/pred_trajectory_safe',Path,queue_size=1)
        self.mark_goal_pub = rospy.Publisher('/mtMPC/x_goal_marker',Marker,queue_size=1)
        self.n = rospy.get_param("/polytope_generation_param/num_vertices")
        self.A1 = np.zeros([self.n,1], dtype=np.float)
        self.A2 = np.zeros([self.n,1], dtype=np.float)
        self.bc = np.empty([self.n,1], dtype=np.float)
        self.Ac = np.empty([self.n,2], dtype=np.float)
        self.bc_old = np.empty([self.n,1], dtype=np.float)
        self.Ac_old = np.empty([self.n,2], dtype=np.float)
        self.toll_target = 1 # 1 ok

    def callback_target(self,msg):
        self.x_goal[0] = msg.x
        self.x_goal[1] = msg.y
        self.x_goal[2] = msg.z
        self.x_goal[3] = 0
        self.x_goal[4] = 0
        self.x_goal[5] = 0
        self.x_goal_mark.pose.position.x=msg.x
        self.x_goal_mark.pose.position.y=msg.y
        self.x_goal_mark.pose.position.z=msg.z
        self.target_sem.release()

    def callback_loc_pos(self,msg):
        self.local_pos.pose.position.x = msg.point.x
        self.local_pos.pose.position.y = msg.point.y
        self.local_pos.pose.position.z = msg.point.z
        self.position_sem.release()

    def callback_vel(self,msg):
        self.vel.twist.linear.x = msg.vector.x
        self.vel.twist.linear.y = msg.vector.y
        self.vel.twist.linear.z = msg.vector.z
        self.velocity_sem.release()

    def callback_polytope(self,msg):
        self.A1 = np.array(msg.polytope.A1, dtype=np.float)
        self.A2 = np.array(msg.polytope.A2, dtype=np.float)
        bc_temp = np.array(msg.polytope.b, dtype=np.float)
        self.A1 = self.A1.reshape((np.size(self.A1,0),1))
        self.A2 = self.A2.reshape((np.size(self.A2,0),1))
        bc_temp = bc_temp.reshape((np.size(bc_temp,0),1))
        #self.Ac[:, 0] = self.A1
        #self.Ac[:, 1] = self.A2
        Ac_temp = np.concatenate((self.A1,self.A2),axis = 1)
        if self.local_pos.pose.position.z<1.00:
                Ac_temp = np.array([[0.002,0.001],
                [0.002,-0.001],
                [-0.002,0.001],
                [-0.002,-0.001],
                [0.0000001, 0.003],
                [-0.0000001, -0.003]])
                bc_temp = np.array([[0.99],[0.99],[1.01],[1.01],[1],[1]])
        self.Ac = np.concatenate((Ac_temp,np.array([[-1,0]])),axis = 0)
        self.bc = np.concatenate((bc_temp,np.array([[9]])),axis=0)
        self.polytope_sem.release()


    def solveqp(self):
        x0 = np.array([[self.local_pos.pose.position.x],
             [self.local_pos.pose.position.y],
             [self.local_pos.pose.position.z],
             [self.vel.twist.linear.x],
             [self.vel.twist.linear.y],
              [self.vel.twist.linear.z]])
        # Construction of the matrices for the QP
        nqxi = np.size(self.Ac,0)
        nyc = np.size(self.Ac,1)
        Acbar = np.zeros(((N+1)*nqxi,(N+1)*nyc))
        bcbar = np.zeros(((N+1)*nqxi,1))
        for ind in range(0,N+1):
            Yref[(ind)*nxi:(ind+1)*nxi] = self.x_goal[0:6]
            Acbar[(ind)*nqxi:(ind+1)*nqxi,ind*nyc:(ind+1)*nyc] = self.Ac
            bcbar[(ind)*nqxi:(ind+1)*nqxi]=self.bc

        bacc3temp = np.dot(bacc1temp,x0)+np.dot(bacc2temp,x0)
        bacc1row = max_acc-bacc3temp
        bacc2row = max_acc+bacc3temp
        bacc = np.concatenate((bacc1row,bacc2row),axis = 0)

        f1 = np.dot(x0.T,Lambda_ye.T)
        f2 = np.dot(f1,Qbar)
        f3 = np.dot(f2,Gamma_ye)
        f4 = np.dot(Yref.T,Qbar)
        f = f3-np.dot(f4,Gamma_ye)

        f1 = np.dot(np.transpose(x0),f1temp)
        f2 = np.dot(np.transpose(Yref),f2temp)
        f3 = np.dot(np.transpose(x0),f3temp)
        f4 = np.dot(np.transpose(x0),f4temp)
        f = f1-f2-f3+f4

        rhon = rho
        rhon[0][-1]=-1
        Aineq1row = np.dot(Acbar,Gamma_ys)+rhon
        Aineq5row = rhon
        Aineq = np.concatenate((Aineq1row,Aineq2row,Aineq3row,Aineq4row,Aineq5row),axis=0)
        #Aineq = np.concatenate((Aineq1row,Aineq2row,Aineq3row,Aineq5row),axis=0)
        bineq1rowtemp = np.dot(Acbar,Lambda_ys)
        bineq1row = bcbar-np.dot(bineq1rowtemp,x0)
        bineq2row = v_max+np.dot(bineq2rowtemp,x0)
        bineq3row = v_max+np.dot(bineq3rowtemp,x0)
        bineq4row = bacc
        bineq5row = np.zeros((1, 1))
        bineq = np.concatenate((bineq1row,bineq2row,bineq3row,bineq4row,bineq5row),axis=0)
        #bineq = np.concatenate((bineq1row,bineq2row,bineq3row,bineq5row),axis=0)

        beq = np.dot(beqwithoutx0,x0)
        f = np.array(f).reshape((np.size(f,1),))
        f = np.array(f)

        #Aineq = np.array(Aineq)
        Aineq = csc_matrix(Aineq)
        bineq = np.array(bineq).reshape((np.size(bineq,0),))
        beq = np.array(beq).reshape((np.size(beq,0),))
        f = f.astype(np.double)
        Aineq = Aineq.astype(np.double)
        bineq = bineq.astype(np.double)
        beq = beq.astype(np.double)
        # solving the QP
        sol = qp.osqp_solve_qp(H,f,Aineq,bineq,Aeq,beq)
        #rospy.loginfo(qp.available_solvers)
        if sol is None:
            rospy.logwarn('INFEASIBLE')
            # rospy.loginfo('H:')
            # rospy.loginfo(H)
            # rospy.loginfo('f:')
            # rospy.loginfo(f)
            # rospy.loginfo('Aineq:')
            # rospy.loginfo(Aineq)
            # rospy.loginfo('bineq:')
            # rospy.loginfo(bineq)#
            rospy.loginfo('x0:')
            rospy.loginfo(x0)
        else:
            print('feasible')
            rospy.loginfo('Slack:')
            rospy.loginfo(sol[-1])
            if sol[-1]<-0.01:
                rospy.logerr('Slack negative')
        return sol

    def plot_input(self,sol):
        x0 = np.array([[self.local_pos.pose.position.x],
             [self.local_pos.pose.position.y],
             [self.local_pos.pose.position.z],
             [self.vel.twist.linear.x],
             [self.vel.twist.linear.y],
              [self.vel.twist.linear.z]])
        Xbarbar = np.dot(Lambda_xi,x0)+np.dot(Gamma_xi,sol.reshape((np.size(sol,0),1)))
        self.predTraj_explo = Path()
        self.predTraj_safe = Path()
        temp_pose_ex = PoseStamped()
        temp_pose_safe = PoseStamped()
        temp_pose_ex.pose = self.local_pos.pose
        temp_pose_safe.pose = self.local_pos.pose
        temp_pose_ex.header.stamp = rospy.Time.now()
        temp_pose_safe.header.stamp = rospy.Time.now()
        temp_pose_ex.header.seq = 0
        temp_pose_ex.header.frame_id = self.frame
        temp_pose_safe.header.frame_id = self.frame
        temp_pose_safe.header.seq = 0
        self.predTraj_explo.poses.append(temp_pose_ex)
        self.predTraj_safe.poses.append(temp_pose_safe)
        self.predTraj_explo.header.stamp = rospy.Time.now()
        self.predTraj_safe.header.stamp = rospy.Time.now()
        self.predTraj_explo.header.frame_id = self.frame
        self.predTraj_safe.header.frame_id = self.frame

        temp_pose_ex = PoseStamped()
        temp_pose_safe = PoseStamped()
        temp_pose_ex.pose.position.x = Xbarbar[6].item()
        temp_pose_ex.pose.position.y = Xbarbar[7].item()
        temp_pose_ex.pose.position.z = Xbarbar[8].item()
        temp_pose_safe.pose.position.x = Xbarbar[6].item()
        temp_pose_safe.pose.position.y = Xbarbar[7].item()
        temp_pose_safe.pose.position.z = Xbarbar[8].item()
        temp_pose_ex.header.stamp = rospy.Time.now()
        temp_pose_safe.header.stamp = rospy.Time.now()
        temp_pose_ex.header.seq = 1
        temp_pose_ex.header.frame_id = self.frame
        temp_pose_safe.header.frame_id = self.frame
        temp_pose_safe.header.seq = 1
        self.predTraj_explo.poses.append(temp_pose_ex)
        self.predTraj_safe.poses.append(temp_pose_safe)
        self.predTraj_explo.header.stamp = rospy.Time.now()
        self.predTraj_safe.header.stamp = rospy.Time.now()
        self.predTraj_explo.header.frame_id = self.frame
        self.predTraj_safe.header.frame_id = self.frame
        for ind in range(2,N):
            temp_pose_ex = PoseStamped()
            temp_pose_safe = PoseStamped()
            temp_pose_safe.pose.position.x = Xbarbar[ind*6].item()
            temp_pose_safe.pose.position.y = Xbarbar[ind*6+1].item()
            temp_pose_safe.pose.position.z = Xbarbar[ind*6+2].item()
            temp_pose_ex.pose.position.x = Xbarbar[(ind+N-1)*6].item()
            temp_pose_ex.pose.position.y = Xbarbar[(ind+N-1)*6+1].item()
            temp_pose_ex.pose.position.z = Xbarbar[(ind+N-1)*6+2].item()
            temp_pose_ex.header.stamp = rospy.Time.now()
            temp_pose_ex.header.frame_id = self.frame
            temp_pose_ex.header.seq = ind
            temp_pose_safe.header.stamp = rospy.Time.now()
            temp_pose_safe.header.frame_id = self.frame
            temp_pose_safe.header.seq = ind
            #print(temp_pose_ex.pose.position)
            self.predTraj_explo.poses.append(temp_pose_ex)
            self.predTraj_safe.poses.append(temp_pose_safe)
            #print(self.predTraj_explo.poses)
            self.predTraj_explo.header.stamp = rospy.Time.now()
            self.predTraj_safe.header.stamp = rospy.Time.now()
            self.predTraj_explo.header.frame_id = self.frame
            self.predTraj_safe.header.frame_id = self.frame
        #print(self.predTraj_explo.poses)
        self.traj_explo_pub.publish(self.predTraj_explo)
        self.traj_safe_pub.publish(self.predTraj_safe)


# Main function
def main():
    # initiate node
    rospy.init_node('mtMPC', anonymous=True)
    controller = mtMPC()
    # ROS loop rate
    rate = rospy.Rate(1/Ts)
    while not rospy.is_shutdown():
        rospy.logwarn(controller.local_pos.pose.position.z)
    # This is where we calculate the trajectory
        # Subscribers        # definition of the input variables, x0, Ac, bc
        tic = time.time()
        ticqp = time.time()
       # controller.polytope_sem.release() ## DA TOGLIERE
        controller.position_sem.acquire()
        controller.velocity_sem.acquire()
        controller.target_sem.acquire()
        controller.polytope_sem.acquire()
        sol = controller.solveqp()
        tocqp = time.time()
        # definition of the new setpoint
        if sol is None :
            # print("no solution")
            rospy.logwarn("no solution")
            sol = controller.solveqp()
            if sol is None:
                # print("no solution again")
                rospy.logwarn("no solution again")
                controller.setpoint.pose.position.x = controller.local_pos.pose.position.x
                controller.setpoint.pose.position.y = controller.local_pos.pose.position.y
                controller.setpoint.pose.position.z = controller.local_pos.pose.position.z
            else:
                controller.setpoint.pose.position.x = sol[0].item()
                controller.setpoint.pose.position.y = sol[1].item()
                controller.setpoint.pose.position.z = sol[2].item()
                # print("publishing trajectories")
                rospy.loginfo("publishing trajectories2")
        else:
            controller.setpoint.pose.position.x = sol[0].item()
            controller.setpoint.pose.position.y = sol[1].item()
            controller.setpoint.pose.position.z = sol[2].item()
            ##
            controller.Ac_old = controller.Ac
            controller.bc_old = controller.bc
            controller.plot_input(sol)
            # print("publishing trajectories")
            rospy.loginfo("publishing trajectories")
            rospy.loginfo(sol[0:3])
        # publishing the setpoint
        controller.setpoint.header.stamp = rospy.Time.now()
        controller.setpoint.header.frame_id = controller.frame
        if sqrt(pow(controller.local_pos.pose.position.x - controller.x_goal[0], 2) + \
             pow(controller.local_pos.pose.position.y - controller.x_goal[1], 2) + \
             pow(controller.local_pos.pose.position.z - controller.x_goal[2], 2)) < controller.toll_target:
            controller.setpoint.pose.position.x = controller.x_goal[0]
            controller.setpoint.pose.position.y = controller.x_goal[1]
            controller.setpoint.pose.position.z = controller.x_goal[2]
            rospy.loginfo("Target reached.")
        controller.setpt_pub.publish(controller.setpoint)
        controller.mark_goal_pub.publish(controller.x_goal_mark)
        toc = time.time()
        # print("Time for qp: ")
        # print(tocqp-ticqp)
        # print("Elapsed time: ")
        # print(toc-tic)
        rospy.loginfo("Time for qp: %f - Elapsed time: %f", tocqp-ticqp, toc-tic)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
