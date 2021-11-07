#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
from numpy import array, dot
from numpy import linalg as LA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Vector3Stamped, Point
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker

from threading import Semaphore
import time
import math
from math import sqrt
import copy


class computeTargetExplor():
    def __init__(self):
        # Tf subriber
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribers
        rospy.Subscriber('/mtMPC/target',Point,self.callback_target)
        self.x_goal_global = PointStamped()
        self.target_sem = Semaphore(0)
        self.x_goal_local = PointStamped()

        rospy.Subscriber('/scan',LaserScan,self.callback_scan)
        self.scan = LaserScan()
        self.vector_scan = np.array([])
        self.scan_sem = Semaphore(0)

        rospy.Subscriber('/dji_sdk/local_position',PointStamped,self.callback_loc_pos)
        #rospy.Subscriber('/uav0/mavros/local_position/pose',PoseStamped,self.callback_loc_pos)
        self.position_glob = PoseStamped()
        self.position_sem = Semaphore(0)

        # Old variables
        self.x_goal_old_global = PointStamped()#np.array([[0],[0],[5]])
        self.x_goal_old_local =  PointStamped()#np.array([[0],[0],[5]])
        self.indiceBestDirectionOld = 0
        self.usePreviousTarget = False

        # Publisher
        self.setpt_ex_pub = rospy.Publisher('/mtMPC/temp_target', Point, queue_size=1)
        self.setpoint = Point()
        self.target_explor_loc = PointStamped()

        
        #### MARKER FOR TEMP TARGET IN GLOBAL FRAME
        self.temp_target_mark = Marker()
        self.temp_target_mark.header.frame_id = 'odom'
        self.temp_target_mark.color.a = 1
        self.temp_target_mark.color.r = 0
        self.temp_target_mark.color.g = 0
        self.temp_target_mark.color.b = 1
        self.temp_target_mark.type = 3
        self.temp_target_mark.scale.x = 1
        self.temp_target_mark.scale.y = 1
        self.temp_target_mark.scale.z = 1
        self.mark_ttarg_pub = rospy.Publisher('/mtMPC/temp_targ_marker',Marker,queue_size=1)
        
        #### MARKER FOR TEMP TARGET IN LOCAL FRAME       
        self.temp_target_local_mark = Marker()
        self.temp_target_local_mark.header.frame_id = 'base_scan'
        self.temp_target_local_mark.color.a = 1
        self.temp_target_local_mark.color.r = 1
        self.temp_target_local_mark.color.g = 0
        self.temp_target_local_mark.color.b = 1
        self.temp_target_local_mark.type = 1
        self.temp_target_local_mark.scale.x = 1
        self.temp_target_local_mark.scale.y = 1
        self.temp_target_local_mark.scale.z = 1
        self.mark_ttarg_local_pub = rospy.Publisher('/mtMPC/temp_targ_local_marker',Marker,queue_size=1)


    def callback_loc_pos(self,msg):
        self.position_glob.header = msg.header
        self.position_glob.pose.position = msg.point
        self.position_sem.release()

    def callback_target(self,msg):
        self.x_goal_global.point = msg
        self.target_sem.release()

    def callback_scan(self,msg):
        self.scan = msg
        self.vector_scan = np.array(msg.ranges)
        self.vector_scan.reshape((np.size(self.vector_scan,0),1))
        self.scan_sem.release()

    def compute_target(self):
        # Scan Data
        angle_min = self.scan.angle_min
        angle_max = self.scan.angle_max
        ranges = self.vector_scan
        angle_increment = self.scan.angle_increment
        range_max = self.scan.range_max
        
        # Global, local and target position
        position = np.array([[0],[0],[0]])
        globpos = np.array([ [self.position_glob.pose.position.x],
        [self.position_glob.pose.position.y],
        [self.position_glob.pose.position.z]])
        x_goal_local = np.array([ 
        [self.x_goal_local.point.x],
        [self.x_goal_local.point.y],
        [self.x_goal_local.point.z]
        ])
        x_goal_global = np.array([ 
        [self.x_goal_global.point.x],
        [self.x_goal_global.point.y],
        [self.x_goal_global.point.z]
        ])
        x_goal_old_global = np.array([
        [self.x_goal_old_global.point.x],
        [self.x_goal_old_global.point.y],
        [self.x_goal_old_global.point.z]])
        x_goal_old_local = np.array([
        [self.x_goal_old_local.point.x],
        [self.x_goal_old_local.point.y],
        [self.x_goal_old_local.point.z]])
        
        # computation of the angle of the goal wrt to lidar x-axis
        p0 = position[0:2]
        p1 = np.array([position[0]+1,position[1]])
        p2 = x_goal_local[0:2]
        n1 = (p2-p0)/LA.norm(p2-p0,2)
        n2 = (p1-p0)/LA.norm(p1-p0,2)
        angleGoal = np.arctan2(n1[0]*n2[1]-n1[1]*n2[0], n1[0]*n2[0]+n1[1]*n2[1]) * 180 / np.pi
        
        # the angle of the goal is expressed in a 0-360 range counterclockwise
        if angleGoal<=0:
            angleGoal = np.absolute(angleGoal)
        else:
            angleGoal = 360-angleGoal        
        
        # the angles of the ranges vector are listed
        angles = np.arange(angle_min,angle_max,angle_increment)*180/math.pi
        for i in range(angles.size):
            if angles[i]<0:
                angles[i] = angles[i]+360    

        #list of indices
        indices = np.arange(0,ranges.size).reshape(ranges.size,1)

        # definitioon of the distance from target
        distanceFromTarget = angles-angleGoal
        distanceFromTargetNorm = copy.copy(distanceFromTarget)
        distanceFromTargetNorm[np.where(distanceFromTargetNorm >= 180)]=distanceFromTargetNorm[np.where(distanceFromTargetNorm >= 180)]-360
        distanceFromTargetNorm[np.where(distanceFromTargetNorm <= -180)]=distanceFromTargetNorm[np.where(distanceFromTargetNorm <= -180)]+360

        angles = np.expand_dims(angles,1)
        distanceFromTargetNorm = np.expand_dims(distanceFromTargetNorm,1)
        ranges = np.expand_dims(ranges,1)
        
        matrix = np.concatenate((angles,distanceFromTargetNorm,ranges,indices),axis = 1)
        

        noObs = np.array(np.where(ranges == np.inf))

        if noObs.size == 0:
            maxVal = np.argmax(ranges,axis=0)
            noObs = np.array(np.where(ranges == ranges[maxVal]))
            
        matrixnoObs = matrix[noObs[0],:]
        ind_temp1 = np.min(np.absolute(matrixnoObs[:,1]))
        ind_temp = np.array(np.where(np.absolute(matrixnoObs[:,1])==ind_temp1))
        indiceBestDirection = int(matrixnoObs[ind_temp[0][0],3])
        angleBestDirection = matrixnoObs[ind_temp[0][0],0]
        # rospy.logwarn('angles')
        # rospy.logwarn(angles)
        # rospy.logwarn('distance targ')
        # rospy.logwarn(distanceFromTargetNorm[0:900])
        # rospy.logwarn('angles best dir')
        # rospy.logwarn(angleBestDirection)

        if LA.norm(globpos[0:2].reshape((2,1))-x_goal_old_global[0:2].reshape((2,1)))<range_max*0.4 \
        or LA.norm(x_goal_old_global[0:2].reshape((2,1))-x_goal_global[0:2].reshape((2,1)))<0.1:

            # if ranges[indiceBestDirection]==np.inf and \
            # ranges[np.mod(indiceBestDirection+1,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection-1,ranges.size)]==np.inf) \
            # and ranges[np.mod(indiceBestDirection+2,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection-2,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection+3,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection-3,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection+4,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection-4,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection+5,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection-5,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection+6,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection-6,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection+7,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+8,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+9,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+10,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+11,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+12,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+13,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+14,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+15,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+16,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection+17,ranges.size)]==np.inf:
            if ranges[np.mod(indiceBestDirection+20,ranges.size)]==np.inf and not(ranges[np.mod(indiceBestDirection-20,ranges.size)]==np.inf):
                rospy.logwarn('ostacolo a dx')
                rospy.logwarn(angleBestDirection)
                angleBestDirection=angleBestDirection+25*angle_increment*180/math.pi
                rospy.logwarn(angleBestDirection)
            elif ranges[np.mod(indiceBestDirection-20,ranges.size)]==np.inf and not(ranges[np.mod(indiceBestDirection+20,ranges.size)]==np.inf):
            # elif ranges[indiceBestDirection]==np.inf and \
            # ranges[np.mod(indiceBestDirection-1,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection+1,ranges.size)]==np.inf) \
            # and ranges[np.mod(indiceBestDirection-2,ranges.size)]==np.inf  or not(ranges[np.mod(indiceBestDirection+2,ranges.size)]==np.inf) \
            # and ranges[np.mod(indiceBestDirection-3,ranges.size)]==np.inf  or not(ranges[np.mod(indiceBestDirection+3,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection-4,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection+4,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection-5,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection+5,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection-6,ranges.size)]==np.inf or not(ranges[np.mod(indiceBestDirection+6,ranges.size)]==np.inf)\
            # and ranges[np.mod(indiceBestDirection-7,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-8,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-9,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-10,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-11,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-12,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-13,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-14,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-15,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-16,ranges.size)]==np.inf \
            # and ranges[np.mod(indiceBestDirection-17,ranges.size)]==np.inf:
                rospy.logwarn('ostacolo a sx')
                rospy.logwarn(angleBestDirection)
                angleBestDirection=angleBestDirection-25*angle_increment*180/math.pi
                rospy.logwarn(angleBestDirection)
            else:
                rospy.logwarn('ostacolo ne a destra ne a sinistra')
                rospy.loginfo('AngleBestDirection')
                rospy.loginfo(angleBestDirection)
                rospy.loginfo('IndexBestdirection')
                rospy.loginfo(indiceBestDirection)
                rospy.loginfo('40 valori attorno')
                if np.mod(indiceBestDirection-20,ranges.size)<np.mod(indiceBestDirection+20,ranges.size):
                    rospy.loginfo(ranges[np.mod(indiceBestDirection-20,ranges.size):np.mod(indiceBestDirection+20,ranges.size)])
                else:
                    rospy.loginfo(ranges[np.mod(indiceBestDirection-20,ranges.size):ranges.size])
                    rospy.loginfo(ranges[0:np.mod(indiceBestDirection+20,ranges.size)])
            target_explor_localx = position[0]+range_max*0.9*np.cos(np.radians(angleBestDirection))
            target_explor_localy = position[1]+range_max*0.9*np.sin(np.radians(angleBestDirection))
            target_explor_localz = x_goal_global[2]-globpos[2]
            self.target_explor_loc.point.x = target_explor_localx
            self.target_explor_loc.point.y = target_explor_localy
            self.target_explor_loc.point.z = target_explor_localz
            #rospy.loginfo('dentro')
            
            if ranges[indiceBestDirection]==np.inf and \
            ranges[np.mod(indiceBestDirection+1,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-1,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+2,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-2,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+3,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-3,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+4,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-4,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+5,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-5,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+6,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-6,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+7,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-7,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+8,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-8,ranges.size)]==np.inf:
                self.target_explor_loc.point.x = self.x_goal_local.point.x
                self.target_explor_loc.point.y = self.x_goal_local.point.y
                self.target_explor_loc.point.z = self.x_goal_local.point.z
                rospy.loginfo('TARGET GLOBAL 1')
            
            self.usePreviousTarget=False 
            self.indiceBestDirectionOld = indiceBestDirection
            rospy.loginfo('Calcolo nuovo target')



        else:
            if ranges[indiceBestDirection]==np.inf and \
            ranges[np.mod(indiceBestDirection+1,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-1,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+2,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-2,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+3,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-3,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+4,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-4,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+5,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-5,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+6,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-6,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+7,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-7,ranges.size)]==np.inf \
            and ranges[np.mod(indiceBestDirection+8,ranges.size)]==np.inf and ranges[np.mod(indiceBestDirection-8,ranges.size)]==np.inf:
                self.target_explor_loc.point.x = self.x_goal_local.point.x
                self.target_explor_loc.point.y = self.x_goal_local.point.y
                self.target_explor_loc.point.z = self.x_goal_local.point.z
                self.indiceBestDirectionOld = indiceBestDirection
                rospy.loginfo('TARGET GLOBAL 2')
            else:
                self.target_explor_loc.point.x = x_goal_old_local[0]
                self.target_explor_loc.point.y = x_goal_old_local[1]
                self.target_explor_loc.point.z = x_goal_old_local[2]
                rospy.loginfo('Il target e quello vecchio')
                #rospy.loginfo('uso target old')
                self.usePreviousTarget=True 
        return 


# Main function
def main():
    # initiate node
    rospy.init_node('temp_target_generator', anonymous=True)
    TargetGen = computeTargetExplor()
    # ROS loop rate
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # Acquire all the semaphore from callbacks
        TargetGen.target_sem.acquire()
        TargetGen.scan_sem.acquire()
        TargetGen.position_sem.acquire()
        # convert the target in local coordinates
        try:
            transBase2map = TargetGen.tfBuffer.lookup_transform('base_scan', 'odom', rospy.Time())
            x_goal_old_local = tf2_geometry_msgs.do_transform_point(TargetGen.x_goal_old_global,transBase2map)
            TargetGen.x_goal_old_local=x_goal_old_local
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        try:
            transBase2map = TargetGen.tfBuffer.lookup_transform('base_scan', 'odom', rospy.Time())
            x_goal_local = tf2_geometry_msgs.do_transform_point(TargetGen.x_goal_global,transBase2map)
            TargetGen.x_goal_local=x_goal_local
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        # Compute local target exploration
        TargetGen.compute_target()
        # IF use new temp target
        try:
            transMap2base = TargetGen.tfBuffer.lookup_transform('odom', 'base_scan', rospy.Time())
            target_explor_glob = tf2_geometry_msgs.do_transform_point(TargetGen.target_explor_loc,transMap2base)
            target_explor_glob.point.z = TargetGen.x_goal_global.point.z
            TargetGen.x_goal_old_global= target_explor_glob
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        
        TargetGen.setpoint = target_explor_glob.point

        # Publish the set point
        TargetGen.setpt_ex_pub.publish(TargetGen.setpoint)
        # update and publish the set point markers
        TargetGen.temp_target_mark.pose.position=TargetGen.setpoint
        TargetGen.mark_ttarg_pub.publish(TargetGen.temp_target_mark)
        TargetGen.temp_target_local_mark.pose.position=TargetGen.target_explor_loc.point
        TargetGen.mark_ttarg_local_pub.publish(TargetGen.temp_target_local_mark)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
