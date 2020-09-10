#!/usr/bin/env python

# import ROS Libraries
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

# import Matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math

# global variable decalration 
global robot_pose,qd,err,v,w,t
robot_pose = Pose2D()
qd = Pose2D()
err = Pose2D()
v = Twist()
w = Twist()
t = 0.0

xl = []
yl = [] 
xd = []
yd = [] 
xe = [] 
ye = []
th = []  
vr = []
wr = []  
s = []

# callback functions to recive robot cooridnates, velocities
def pose_callback(msg):
    global robot_pose,v,w
    
    robot_pose.x = msg.pose.pose.position.x
    robot_pose.y = msg.pose.pose.position.y
    v = msg.twist.twist.linear.x
    w = msg.twist.twist.angular.z
    
# callback functions to get desired cooridnates
def desired_callback(msg):
    global qd,t
    
    qd.x = msg.x
    qd.y = msg.y
    t = t + 0.1

# callback functions to get Tracking errors
def errors_callback(msg):
    global err
    
    err.x = msg.x
    err.y = msg.y
    err.theta = msg.theta 


# animate functions to plot data
def animate1(i):
       
    xd.append(float(qd.x))
    yd.append(float(qd.y))
    ax1.plot(xd,yd, 'r',linestyle='dashed',label="desired trajectory",lw=2)  

    xl.append(float(robot_pose.x))
    yl.append(float(robot_pose.y))
    ax1.plot(xl,yl, 'k',label="robot trajectory",lw=2)  
    
    ax1.grid(True)
    ax1.legend()
    #ax1.axis("equal")

def animate2(i):

    xe.append(float(err.x))
    ye.append(float(err.y))
    th.append(float(err.theta))
    s.append(float(t))

    ax2.plot(s ,xe,'k',label=r"$x_e$" ,lw=2)  
    ax2.plot(s ,ye,'r',label=r"$y_e$" ,lw=2)  
    ax2.plot(s ,th,'b',label=r'$\theta_e$' ,lw=2) 

    vr.append(float(v))
    wr.append(float(w))

    ax3.plot(s ,vr, 'k',label=r"$v_r$" ,lw=2)  
    ax3.plot(s ,wr, 'b',label=r"$w_r$" ,lw=2) 

    ax2.grid(True)
    ax2.legend()

    ax3.grid(True)
    ax3.legend()
    
if __name__ == '__main__':

    # initialization of ROS node
    rospy.init_node('back_cir', anonymous=True) #make node 
    sub = rospy.Subscriber("/odom", Odometry, pose_callback, queue_size=1000)
    sub1 = rospy.Subscriber("/desired_traj_pub", Pose2D, desired_callback, queue_size=1000)
    sub2 = rospy.Subscriber("/errors_pub", Pose2D, errors_callback, queue_size=1000)

    fig1 = plt.figure()
    fig2 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax2 = fig2.add_subplot(211)
    ax3 = fig2.add_subplot(212,sharex=ax2)

    ax1.set_xlim([-2.5,0.5])
    ax1.set_ylim([-1.5,1.5]) 
    ax2.set_xlim(xmin=0,xmax=40)

    ax3.set_xlabel("Time(s)")
    ax3.set_ylabel("velocities")
    ax2.set_ylabel("Tracking Errors")



    ani1 = animation.FuncAnimation(fig1, animate1, interval=100)
    ani2 = animation.FuncAnimation(fig2, animate2, interval=100)
    plt.show()
    rospy.spin()






