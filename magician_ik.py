#!/usr/bin/python

import ikpy
import rospy
import numpy as np
from ikpy import plot_utils
from std_msgs.msg import Float64


my_chain = ikpy.chain.Chain.from_urdf_file("/home/general_ws/src/description/urdf/magician.urdf")
# print my_chain

x = 0.0
y = -0.206
z = 0.18


pub = []
coord = [0,0,0]
buff_coord = [0,0,0]
while not rospy.is_shutdown():
    rospy.init_node('magician_talker', anonymous=True)
    for i in range(0,7) :
        pub.append(rospy.Publisher('/magician/joint'+str(i)+'_position_controller/command', Float64, queue_size=10))
    rate = rospy.Rate(10) # 10hz
    for i in range (0,3):
        buff_coord[i] = coord[i]
    split = raw_input('coordenadas: ').split(",")
    for i in range(0,3):
        split[i] = float(split[i])
    buffx = x
    buffy = y
    buffz = z
    x = x + split[0]
    y = y + split[1]
    z = z + split[2]

    coord[0] = x
    coord[1] = y
    coord[2] = z

    for a in range (0,3):
        if a == 0:
            if coord[a]>buff_coord[a]:
                for count in np.arange(buff_coord[a],coord[a]+0.006,0.006):
                    joint_values = my_chain.inverse_kinematics([[1, 0, 0, count],
                                                                [0, 1, 0, y],
                                                                [0, 0, 1, z],
                                                                [0, 0, 0, 1]])
                    for i in range (0,7):
                        pub[i].publish(float(joint_values[i]))
                coord[a]=count
            else:
                for count in np.arange(buff_coord[a],coord[a]-0.006,-0.006):
                    joint_values = my_chain.inverse_kinematics([[1, 0, 0, count],
                                                                [0, 1, 0, y],
                                                                [0, 0, 1, z],
                                                                [0, 0, 0, 1]])
                    for i in range (0,7):
                        pub[i].publish(float(joint_values[i]))
                coord[a]=count
        elif a == 1:
            if coord[a]>buff_coord[a]:
                for count in np.arange(buff_coord[a],coord[a]+0.006,0.006):
                    joint_values = my_chain.inverse_kinematics([[1, 0, 0, x],
                                                                [0, 1, 0, count],
                                                                [0, 0, 1, z],
                                                                [0, 0, 0, 1]])
                    for i in range (0,7):
                        pub[i].publish(float(joint_values[i]))
                coord[a]=count
            else:
                for count in np.arange(buff_coord[a],coord[a]-0.006,-0.006):
                    joint_values = my_chain.inverse_kinematics([[1, 0, 0, x],
                                                                [0, 1, 0, count],
                                                                [0, 0, 1, z],
                                                                [0, 0, 0, 1]])
                    for i in range (0,7):
                        pub[i].publish(float(joint_values[i]))
                coord[a]=count
        elif a == 2:
            if coord[a]>buff_coord[a]:
                for count in np.arange(buff_coord[a],coord[a]+0.006,0.006):
                    joint_values = my_chain.inverse_kinematics([[1, 0, 0, x],
                                                                [0, 1, 0, y],
                                                                [0, 0, 1, count],
                                                                [0, 0, 0, 1]])
                    for i in range (0,7):
                        pub[i].publish(float(joint_values[i]))
                coord[a]=count
            else:
                for count in np.arange(buff_coord[a],coord[a]-0.006,-0.006):
                    joint_values = my_chain.inverse_kinematics([[1, 0, 0, x],
                                                                [0, 1, 0, y],
                                                                [0, 0, 1, count],
                                                                [0, 0, 0, 1]])
                    for i in range (0,7):
                        pub[i].publish(float(joint_values[i]))
                coord[a]=count



    # joint_values = my_chain.inverse_kinematics([[1, 0, 0, x],
    #                                             [0, 1, 0, y],
    #                                             [0, 0, 1, z],
    #                                             [0, 0, 0, 1]])
    # print joint_values

    # for i in range (0,7):
	# 			pub[i].publish(float(joint_values[i]))