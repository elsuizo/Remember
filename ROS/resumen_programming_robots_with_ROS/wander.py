#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)

g_range_ahead = 1 # algun valor para comenzar

# el topic al que nos subscribimos
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
# el mensaje que vamos a publicar
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
# solo hay dos estados adelante y atras
driving_forward = True
# este seria el rate con que se actualiza el lazo(son 10Hz en promedio)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        # NOTE(elsuizo:2020-11-18): aca mira si estamos en el rango de distancia
        # o si ya paso el tiempo que pusimos
        if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(5)
    else:
        if rospy.Time.now() > state_change_time:
            driving_forward = True # ya hemos recorrido mucho tiempo de ir para adelante!!!
            state_change_time = rospy.Time.now() + rospy.Duration(30)
    # TODO(elsuizo:2020-11-18): no se si hace falta crear este objeto en cada iteracion!!!
    twist = Twist()
    if driving_forward:
        twist.linear.x = 1
    else:
        twist.angular.z = 1

    cmd_vel_pub.publish(twist)

    rate.sleep()
