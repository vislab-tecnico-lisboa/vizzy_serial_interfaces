#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy


# Brings in the SimpleActionClient
import actionlib

import woz_dialog_msgs.msg
from vizzy_msgs.srv import BatteryState
from vizzy_msgs.srv import BatteryStateResponse

def Kokam_battery_client():
    rospy.wait_for_service('kokam_battery_state')
    try:
        add_two_ints = rospy.ServiceProxy('kokam_battery_state', BatteryState)
        resp1 = add_two_ints()
	#print (type(resp1.battery_state))
        return resp1
    except rospy.ServiceException, e:
        print ("Service call failed: "+ str(e))

def test_action():


    client = actionlib.SimpleActionClient('/nuance_speech_tts', woz_dialog_msgs.msg.SpeechAction)

    rate = rospy.Rate(1/15.0) # 10hz
    while not rospy.is_shutdown():
        my_res = Kokam_battery_client()
	if my_res.battery_state == BatteryStateResponse.LOW_BATTERY:
	    client.wait_for_server()

    	    goal = woz_dialog_msgs.msg.SpeechGoal(language="pt_PT", voice="Joana", message="Liga-me a ficha que preciso de energia")

    	    client.send_goal(goal)

    	    client.wait_for_result()

        rate.sleep()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.

        rospy.init_node('speech_client_py')
        result = test_action()
	#Kokam_battery_client()
        #print("Result:", result.success)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)



