#!/usr/bin/env python3

"""
auto_exposure_control.py subscribes to a ROS image topic and performs
histogram based automatic exposure control based on the paper
"Automatic Camera Exposure Control", N. Nourani-Vatani, J. Roberts
"""
#import roslib
import math
import numpy as np
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client

# I term for PI controller
err_i = 0
i_num = 0
c_exp_value = 3000

def get_exposure(dyn_client):
    values = dyn_client.get_configuration()
    return values['exposure_value']

def set_exposure(dyn_client, exposure):
    params = {'exposure_value' : exposure}
    config = dyn_client.update_configuration(params)

def target_exposure_callback(msg):
    global c_exp_value
    c_exp_value = msg.data

def image_callback(image, args):
    global err_i, i_num, c_exp_value

    # skip 4 frames out of 5
    # print(i_num)
    # i_num+=1
    # if (i_num%5 != 0):
    #     return


    bridge = args['cv_bridge']
    # dyn_client = args['dyn_client']
    cv_image = bridge.imgmsg_to_cv2(image,
                                    desired_encoding = "bgr8")
    
    (rows, cols, channels) = cv_image.shape
    if (channels == 3):
        brightness_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[:,:,2]
    else:
        brightness_image = cv_image

    crop_size = 120
    brightness_image = brightness_image[crop_size:rows-crop_size, crop_size:cols-crop_size]
    (rows, cols) = brightness_image.shape
    
    hist = cv2.calcHist([brightness_image],[0],None,[5],[0,256])
    
    mean_sample_value = 0
    for i in range(len(hist)):
        mean_sample_value += hist[i]*(i+1)
        
    mean_sample_value /= (rows*cols)
    
    #focus_region = brightness_image[rows/2-10:rows/2+10, cols/2-10:cols/2+10]
    #brightness_value = numpy.mean(focus_region)

    # Middle value MSV is 2.5, range is 0-5
    # Note: You may need to retune the PI gains if you change this
    desired_msv = 2.5
    # Gains
    k_p = 0.01
    err_p = desired_msv-mean_sample_value
    
    # Don't change exposure if we're close enough. Changing too often slows
    # down the data rate of the camera.
    if abs(err_p) > 0.1:
        # set_exposure(dyn_client, get_exposure(dyn_client)+k_p*err_p+k_i*err_i)
        exp_time_msg = Int16()
        exp_time_msg.data = int(max(min(c_exp_value+c_exp_value*k_p*err_p, args['max_exp']), args['min_exp']))
        args['pub'].publish(exp_time_msg)
        args['d_pub'].publish(exp_time_msg)

        if exp_time_msg.data == args['max_exp']:
            rospy.loginfo("INITIAL AUTO EXPOSURE COMPUTATION REACHED MAX EXPOSURE!")
            rospy.signal_shutdown("initial auto exposure computation reached max exposure!")
        elif exp_time_msg.data == args['min_exp']:
            rospy.loginfo("INITIAL AUTO EXPOSURE COMPUTATION REACHED MIN EXPOSURE!")
            rospy.signal_shutdown("initial auto exposure computation reached min exposure!")
    else:
        rospy.loginfo("INITIAL AUTO EXPOSURE COMPUTATION IS COMPLETED!")
        rospy.signal_shutdown("initial auto exposure computation is completed!")

def main(args):
    rospy.init_node('auto_exposure_control')

    bridge = CvBridge()
    wait_down_cam = rospy.get_param('~wait_down_camera', False)
    max_exp = rospy.get_param("~max_exp", 10000)
    min_exp = rospy.get_param("~min_exp", 10000)
    # dyn_client = dynamic_reconfigure.client.Client(camera_name)

    params = {'auto_exposure' : False}
    # config = dyn_client.update_configuration(params)


    rospy.loginfo("WAITING FOR FRONT CAMERA ....")
    rospy.wait_for_message("/camera_front/target_exposure_time", Int16)

    if (wait_down_cam):
        rospy.loginfo("WAITING FOR DOWN CAMERA ....")
        rospy.wait_for_message("/camera_down/target_exposure_time", Int16)
    else:
        rospy.logwarn("SKIPPING DOWN CAMERA ...")

    rospy.loginfo("STARTING EXPOSURE CALIBRATION ...")

    exp_pub = rospy.Publisher("/camera_front" + '/exposure_time', Int16, queue_size=1)
    down_exp_pub = rospy.Publisher("/camera_down" + '/exposure_time', Int16, queue_size=1)
    
    args = {}
    args['cv_bridge'] = bridge
    # args['dyn_client'] = dyn_client
    args['pub'] = exp_pub
    args['d_pub'] = down_exp_pub
    args['max_exp'] = max_exp
    args['min_exp'] = min_exp

    img_sub=rospy.Subscriber("/camera_front" + '/fisheye1/image_raw', Image, image_callback, args)
    exp_sub=rospy.Subscriber("/camera_front" + '/target_exposure_time', Int16, target_exposure_callback)

    rospy.spin()
    
if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
