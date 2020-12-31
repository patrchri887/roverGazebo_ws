#!/usr/bin/env python
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import matplotlib.pyplot as plt
import numpy as np

def gradient(img):
    # grayscale the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gaussian blur of image with a 5x5 kernel
    gauss = cv2.GaussianBlur(gray,(5,5),0)
    # Return the canny of the image
    return cv2.Canny(gauss,100,200)

def region_of_interest(img):
    height = img.shape[0]
    width = img.shape[1]
    polygons = np.array([
    [(0,height), (0,450), (450,450), (width,450), (width,height)]
    ])
    #polygon_asph = np.array([
    #[(0,600), (211, 500), (572,487), (793, 617),(width, height), (0, height)]
    #])  
    #clear detections from the asphalt
    #mask_asph = np.zeros_like(img)
    mask_bg = np.zeros_like(img)
    #cv2.imshow("Crop selection", cv2.fillPoly(mask_bg, polygons, 255))
    masked_image = cv2.bitwise_and(img,cv2.fillPoly(mask_bg, polygons, 255))
    #square = cv2.fillPoly(mask_asph, polygon_asph, 255)
    #square = cv2.subtract(255, square)
    #return cv2.bitwise_and(masked_image,square, 255) 
    return masked_image

def make_coordinates(img, line_parameters):
    slope, intercept = line_parameters
    y1 = img.shape[0]
    y2 = int(y1*0.5)
    x1 = int((y1-intercept)/slope)
    x2 = int((y2-intercept)/slope)
    return np.array([x1,y1,x2,y2])

def average_slope_intercep(img,lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
        parameters = np.polyfit((x1,x2),(y1,y2),1)
        slope = parameters[0]
        intercept = parameters[1]

        if slope < 0:
            left_fit.append((slope,intercept))
        else:
            right_fit.append((slope,intercept))

    left_fit_average=np.average(left_fit,axis=0)
    right_fit_average=np.average(right_fit,axis=0)
    left_line=make_coordinates(img,left_fit_average)
    right_line=make_coordinates(img,right_fit_average)

    return np.array([left_line,right_line])


def display_lines(img,lines):
    line_image = np.zeros_like(img)
    if lines is not None:
	for line in lines:
		x1, y1, x2, y2 = line.reshape(4)
		cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
    return line_image

def image_callback(msg):
    print("Received an image!")
    # Instantiate CvBridge
    bridge = CvBridge()
    try:
        # Convert your ROS Image message to OpenCV2
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:

	#Copy of the original frame
	frame_copy = np.copy(frame)

	# canny of image
	canny_frame = gradient(frame_copy)

	# apply mask in region of interest
	cropped_image = region_of_interest(canny_frame)

	# hough transformation of image
	lines = cv2.HoughLinesP(cropped_image,2,np.pi/180,150,np.array([]),minLineLength=40,maxLineGap=5)
	averaged_lines = average_slope_intercep(frame_copy,lines)
        print(averaged_lines)	
	line_image = display_lines(frame_copy,averaged_lines)
	combo_image = cv2.addWeighted(frame_copy,0.8,line_image,1,1)
	
	#Show manipulated image feed
        cv2.imshow("Result feed", combo_image)
	#plt.imshow(canny_frame)
	cv2.waitKey(1)
	#plt.show()


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera1/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
