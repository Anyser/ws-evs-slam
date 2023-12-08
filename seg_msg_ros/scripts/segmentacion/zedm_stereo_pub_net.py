#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge, CvBridgeError

import pyzed.sl as sl
from seg_msg_ros.msg import msg_mask
import message_filters

import PIL.Image as Im
import numpy as np

from modelos.Deeplabv3_resnet50 import SegmentationModel
segmentation_model = SegmentationModel()

class SegmentationMask:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.loginfo("Segmentador de objetos")
        self._rate = rospy.get_param('~publish_rate', 50)
        self._use_mask = True

        self.zed=sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.VGA #VGA-HD720-HD1080-HD1200-HD2K
        init.coordinate_units = sl.UNIT.METER
        init.camera_fps=30
        err = self.zed.open(init)
    
        if err != sl.ERROR_CODE.SUCCESS :
            print(repr(err))
            self.zed.close()
            exit()
        
        self.mat_left = sl.Mat()
        self.mat_right = sl.Mat()
        self.runtime = sl.RuntimeParameters()

        self.pub_img_left=rospy.Publisher("img/left",SensorImage,queue_size=1)
        self.pub_img_right=rospy.Publisher("img/right",SensorImage,queue_size=1)
        self.pub_img_mask=rospy.Publisher("img/mask",SensorImage,queue_size=1)
      
    def resize_image(self,img, scale_factor):
        width = int(img.shape[1] * scale_factor)
        height = int(img.shape[0] * scale_factor)
        dim=(width,height)
        resized=cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
        return resized
    
    def resize_np_image(self, np_image, size):
        return cv2.resize(np_image, size, interpolation=cv2.INTER_NEAREST)
    
    def process_mask(self,mask):

        kernel_dilate = np.ones((9, 9), np.uint8)
        dilated_mask = cv2.dilate(mask, kernel_dilate, iterations=2)
        kernel_erode = np.ones((3, 3), np.uint8)  # size kernel 
        eroded_mask = cv2.erode(dilated_mask, kernel_erode, iterations=1)
        
        return eroded_mask
    
    def publish_image(self,img,timestamp,encoding):
            
            if img is not None:
                msg = SensorImage()
                try:
                #msg.header.frame_id="dataset"
                    msg.header.stamp=timestamp
                    msg.height=img.shape[0]
                    msg.width=img.shape[1]
                    msg.encoding=encoding
                    if encoding == 'rgb8':
                        img = img[:, :, :3]
                        msg.step=img.shape[1]*3
                        msg.data = self.bridge.cv2_to_imgmsg(img, encoding).data
                    elif encoding == 'mono8':
                        depth_normalized = cv2.normalize(img, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                        depth_scaled = (255 * depth_normalized).astype(np.uint8)
                        img = depth_scaled
                        msg.step = img.shape[1]  # Ancho, ya que ahora es un solo canal de 8 bits
                        msg.data = self.bridge.cv2_to_imgmsg(img, encoding).data
                    elif encoding=='32FC1':
                        msg.step=img.shape[1]*4
                        msg.data = self.bridge.cv2_to_imgmsg(img, "passthrough").data
                except CvBridgeError as e:
                    rospy.loginfo("El error esta en el publicador")
                    rospy.loginfo(e)

                return msg
    



    def publish_image_m(self, img, timestamp):
        if img is not None:
            msg = SensorImage()
            #msg.header.frame_id="dataset"
            msg.header.stamp = rospy.Time.from_sec(timestamp)
            msg.height = img.shape[0]
            msg.width = img.shape[1]
            msg.encoding = "mono8"
            msg.is_bigendian = False
            msg.step = msg.width
            msg.data = img.tobytes()
        else:
            rospy.loginfo("invalid image")
            msg     = None
    
        return msg
    

    def publish_mask(self,mask, timestamp):
        mask_msg=self.create_msg(mask, timestamp)
        self.pub_mask.publish(mask_msg)


    def create_msg(self,mask,timestamp):
        msg=msg_mask()
        msg.header.stamp=rospy.Time.from_sec(timestamp)
        msg.data=mask.flatten().tolist()
        return msg
    



    def publish_sync_data(self):
        scale_factor=0.75
        while not rospy.is_shutdown():
            try:
                if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS :
                    timestamp = rospy.Time.now()
                    self.zed.retrieve_image(self.mat_left, sl.VIEW.LEFT)
                    self.zed.retrieve_image(self.mat_right, sl.VIEW.RIGHT)
                    img_left=self.mat_left.get_data()
                    if img_left.shape[2] == 4:
                        img_left = img_left[:, :, :3]
                    self.pub_img_left.publish(self.publish_image(img_left, timestamp, "rgb8"))
                    
                    img_right=self.mat_right.get_data()
                    if img_right.shape[2] == 4:
                        img_right = img_right[:, :, :3]
                    self.pub_img_right.publish(self.publish_image(img_right, timestamp, "rgb8"))


                    if self._use_mask:                    
                        resized_im_rgb =self.resize_image(img_left, scale_factor)
                        segmentation_mask = segmentation_model.infer(resized_im_rgb)
                        npimg=(segmentation_mask*58).astype(np.uint8)
                        _, npimg=cv2.threshold(npimg,0,255,cv2.THRESH_BINARY)    
                        npimg=self.resize_np_image(npimg,(img_left.shape[1], img_left.shape[0]))
                        npimg=self.process_mask(npimg)
                        self.pub_img_mask.publish(self.publish_image(npimg, timestamp, "mono8"))

                    rospy.loginfo("información [LEFT-RIGHT-MASK]")
                    rospy.Rate(self._rate).sleep()
                
                    if  rospy.get_param("shutdown", False):
                        rospy.signal_shutdown("Closing the node")
                        self.zed.close()
                        break

            except rospy.ROSInterruptException:
                pass


        
if __name__ == "__main__":
    rospy.loginfo("Segmentador de objetos")
    rospy.init_node("NodeSeg")
    Node_publisher= SegmentationMask()
    Node_publisher.publish_sync_data()
    rospy.spin()