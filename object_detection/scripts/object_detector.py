#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import rospy
import rospkg
from queue import Queue
import threading
import numpy as np
import time

import mxnet as mx
from gluoncv import model_zoo, data, utils
from matplotlib import pyplot as plt
from PIL import Image

# Initialize ROS node
rospy.init_node('object_detector')

# Finally load model:
net = model_zoo.get_model('ssd_512_resnet50_v1_voc', pretrained=True)

class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class cvThread(threading.Thread):
    """
    Thread that displays and processes the current image
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.image = None

        # Initialize published Twist message
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.last_time = time.time()

    def run(self):
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while True:
            self.image = self.queue.get()

            # Process the current image
            self.processImage(self.image)

            # Add processed images as small images on top of main image
            #result = self.addSmallPictures(self.image, [mask]) #ez ne fog kelleni
            cv2.imshow("frame", self.image)

            # Check for 'q' key to exit
            k = cv2.waitKey(1) & 0xFF
            if k in [27, ord('q')]:
                # Stop every motion
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                pub.publish(self.cmd_vel)
                # Quit
                rospy.signal_shutdown('Quit')

    def processImage(self, img):
        
        # Create an OpenCV window to show the result
        cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Detection", 800,600)
        # Detection
        processed = AI_process(img)
        image_processed=mx.image.imread(processed)
        image_to_cv=image_processed.asnumpy()
        image_bgr = cv2.cvtColor(image_to_cv, cv2.COLOR_RGB2BGR)
        
        #Detection ablakban megjelenítés
        cv2.imshow("Detection", image_bgr)
        
        self.last_time = time.time()

        # Publish cmd_vel
        pub.publish(self.cmd_vel)
        
        # Return processed frames
        return processed

    # Add small images to the top row of the main image
    def addSmallPictures(self, img, small_images, size=(160, 120)):
        '''
        :param img: main image
        :param small_images: array of small images
        :param size: size of small images
        :return: overlayed image
        '''

        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img

def queueMonocular(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        #cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # in case of non-compressed image stream only
        cv2Img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        qMono.put(cv2Img)

def AI_process(picture):
    image_rgb = cv2.cvtColor(picture, cv2.COLOR_BGR2RGB)
    pic = mx.nd.array(image_rgb)
    x, img = data.transforms.presets.ssd.transform_test(pic,short=512)
    class_IDs, scores, bounding_boxes = net(x)

    ax = utils.viz.plot_bbox(img, bounding_boxes[0], scores[0],
                             class_IDs[0], class_names=net.classes)
    
    ax.axis('off')
    plt.tight_layout()

    fig = ax.get_figure()
    fig.canvas.draw()
    img_data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    img_data = img_data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    pil_img = Image.fromarray(img_data)

    output_file = 'output.jpg'
    pil_img.save(output_file, 'JPEG')

    return output_file

queueSize = 1      
qMono = BufferQueue(queueSize)

bridge = CvBridge()


# Define your image topic
image_topic = "/camera/image/compressed"
# Set up your subscriber and define its callback
rospy.Subscriber(image_topic, CompressedImage, queueMonocular)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Start image processing thread
cvThreadHandle = cvThread(qMono)
cvThreadHandle.setDaemon(True)
cvThreadHandle.start()

# Spin until Ctrl+C
rospy.spin()