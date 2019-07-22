import rospy, message_filters
from sensor_msgs.msg import Image
from openpose_msg.msg import Openpose
from pred_data_msg.msg import PredData
import argparse
from data_processor_class_new import DataProc
import cv2
from cv_bridge import CvBridge, CvBridgeError
from utils.misc import *
from pynput import keyboard
from parameters import Parameters


# os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'
# os.environ['CUDA_VISIBLE_DEVICES'] = "1"

param_master = Parameters()
flag = False
data_processor = DataProc(param_master.data_path)
depth_bank = []
pub_view = rospy.Publisher('final_imgs', Image, queue_size=1)


def on_press(key):
    """
    The listener for pressed keys in this script.
    To start data collection, press 'b'; to end it, press 'e'. Press 'q' at any time to quit all relevant processes.
    :param key: the pressed key
    """
    global flag, data_processor
    try:
        if key.char == 'b':
            print 'Starting action collection...'
            flag = True
        elif key.char == 'e':
            flag = False
            print 'Action collection complete'
            iteration = int(raw_input('Specify the iteration:'))
            # action = 10
            data_processor.process_iteration(iteration)
        elif key.char == 'q':
            close_windows()
            os._exit(0)
    except AttributeError:
        print('special key {0} pressed'.format(key))


def display_img(img, t):
    """
    Publish the OpenPose rendered image marked with the historical trajectory
    :param img: the OpenPose rendered image
    :param t: the trajectory
    """
    bridge = CvBridge()
    # Paint the trajectory
    for i in range(len(t)):
        t[i] = tuple(t[i])
        if t[i] != (0, 0):
            cv2.circle(img, t[i], 4, (242, 194, 75), cv2.FILLED)
            if i > 0:
                cv2.line(img, t[i - 1], t[i], (189, 242, 75), thickness=2)
    cv_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    pub_view.publish(cv_msg)


def in_range(r_h):
    """
    Check if the predicted OpenPose pixel coordinates are valid
    :param r_h: the predicted OpenPose pixel coordinates
    :return: whether r_h is a valid prediction
    """
    return (r_h[0] > 0) and (r_h[0] < param_master.height) and (r_h[1] > 0) and (r_h[1] < param_master.width)


# r_buffer stores the most recent 5 positions of the right hand
r_buffer = []


def callback(data, points, img):
    """
    The callback for the listeners
    :param data: the depth image from kinect
    :param points: the keypoints from openpose_pub
    :param img: the image from openpose_pub
    """
    global flag, data_processor, r_buffer
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "32FC1")
        img = bridge.imgmsg_to_cv2(img, "rgb8")
        keypoints = points.data

        # Extract right arm data
        r = list(keypoints[2:10])

        if in_range(r[6:8]):
            r_buffer.append(r[6:8])

        if flag:
            r_final = []
            r_h = []

            for i in range(len(r) / 2):
                r_h = r[2 * i: 2 * i + 2]
                if not in_range(r_h):
                    return
                # Find the depth according to pixel
                # Append the depth data
                r_h.append(cv_image[r_h[1], r_h[0]])
                r_final.append(r_h)

            imgs = []
            imgs.extend(cv_image)
            data_processor.collect_data(r_final, imgs)
            rospy.loginfo(r_h)

        if m == 1:
            if len(r_buffer) > 5:
                r_buffer.pop(0)
            if len(r_buffer) == 5:
                # Visualize the image here
                display_img(img, r_buffer)

    except CvBridgeError as e:
        rospy.loginfo("Conversion failed")


print 'During the data collection, press B to begin recording an action and E to end recording an action, ' \
      + 'then specify the action as prompted. Press q to exit at any time.'

m = param_master.display

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release_standard)
listener.start()

curr_path = os.getcwd()
launch_windows(curr_path, param_master.mode)

rospy.init_node("kinect_collect")
# Two different subscribers for image and openpose json data
image_sub = message_filters.Subscriber("/kinect2/%s/image_depth_rect" % param_master.mode, Image)
points_sub = message_filters.Subscriber("/openpose_img", Openpose)
rgb_sub = message_filters.Subscriber("/openpose_out", Image)
# Messages are synchronized
ts = message_filters.ApproximateTimeSynchronizer([image_sub, points_sub, rgb_sub], queue_size=3, slop=0.05)
ts.registerCallback(callback)
print "Callback started"
rospy.spin()
