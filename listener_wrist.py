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


os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'
os.environ['CUDA_VISIBLE_DEVICES'] = "1"

param_master = Parameters()
flag = False
data_processor = DataProc(param_master.data_path)
depth_bank = []
pub_view = rospy.Publisher('final_imgs', Image, queue_size=3)


def on_press(key):
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


def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def convert_back(p):
    global f, depth_bank
    p = p.tolist()
    if not judge_outlier(p):
        if len(depth_bank) == 3:
            depth_bank.pop(0)
        depth_bank.append(p[2])
    else:
        if len(depth_bank) > 1:
            diff = float(depth_bank[len(depth_bank) - 1] - depth_bank[0]) / (len(depth_bank) - 1)
            p[2] = depth_bank[len(depth_bank) - 1] + diff
        elif len(depth_bank) == 1:
            p[2] = depth_bank[0]
        else:
            p[2] = 500
    x_w = p[0]
    y_w = p[1]
    z_w = p[2]

    p[0] = int(x_w * f / z_w)
    p[1] = int(y_w * f / z_w)
    p.pop(2)
    return p


def display_img(img, t):
    """
    Publish the OpenPose rendered image marked with the historical trajectory
    :param img: the OpenPose rendered image
    :param t: the trajectory
    :return:
    """
    bridge = CvBridge()
    # Paint the trajectory
    for i in range(len(t)):
        t[i] = tuple(t[i])
        if t[i] != (0, 0):
            cv2.circle(img, t[i], 3, (242, 194, 75), cv2.FILLED)
            if i > 0:
                cv2.line(img, t[i - 1], t[i], (189, 242, 75))
    cv_msg = bridge.cv2_to_imgmsg(img, "bgr8")
    print "PUB"
    pub_view.publish(cv_msg)


def in_range(r_h):
    return (r_h[0] > 0) and (r_h[0] < param_master.height) and (r_h[1] > 0) and (r_h[1] < param_master.width)


r_buffer = []


def callback(data, points, img):
    global flag, data_processor, r_buffer
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "32FC1")
        img = bridge.imgmsg_to_cv2(img, "rgb8")
        keypoints = points.data

        if flag:
            # Extract right arm data
            r = list(keypoints[2:10])
            r_final = []
            r_h = []
            imgs = []
            for i in range(len(r) / 2):
                r_h = r[2 * i: 2 * i + 2]
                if r_h[0] == 0 or r_h[1] == 0:
                    return
                # Find the depth according to pixel
                # Append the depth data
                r_h.append(cv_image[r_h[1], r_h[0]])
                r_final.append(r_h)
            imgs.append(cv_image)
            data_processor.collect_data(r_final, imgs)
            if m == 1:
                r_buffer.append(r_final[-1])
                if len(r_buffer) > 5:
                    r_buffer.pop(0)
                if len(r_buffer) == 5:
                    # Visualize the image here
                    # Convert real world data back to the image in pixels
                    new_traj = []
                    for t in r_buffer:
                        for i in range(len(t)):
                            nt = [int(t[0]), int(t[1])]
                            new_traj.append(nt)
                    display_img(img, new_traj)
            rospy.loginfo(r_h)
    except CvBridgeError as e:
        rospy.loginfo("Conversion failed")


print 'During the data collection, press B to begin recording an action and E to end recording an action, ' \
      + 'then specify the action as prompted. Press q to exit at any time.'

m = param_master.display

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
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
