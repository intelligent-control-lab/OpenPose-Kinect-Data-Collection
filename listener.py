import rospy, message_filters
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import Image
from openpose_msg.msg import Openpose
from pred_data_msg.msg import PredData
# from control_kinova import *
import math, os, subprocess, signal, sys, argparse
import numpy as np
from data_processor_class_new import DataProc, judge_outlier
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pynput import keyboard
from control_kinova import *

os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID'
os.environ['CUDA_VISIBLE_DEVICES'] = "1"

f = (1080 / 2) / math.tan(1.0 / 6 * math.pi)
flag = False
data_processor = DataProc()

acts = open('action_related/actions.txt', 'r')
actions = acts.readlines()


def on_press(key):
    global m, flag, data_processor
    try:
        if m == 0:
            # If we ARE collecting
            if key.char == 'b':
                print 'Starting action collection...'
                flag = True
            elif key.char == 'e':
                flag = False
                print 'Action collection complete'
                action = int(raw_input('Specify the action:'))
                # action = 10
                data_processor.process_action(action)
    except AttributeError:
        print('special key {0} pressed'.format(key))


def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False


depth_bank = []


def convert_back(p):
    global f, depth_bank
    print p
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


def display_img(img, t, action):
    # Put the text for the action on there
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, 'Action:%s' % actions[action], (10, 17), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    # Paint the trajectory
    for i in range(len(t)):
        t[i] = tuple(t[i])
        if t[i] != (0, 0):
            cv2.circle(img, t[i], 3, (242, 194, 75), cv2.FILLED)
            if i > 0:
                cv2.line(img, t[i - 1], t[i], (189, 242, 75))

    cv2.imwrite("img.png", img)
    cv2.imshow("Predicted action", img)


def action_talker(action, pred_traj):
    pub = rospy.Publisher('predictions', PredData, queue_size=2)
    rate = rospy.Rate(24)  # 20hz

    while not rospy.is_shutdown():
        act = PredData()
        act.action = action
        act.traj.data = pred_traj
        pub.publish(act)
        rate.sleep()


r_buffer = []


def callback(data, points, img):
    global m, flag, data_processor, r_buffer
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "32FC1")
        img = bridge.imgmsg_to_cv2(img, "rgb8")
        keypoints = points.data
        if m == 1:
            flag = True

        if flag:
            # Extract right arm data
            r = list(keypoints[2:10])
            r_final = []
            r_h = []
            imgs = []
            for i in range(len(r) / 2):
                r_h = r[2 * i: 2 * i + 2]
                # Find the depth according to pixel
                # Append the depth data
                r_h.append(cv_image[r_h[1], r_h[0]])
                # r_h = convert_real(r_h)
                r_final.append(r_h)
            imgs.append(cv_image)
            if m == 0:
                data_processor.collect_data(r_final, imgs)
            else:
                r_buffer.append(r_final[-1])
                if len(r_buffer) > 20:
                    r_buffer.pop(0)
                if len(r_buffer) == 20:
                    r_buff = np.array(r_buffer)
                    r_buff = np.reshape([r_buff], (1, 20, 3))
                    # FUTURE WORK: WHOLE ARM PREDICTION
                    traj, action = data_processor.predict_traj(r_buff)
                    # Visualize the image here
                    # Convert real world data back to the image in pixels
                    new_traj = []
                    for t in traj:
                        for i in range(len(t)):
                            new_traj.append(convert_back(t[i]))
                    # Translate one-hot encoding back to integer
                    action = np.argmax(action)
                    # IDK ABOUT THIS ONE PLS DOUBLE CHECK
                    traj = traj.flatten()

                    display_img(img, new_traj, action)
                    # if not controller.in_action:
                    #     make_decision(action)

                    action_talker(action, traj)
            rospy.loginfo(r_h)
    except CvBridgeError as e:
        rospy.loginfo("Conversion failed")


def launch_windows_wrist():
    os.system("gnome-terminal -e 'bash -c \"roslaunch kinect2_bridge kinect2_bridge.launch; exec bash \"'")
    os.system(
        "gnome-terminal -e 'bash -c \"rosrun virtual_cam stream _device:=/dev/video1 _width:=512 _height:=424 _fourcc:=YV12 image:=/kinect2/sd/image_color_rect; exec bash \"'")
    os.chdir('/home/icl-baby/Documents/Siyan-Summer2019/New_Code')
    os.system("gnome-terminal -e 'bash -c \"python openpose_pub.py; exec bash \"'")


parser = argparse.ArgumentParser(description='Specify if you would like to collect data or demo.')
parser.add_argument('mode', metavar='M', type=int, nargs='+',
                    help='0 for data collection, 1 for demo')

args = parser.parse_args()
m = args.mode[0]
print m
if m == 0:
    print 'During the data collection, press B to begin recording an action and E to end recording an action, then specify the action as prompted.'

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

launch_windows_wrist()

rospy.init_node("kinect_collect")
# begin()
# Two different subscribers for image and openpose json data
image_sub = message_filters.Subscriber("/kinect2/sd/image_depth_rect", Image)
points_sub = message_filters.Subscriber("/openpose_img", Openpose)
rgb_sub = message_filters.Subscriber("kinect2/sd/image_color_rect", Image)
# Messages are synchronized
ts = message_filters.ApproximateTimeSynchronizer([image_sub, points_sub, rgb_sub], queue_size=2, slop=0.05)
ts.registerCallback(callback)
print "Callback started"
rospy.spin()
