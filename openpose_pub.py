import rospy, cv2
from openpose_msg.msg import Openpose
from sensor_msgs.msg import Image
import glob, json, os, time
from cv_bridge import CvBridge, CvBridgeError
# from PIL import Image


def talker(path, img_path, open_path):
    bridge = CvBridge()
    for f in glob.glob(path + '/*'):
        os.remove(f)
    for f in glob.glob(img_path + '/*'):
        os.remove(f)
    pub = rospy.Publisher('openpose_img', Openpose, queue_size=5)
    pub_img = rospy.Publisher('openpose_out', Image, queue_size=5)
    rospy.init_node('openpose_ros', anonymous=True)
    rate = rospy.Rate(20)  # 20hz
    x_prev = -1
    keypoints = Openpose()
    os.chdir(open_path)
    os.system("gnome-terminal -e 'bash -c \"./build/examples/openpose/openpose.bin "
              + "--write_json /home/icl-baby/test-openpose/openpose --write_images /home/icl-baby/test-openpose/imgs --display 0; exec bash\"'")
    # --write_video /home/icl-baby/collection_data.avi --write_video_fps 22
    while not rospy.is_shutdown():
        x = len(glob.glob(path + '/*')) - 1
        if x >= 0 and x > x_prev:
            json_file = '/%012d_keypoints.json' % x
            file = open(path + json_file, 'r')
            f = json.load(file)
            if len(f['people']) != 0:
                kp = f['people'][0]['pose_keypoints_2d']
                keypoints.data = [int(kp[j]) for j in range(len(kp))
                                  if j not in [3 * i + 2 for i in range(22)]]
                keypoints.header.stamp = rospy.Time.now()
                pub.publish(keypoints)
                rospy.loginfo('Published keypoints')
                img_file = '/%012d_rendered.png' % x
                # Image.open(img_path + img_file).convert("RGB").save(img_path + img_file)
                if os.path.exists(img_path + img_file):
                    img = cv2.imread(img_path + img_file)
                    if img is not None:
                        try:
                            image_message = bridge.cv2_to_imgmsg(img, encoding="rgb8")
                            image_message.header.stamp = rospy.Time.now()
                            pub_img.publish(image_message)
                            os.remove(img_path + img_file)
                        except CvBridgeError as e:
                            print "Conversion failed %s" % e
        x_prev = x
        rate.sleep()


if __name__ == '__main__':
    try:
        talker('/home/icl-baby/test-openpose/openpose', '/home/icl-baby/test-openpose/imgs',
               '/home/icl-baby/openpose')
    except rospy.ROSInterruptException:
        pass
