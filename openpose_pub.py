import rospy
from openpose_msg.msg import Openpose
import glob, json, os

def talker(path, open_path):
	for f in glob.glob(path + '/*'):
		os.remove(f)
	pub = rospy.Publisher('openpose_img', Openpose, queue_size=5)
	rospy.init_node('openpose_ros', anonymous=True)
	rate = rospy.Rate(24) # 20hz
	x_prev = -1
	keypoints = Openpose()
	os.chdir(open_path)
	os.system("gnome-terminal -e 'bash -c \"./build/examples/openpose/openpose.bin --write_json /home/icl-baby/test-openpose/openpose; exec bash\"'")
	# --write_video /home/icl-baby/collection_data.avi --write_video_fps 22
	while not rospy.is_shutdown():
		x = len(glob.glob(path + '/*')) - 1
		if x >= 0 and x > x_prev:
			json_file = '/%012d_keypoints.json' %x
			file = open(path + json_file, 'r')
			f = json.load(file)
			if f['people'] != []:
				kp = f['people'][0]['pose_keypoints_2d']
				keypoints.data = [int(kp[j]) for j in range(len(kp))
					if j not in [3 * i + 2 for i in range(22)]]
				keypoints.header.stamp = rospy.Time.now()
				rospy.loginfo("Keypoint Array Passed!")
				pub.publish(keypoints)
		x_prev = x
		rate.sleep()

if __name__ == '__main__':
	try:
		talker('/home/icl-baby/test-openpose/openpose',
			'/home/icl-baby/openpose')
	except rospy.ROSInterruptException:
		pass