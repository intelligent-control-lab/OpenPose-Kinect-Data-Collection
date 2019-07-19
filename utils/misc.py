import os, time, subprocess, sys
from pynput import keyboard

"""
This file contains multiple functions
"""

def launch_windows(curr_path, mode):
    """
    Launch the necessary processes for script execution
    :param curr_path: current working directory
    :param mode: mode for kinect camera; sd for 512x424 image, qhd for 960x540 image, hd for 1920x1080 image
    """
    subprocess.call("gnome-terminal -e 'bash -c \"roslaunch kinect2_bridge kinect2_bridge.launch; exec bash \"'",
                    shell=True)
    if mode == 'sd':
        width, height = 512, 424
    elif mode == 'hd':
        width, height = 1920, 1080
    else:
        mode = 'qhd'
        width, height = 960, 540
    os.system(
        "gnome-terminal -e 'bash -c \"rosrun virtual_cam stream _device:=/dev/video1 _width:=%d _height:=%d _fourcc:=YV12 image:=/kinect2/%s/image_color_rect; exec bash \"'" % (width, height, mode))
    os.chdir(curr_path)
    os.system("gnome-terminal -e 'bash -c \"python openpose_pub.py; exec bash \"'")
    os.system("gnome-terminal -e 'bash -c \"rosrun image_view image_view image:=/final_imgs; exec bash \"'")


def close_windows():
    out_process = subprocess.check_output(['ps', '-ef']).split('\n')
    for str in out_process:
        if ('kinetic/' in str) or ('openpose' in str) or ('video1' in str):
            str_arr = str.split(" ")
            str_arr.pop(0)
            i = 0
            while str_arr[i] == '':
                i = i + 1
            id = str_arr[i]
            os.system('kill -9 %s' % id)


def on_release_standard(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def on_press_standard(key):
    try:
        if key.char == 'q':
            close_windows()
            os._exit(0)
    except AttributeError:
        print('special key {0} pressed'.format(key))


def judge_outlier(a):
    flag = True
    k = a[2]
    flag = flag and (k < 400 or k > 1000)
    return flag


def convert_real(p):
    global f
    x_v = p[0]
    y_v = p[1]
    z_w = p[2]

    if z_w == 0:
        print "invalid depth data"
        return [0.0, 0.0, 0.0]
    # The new computed values will be in millimeters
    # Since f, x_v and y_v all have pixels as the unit
    p[0] = z_w * x_v / f
    p[1] = z_w * y_v / f
    return p


def convert_back(p):
    global f
    x_w = p[0]
    y_w = p[1]
    z_w = p[2]

    p[0] = int(x_w * f / z_w)
    p[1] = int(y_w * f / z_w)
    # p.pop(2)
    return p
