import pickle, time


class DataProc:
    """
    A class for processing collected data
    """
    def __init__(self, data_path):
        self.data_arr = []
        self.img_arr = []
        self.data_path = data_path

    def process_iteration(self, iteration):
        """
        This function is called when one iteration of action collection is complete
        :param iteration: the iteration
        """
        # Dump everything into a pickle file
        file = open('%s/keypoints/%d_%d.p' % (self.data_path, iteration, int(time.time())), 'wb')
        pickle.dump(self.data_arr, file)
        self.data_arr = []
        print "============== Keypoint data has been saved =============="

        file = open('%s/depth/imgs_%d_%d.p' % (self.data_path, iteration, int(time.time())), 'wb')
        pickle.dump(self.img_arr, file)
        self.img_arr = []
        print "============== Image data has been saved =============="

    def collect_data(self, data, imgs):
        """
        This method adds the keypoint and image data to their respective arrays
        :param data: the keypoints in the form of (openpose_x, openpose_y, kinect_z)
        :param imgs: the kinect depth image
        """
        self.data_arr.append(data)
        self.img_arr.append(imgs)
