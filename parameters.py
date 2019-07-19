class Parameters:
    """
    A class where you specify the parameters for this package
    Because there are so many parameters
    """
    def __init__(self):
        # TODO: Change the parameters according to your needs
        #=========== listener variables ===========#
        # Variable for visualization: 0 for no visualization, 1 for visualization
        self.display = 1
        self.mode = 'hd'

        if self.mode == 'sd':
            self.width, self.height = 512, 424
        elif self.mode == 'hd':
            self.width, self.height = 1920, 1080
        else:
            self.width, self.height = 960, 540

        #=========== openpose_pub variables ============#
        # Variable for
        self.openpose_path = "/home/icl-baby/openpose"
        self.json_path = "/home/icl-baby/test-openpose/openpose"
        self.img_path = "/home/icl-baby/test-openpose/imgs"

        #======= data_processor_class variables =======#
        self.data_path = "data"