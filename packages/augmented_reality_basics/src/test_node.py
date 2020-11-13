# import os

# import numpy as np
import rospy
# import cv2
# import yaml
# from image_geometry import PinholeCameraModel
from duckietown.dtros import DTROS, NodeType
# from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String


# class Point:
#     """
#     Point class. Convenience class for storing ROS-independent 3D points.
#     """
#     def __init__(self, x=None, y=None, z=None):
#         self.x = x  #: x-coordinate
#         self.y = y  #: y-coordinate
#         self.z = z  #: z-coordinate
#
#     @staticmethod
#     def from_message(msg):
#         """
#         Generates a class instance from a ROS message. Expects that the message has attributes ``x`` and ``y``.
#         If the message additionally has a ``z`` attribute, it will take it as well. Otherwise ``z`` will be set to 0.
#         Args:
#             msg: A ROS message or another object with ``x`` and ``y`` attributes
#         Returns:
#             :py:class:`Point` : A Point object
#         """
#         x = msg.x
#         y = msg.y
#         try:
#             z = msg.z
#         except AttributeError:
#             z = 0
#         return Point(x, y, z)
#
# class Augmenter:
#
#     def __init__(self, pcm, homography):
#         self.pcm = pcm
#         self.H = homography
#         self.Hinv = np.linalg.inv(self.H)
#
#     def process_image(self, img):
#         img_rect = self.pcm.rectifyImage(img)
#         return img_rect
#
#     def ground2pixel(self, point):
#         if point.z != 0:
#             msg = 'This method assumes that the point is a ground point (z=0). '
#             msg += 'However, the point is (%s,%s,%s)' % (point.x, point.y, point.z)
#             raise ValueError(msg)
#
#         ground_point = np.array([point.x, point.y, 1.0])
#         image_point = self.Hinv @ ground_point
#         image_point = image_point / image_point[2]
#
#         pixel = Point()
#         pixel.x = image_point[0]
#         pixel.y = image_point[1]
#
#         return pixel
#
#     def render_segments(self, pxl, image):
#         image[int(pxl.x), int(pxl.y), :] = [255, 0, 0]


class AugmentedRealityBasicsNode(DTROS):

    def __init__(self, node_name):
        # # initialize the DTROS parent class
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # self.veh_name = rospy.get_namespace().strip("/")
        # # self.veh_name = "dekibot"
        # map_name = "test"
        # self.camera_info_received = False
        # self.cam_info = None
        # self.pcm = PinholeCameraModel()
        # self.pcm.fromCameraInfo(self.cam_info)
        # self.homography = self.load_extrinsics()
        # self.ext_param_ground_to_cam = np.array(self.homography).reshape((3, 3))
        #
        # self.sub_cam = rospy.Subscriber(f'/{self.veh_name}/camera_node/image/compressed',
        #                                                CompressedImage, self.callback)
        #
        # self.sub_cam_info = rospy.Subscriber("~camera_info", CameraInfo, self.cb_camera_info, queue_size=1)

        # self.pub_aug_image = rospy.Publisher(f'/{self.veh_name}/{node_name}/{map_name}/image/compressed', CompressedImage, queue_size=10)
        self.pub_aug_image = rospy.Publisher('~chatter', String, queue_size=10)
        # self.image = None

    # def callback(self, data):
    #     self.image = cv2.cvtColor(np.ascontiguousarray(data), cv2.COLOR_BGR2RGB)
    #
    # def cb_camera_info(self, msg):
    #     if not self.camera_info_received:
    #         self.cam_info = msg
    #
    #     self.camera_info_received = True
    #
    # def load_extrinsics(self):
    #     """
    #     Loads the homography matrix from the extrinsic calibration file.
    #     Returns:
    #         :obj:`numpy array`: the loaded homography matrix
    #     """
    #     # load intrinsic calibration
    #     cali_file_folder = '/data/config/calibrations/camera_extrinsic/'
    #     cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
    #
    #     # Locate calibration yaml file or use the default otherwise
    #     if not os.path.isfile(cali_file):
    #         self.log("Can't find calibration file: %s.\n Using default calibration instead."
    #                  % cali_file, 'warn')
    #         cali_file = (cali_file_folder + "default.yaml")
    #
    #     # Shutdown if no calibration file not found
    #     if not os.path.isfile(cali_file):
    #         msg = 'Found no calibration file ... aborting'
    #         self.log(msg, 'err')
    #         rospy.signal_shutdown(msg)
    #
    #     try:
    #         with open(cali_file, 'r') as stream:
    #             calib_data = yaml.load(stream)
    #     except yaml.YAMLError:
    #         msg = 'Error in parsing calibration file %s ... aborting' % cali_file
    #         self.log(msg, 'err')
    #         rospy.signal_shutdown(msg)
    #
    #     return calib_data['homography']

    def aug(self):
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # aug = Augmenter(self.pcm, self.ext_param_ground_to_cam)
            # test_point = Point(0.315, 0.093, 0)
            # img_rect = aug.process_image(self.image)
            # pxl = aug.ground2pixel(test_point)
            # img_rend = aug.render_segments(pxl, img_rect)
            # msg = CompressedImage()
            # msg.header.stamp = rospy.Time.now()
            # msg.format = "jpeg"
            # msg.data = np.array(cv2.imencode('.jpg', img_rend)[1]).tostring()
            # self.pub_aug_image.publish(msg)
            message = "Hello from"
            rospy.loginfo("Publishing message: '%s'" % message)
            self.pub_aug_image.publish(message)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    node.aug()
    # keep spinning
    rospy.spin()
