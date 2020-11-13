#!/usr/bin/env python3

import os

import numpy as np
import rospy
import rospkg
import cv2
import yaml
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo

class Point:
    """
    Point class. Convenience class for storing ROS-independent 3D points.
    """
    def __init__(self, x=None, y=None, z=None):
        self.x = x  #: x-coordinate
        self.y = y  #: y-coordinate
        self.z = z  #: z-coordinate

    @staticmethod
    def from_message(msg):
        """
        Generates a class instance from a ROS message. Expects that the message has attributes ``x`` and ``y``.
        If the message additionally has a ``z`` attribute, it will take it as well. Otherwise ``z`` will be set to 0.
        Args:
            msg: A ROS message or another object with ``x`` and ``y`` attributes
        Returns:
            :py:class:`Point` : A Point object
        """
        x = msg.x
        y = msg.y
        try:
            z = msg.z
        except AttributeError:
            z = 0
        return Point(x, y, z)

class Segment:
    def __init__(self, point1: Point, point2: Point, color, coord_sys):
        self.point1 = point1
        self.point2 = point2
        self.color = color
        self.coord_sys = coord_sys


class Augmenter:

    def __init__(self, cam_info, homography):
        self.cam_info = cam_info
        self.K = np.array(self.cam_info.K).reshape((3, 3))
        self.D = np.array(self.cam_info.D)
        self.H = homography
        self.Hinv = np.linalg.inv(self.H)
        self.defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0, 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}

    def process_image(self, img):
        img_und = cv2.undistort(img, self.K, self.D, None, None)
        return img_und

    def ground2pixel(self, point):
        if point.z != 0:
            msg = 'This method assumes that the point is a ground point (z=0). '
            msg += 'However, the point is (%s,%s,%s)' % (point.x, point.y, point.z)
            raise ValueError(msg)

        ground_point = np.array([point.x, point.y, 1.0])
        image_point = self.Hinv @ ground_point
        image_point = image_point / image_point[2]

        pixel = Point()
        pixel.x = image_point[0]
        pixel.y = image_point[1]



        return pixel

    def segment_ground2pixel(self, list_seg, w, h):
        list_seg_pxl = []
        for seg in list_seg:
            pxl1 = self.ground2pixel(seg.point1)
            pxl2 = self.ground2pixel(seg.point2)
            pixels = np.array([[[pxl1.x, pxl1.y], [pxl2.x, pxl2.y]]])
            newcameramtx, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1, (w, h))
            image_point_undist = cv2.undistortPoints(pixels, self.K, self.D, None, newcameramtx)
            image_point_undist = np.squeeze(image_point_undist)
            image_point_undist = np.rint(image_point_undist)
            image_point_undist = image_point_undist.astype(int)
            pixel1 = Point(x=image_point_undist[0][0], y=image_point_undist[0][1])
            pixel2 = Point(x=image_point_undist[1][0], y=image_point_undist[1][1])

            new_seg = Segment(point1=pixel1, point2=pixel2, color=seg.color, coord_sys="image01")
            list_seg_pxl.append(new_seg)


        return list_seg_pxl

    def segment_image2pixels(self, list_seg, image_height, image_width):
        list_seg_pxl = []
        for seg in list_seg:
            pxl1_u = image_width * seg.point1.y
            pxl1_v = image_height * seg.point1.x
            pxl1 = Point(x=pxl1_u, y=pxl1_v)
            pxl2_u = image_width * seg.point2.y
            pxl2_v = image_height * seg.point2.x
            pxl2 = Point(x=pxl2_u, y=pxl2_v)
            # pixels = np.array([[[pxl1.x, pxl1.y], [pxl2.x, pxl2.y]]])
            # image_point_undist = cv2.undistortPoints(pixels, self.K, self.D)
            # print(image_point_undist)
            new_seg = Segment(point1=pxl1, point2=pxl2, color=seg.color, coord_sys="image01")
            list_seg_pxl.append(new_seg)


        return list_seg_pxl

    def render_segments(self, list_seg, image):
        image = self.draw_segment(image, list_seg)

        return image

    def draw_segment(self, image, list_seg):
        for seg in list_seg:
            _color_type, [r, g, b] = self.defined_colors[seg.color]
            cv2.line(image, (seg.point1.x, seg.point1.y), (seg.point2.x, seg.point2.y), (b * 255, g * 255, r * 255), 5)

        return image

class AugmentedRealityBasicsNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
        self.map_file = rospy.get_param(f'/{self.veh_name}/{node_name}/map_file', 100)
        self.pkg_name = rospy.get_param(f'/{self.veh_name}/{node_name}/pkg_name', 100)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(self.pkg_name)
        map_file_path = os.path.join(pkg_path, 'include', 'maps', f'{self.map_file}.yaml')
        map_dict = self.readYamlFile(map_file_path)
        map_points = map_dict["points"]
        map_seg = map_dict["segments"]

        self.list_seg = []
        for seg in map_seg:
            points = seg["points"]
            coord_sys = map_points[points[0]][0]
            if coord_sys == "image01":
                point1 = map_points[points[0]][1]
                point1 = Point(x=point1[0], y=point1[1])
                point2 = map_points[points[1]][1]
                point2 = Point(x=point2[0], y=point2[1])
            else:
                point1 = map_points[points[0]][1]
                point1 = Point(x=point1[0], y=point1[1], z=point1[2])
                point2 = map_points[points[1]][1]
                point2 = Point(x=point2[0], y=point2[1], z=point2[2])

            color = seg["color"]
            segment = Segment(point1=point1, point2=point2, color=color, coord_sys=coord_sys)
            self.list_seg.append(segment)

        self.bridge = CvBridge()
        self.image = None
        self.image_height = None
        self.image_width = None
        self.camera_info_received = False
        self.cam_info = None
        self.homography = self.load_extrinsics()
        self.ext_param_ground_to_cam = np.array(self.homography).reshape((3, 3))
        self.sub_image_comp = rospy.Subscriber(
            f'/{self.veh_name}/camera_node/image/compressed',
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1
        )
        self.sub_cam_info = rospy.Subscriber(f'/{self.veh_name}/camera_node/camera_info', CameraInfo, self.cb_camera_info, queue_size=1)
        # construct publisher
        self.pub_aug_image = rospy.Publisher(f'/{self.veh_name}/{node_name}/{self.map_file}/image/compressed', CompressedImage, queue_size=10)

    def image_cb(self, image_msg):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr('Could not decode image: %s' % e)
            return

        self.image = image
        self.image_height = np.shape(image)[0]
        self.image_width = np.shape(image)[1]

    def cb_camera_info(self, msg):
        if not self.camera_info_received:
            self.cam_info = msg

        self.camera_info_received = True

    def load_extrinsics(self):
        """
        Loads the homography matrix from the extrinsic calibration file.
        Returns:
            :obj:`numpy array`: the loaded homography matrix
        """
        # load intrinsic calibration
        cali_file_folder = '/data/config/calibrations/camera_extrinsic/'
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log("Can't find calibration file: %s.\n Using default calibration instead."
                     % cali_file, 'warn')
            cali_file = (cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = 'Found no calibration file ... aborting'
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        try:
            with open(cali_file, 'r') as stream:
                calib_data = yaml.load(stream)
        except yaml.YAMLError:
            msg = 'Error in parsing calibration file %s ... aborting' % cali_file
            self.log(msg, 'err')
            rospy.signal_shutdown(msg)

        return calib_data['homography']

    def readYamlFile(self, fname):
        """
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                    % (fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def run(self):
        while not rospy.is_shutdown():
            if self.image is None:
                continue
            if not self.camera_info_received:
                continue
            aug = Augmenter(self.cam_info, self.ext_param_ground_to_cam)
            img_undist = aug.process_image(self.image)
            if self.list_seg:
                if self.list_seg[0].coord_sys == "axle":
                    list_seg_pxl = aug.segment_ground2pixel(self.list_seg, w=self.image_width, h=self.image_height)
                    img_rend = aug.render_segments(list_seg_pxl, img_undist)
                    comp_image = self.bridge.cv2_to_compressed_imgmsg(img_rend)
                    self.pub_aug_image.publish(comp_image)
                elif self.list_seg[0].coord_sys == "image01":
                    list_seg_pxl = aug.segment_image2pixels(self.list_seg,self.image_height, self.image_width)
                    img_rend = aug.render_segments(list_seg_pxl, img_undist)
                    comp_image = self.bridge.cv2_to_compressed_imgmsg(img_rend)
                    self.pub_aug_image.publish(comp_image)


if __name__ == '__main__':
    # create the node
    node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()


