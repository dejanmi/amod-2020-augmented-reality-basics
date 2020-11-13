#!/usr/bin/env python3

import os
import numpy as np
import rospy
import rospkg
import yaml
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo

from augmenter import Point, Segment, Augmenter


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
                    list_seg_pxl = aug.segment_ground2pixel(self.list_seg, w=self.image_width, h=self.image_height, map=self.map_file)
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


