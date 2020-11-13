import numpy as np
import cv2

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

    def segment_ground2pixel(self, list_seg, w, h, map):
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

            if map == "calibration_pattern":
                pixel1 = Point(x=image_point_undist[0][0], y=image_point_undist[0][1])
                pixel2 = Point(x=image_point_undist[1][0], y=image_point_undist[1][1])
            else:
                pixel1 = Point(x=int(round(pxl1.x)), y=int(round(pxl1.y)))
                pixel2 = Point(x=int(round(pxl2.x)), y=int(round(pxl2.y)))


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
