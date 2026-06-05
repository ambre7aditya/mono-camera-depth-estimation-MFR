#!/usr/bin/env python3

import os
import cv2
import yaml
from foxglove_msgs import msg
import rospy
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from foxglove_msgs.msg import ImageMarkerArray


IMAGE_TOPIC = "/zed2/zed_node/left/image_rect_color"
BBOX_TOPIC = "/stereo_cone_perception/bounding_boxes"

DEBUG_IMAGE_TOPIC = "/mono_cone_perception/debug_image"
MONO_CONES_TOPIC = "/mono_cone_perception/cones"

HOMOGRAPHY_PATH = os.path.expanduser(
    "~/mono_cone_perception/config/homography_manual.yaml"
)


class MonoConeRosNode:
    def __init__(self):
        rospy.init_node("mono_cone_ros_node", anonymous=False)

        self.bridge = CvBridge()

        self.latest_image_msg = None
        self.latest_cv_image = None

        self.H_img_to_world, self.roi_polygon = self.load_homography(HOMOGRAPHY_PATH)

        self.debug_image_pub = rospy.Publisher(
            DEBUG_IMAGE_TOPIC,
            Image,
            queue_size=1,
        )

        self.cones_pub = rospy.Publisher(
            MONO_CONES_TOPIC,
            PointCloud,
            queue_size=1,
        )

        self.image_sub = rospy.Subscriber(
            IMAGE_TOPIC,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )

        self.bbox_sub = rospy.Subscriber(
            BBOX_TOPIC,
            ImageMarkerArray,
            self.bbox_callback,
            queue_size=1,
        )

        rospy.loginfo("Mono cone ROS node started.")
        rospy.loginfo(f"Image topic: {IMAGE_TOPIC}")
        rospy.loginfo(f"BBox topic:  {BBOX_TOPIC}")
        rospy.loginfo(f"Debug topic: {DEBUG_IMAGE_TOPIC}")
        rospy.loginfo(f"Cones topic: {MONO_CONES_TOPIC}")
        rospy.loginfo(f"Homography:  {HOMOGRAPHY_PATH}")

    def load_homography(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        H = np.array(data["H_img_to_world"], dtype=np.float64)

        roi_polygon = None
        if "image_points" in data and data["image_points"]:
            roi_polygon = np.array(data["image_points"], dtype=np.float32)

        return H, roi_polygon

    def image_callback(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image_msg = msg
        except Exception as e:
            rospy.logwarn(f"Failed to convert image: {e}")

    def bbox_to_bottom_center(self, marker):
        if len(marker.points) < 4:
            return None

        xs = [p.x for p in marker.points]
        ys = [p.y for p in marker.points]

        x_min = min(xs)
        x_max = max(xs)
        y_min = min(ys)
        y_max = max(ys)

        u = 0.5 * (x_min + x_max)
        v = y_max

        return x_min, y_min, x_max, y_max, u, v

    def point_inside_roi(self, u, v):
        if self.roi_polygon is None:
            return True

        result = cv2.pointPolygonTest(
            self.roi_polygon,
            (float(u), float(v)),
            False,
        )

        return result >= 0

    def image_point_to_world(self, u, v):
        point = np.array([[[u, v]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.H_img_to_world)
        X, Y = transformed[0, 0]
        return float(X), float(Y)

    def bbox_callback(self, msg):
        if self.latest_cv_image is None:
            return

        debug_image = self.latest_cv_image.copy()

        if self.roi_polygon is not None:
            roi_int = self.roi_polygon.astype(np.int32)
            cv2.polylines(
                debug_image,
                [roi_int],
                isClosed=True,
                color=(0, 0, 255),
                thickness=2,
            )

        pointcloud = PointCloud()
        if len(msg.markers) > 0:

            pointcloud.header = msg.markers[0].header

        else:

            if self.latest_image_msg is not None:

                pointcloud.header = self.latest_image_msg.header

        pointcloud.header.frame_id = "mono_camera_ground"

        accepted_count = 0
        skipped_count = 0

        for marker in msg.markers:
            bbox = self.bbox_to_bottom_center(marker)

            if bbox is None:
                skipped_count += 1
                continue

            x_min, y_min, x_max, y_max, u, v = bbox

            inside_roi = self.point_inside_roi(u, v)

            if not inside_roi:
                skipped_count += 1

                cv2.rectangle(
                    debug_image,
                    (int(x_min), int(y_min)),
                    (int(x_max), int(y_max)),
                    (120, 120, 120),
                    1,
                )
                continue

            X, Y = self.image_point_to_world(u, v)

            point = Point32()
            point.x = X
            point.y = Y
            point.z = 0.0
            pointcloud.points.append(point)

            accepted_count += 1

            cv2.rectangle(
                debug_image,
                (int(x_min), int(y_min)),
                (int(x_max), int(y_max)),
                (0, 255, 255),
                2,
            )

            cv2.circle(
                debug_image,
                (int(u), int(v)),
                4,
                (0, 0, 255),
                -1,
            )

            label = f"X={X:.1f}, Y={Y:.1f}"
            cv2.putText(
                debug_image,
                label,
                (int(x_min), int(y_min) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
            )

        status_text = f"mono cones: {accepted_count}, skipped: {skipped_count}"

        cv2.putText(
            debug_image,
            status_text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        self.cones_pub.publish(pointcloud)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            if len(msg.markers) > 0:

                debug_msg.header = msg.markers[0].header

            elif self.latest_image_msg is not None:

                debug_msg.header = self.latest_image_msg.header
           
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logwarn(f"Failed to publish debug image: {e}")

        rospy.loginfo_throttle(
            1.0,
            f"Published {accepted_count} mono cones, skipped {skipped_count}",
        )

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = MonoConeRosNode()
    node.run()