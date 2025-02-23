#!/usr/bin/env python
import requests
import rospy
import cv2
import numpy as np
import open3d as o3d
import base64
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

MOBIUS_URL = "http://114.71.220.59:7579/Mobius/AIoT_Test"
HEADERS = {
    "X-M2M-Origin": "S",
    "X-M2M-RI": "12345",
    "Content-Type": "application/json;ty=4",
    "Accept": "application/json"
}

def create_container(container_name):
    """Mobius에 컨테이너 생성"""
    url = f"{MOBIUS_URL}"
    data = {
        "m2m:cnt": {
            "rn": container_name,
            "lbl": ["ss"],
            "mbs": 4000000000
        }
    }
    response = requests.post(url, headers=HEADERS, json=data)
    return response.status_code == 201

def upload_file(container_url, resource_name, content, content_type):
    """파일이 아닌 데이터 자체를 Mobius에 업로드"""
    url = f"{container_url}/{resource_name}"
    data = {
        "m2m:cin": {
            "con": content,
            "cnf": content_type
        }
    }
    response = requests.post(url, headers=HEADERS, json=data)
    if response.status_code == 201:
        rospy.loginfo(f"{resource_name} uploaded successfully.")
    else:
        rospy.logerr(f"Failed to upload {resource_name}. Status: {response.status_code}, Response: {response.text}")

class RealSenseVelodyneRecorder:
    def __init__(self):
        rospy.init_node("realsense_velodyne_recorder", anonymous=True)
        self.bridge = CvBridge()

        # 현재 시간을 기반으로 컨테이너 생성
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.container_url = f"{MOBIUS_URL}/{self.timestamp}"
        
        if not create_container(self.timestamp):
            rospy.logerr(f"Failed to create container: {self.timestamp}")
            return

        # ROS 토픽 구독
        rospy.Subscriber("/camera/color/image_raw", Image, self.upload_rgb)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.upload_depth)
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.upload_realsense_pcd)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.upload_velodyne_pcd)

    def upload_rgb(self, msg):
        """RGB 이미지를 Base64로 인코딩하여 Mobius에 업로드"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buffer = cv2.imencode(".png", cv_image)
            base64_img = base64.b64encode(buffer).decode("utf-8")
            upload_file(self.container_url, "rgb_image", base64_img, "image/png")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to process RGB image: {e}")

    def upload_depth(self, msg):
        """Depth 이미지를 Base64로 인코딩하여 Mobius에 업로드"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            _, buffer = cv2.imencode(".png", depth_normalized)
            base64_img = base64.b64encode(buffer).decode("utf-8")
            upload_file(self.container_url, "depth_image", base64_img, "image/png")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to process Depth image: {e}")

    def upload_realsense_pcd(self, msg):
        """RealSense 포인트 클라우드를 PCD 텍스트 형식으로 변환하여 Mobius에 업로드"""
        try:
            cloud_points = [[p[0], p[1], p[2]] for p in point_cloud2.read_points(msg, skip_nans=True)]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points))
            pcd_str = o3d.io.write_point_cloud_to_buffer(pcd, file_format="pcd")
            base64_pcd = base64.b64encode(pcd_str).decode("utf-8")
            upload_file(self.container_url, "realsense_point_cloud", base64_pcd, "application/octet-stream")
        except Exception as e:
            rospy.logerr(f"Failed to process RealSense PCD: {e}")

    def upload_velodyne_pcd(self, msg):
        """Velodyne 포인트 클라우드를 PCD 텍스트 형식으로 변환하여 Mobius에 업로드"""
        try:
            cloud_points = [[p[0], p[1], p[2]] for p in point_cloud2.read_points(msg, skip_nans=True)]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points))
            pcd_str = o3d.io.write_point_cloud_to_buffer(pcd, file_format="pcd")
            base64_pcd = base64.b64encode(pcd_str).decode("utf-8")
            upload_file(self.container_url, "velodyne_point_cloud", base64_pcd, "application/octet-stream")
        except Exception as e:
            rospy.logerr(f"Failed to process Velodyne PCD: {e}")

if __name__ == "__main__":
    recorder = RealSenseVelodyneRecorder()
    rospy.spin()
