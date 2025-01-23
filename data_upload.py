#!/usr/bin/env python
from tokenize import String

import requests
import rospy
import os
import cv2
import numpy as np
import open3d as o3d
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import threading  # 입력 처리를 위한 스레드 사용
import time
import os
import base64

mobius_url = "http://114.71.220.59:7579/Mobius/AIoT_Test"

headers = {
    "X-M2M-Origin": "S",
    "X-M2M-RI": "12345",
    "Content-Type": "application/json;ty=4",
    "Accept": "application/json"
}

def create_container(container_name) :
    url = f"{mobius_url}"
    data = {
        "m2m:cnt": {
            "rn": container_name,
            "lbl": ["ss"],
            "mbs": 4000000000
        }
    }
    response = requests.post(url, headers=headers, json=data)
    if response.status_code == 201:
        print(f"Container '{container_name}' created successfully.")
        return True
    else:
        print(f"Failed to create container '{container_name}'. Status code: {response.status_code}, Response: {response.text}")
        return False

def upload_file(container_url, file_path):

    # 파일 이름과 확장자 추출
    file_name, file_extension = os.path.splitext(os.path.basename(file_path))
    
    # 파일 확장자에 따라 콘텐츠 타입 설정
    if file_extension.lower() == 'velodyne_point_cloud.pcd':
        content_type = 'application/octet-stream'
        resource_name = 'lidar_pcd'
    elif file_extension.lower() == 'rgb_image.png':
        content_type = 'image/png'
        resource_name = 'rgb_image_png'
    elif file_extension.loewr() == 'realsense_point_cloud.pcd':
        content_type = 'application/octet-stream'
        resource_name = 'camera_pcd'
    elif file_extension.lower() == 'depth_image.png':
        content_type = 'image/png'
        resource_name = 'depth_image_png'
    else:
        print(f"Unsupported file type: {file_extension}")

        return
    
def main():
    # 현재 시간을 기반으로 컨테이너 이름 생성
    timestamp = int(time.time())
    container_name = f"{timestamp}"
    
    # 컨테이너 URL 설정
    container_url = f"{mobius_url}/{container_name}".rstrip('/')
    print(f"Container URL: {container_url}")
    
    file_path = "~/pipeline/save" # 파일 저장경로(변동 시 이것도 바꿔야함)
    
    # 컨테이너 생성
    if create_container(container_name):
        upload_file(container_url, file_path)

class RealSenseVelodyneRecorder:
    def __init__(self):
        rospy.init_node("realsense_velodyne_recorder", anonymous=True)
        self.bridge = CvBridge()
        
        # 저장 디렉토리 설정
        self.save_dir = "save"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # 데이터 저장 플래그
        self.save_flag = False

        # ROS 토픽 구독 (RealSense)
        rospy.Subscriber("/camera/color/image_raw", Image, self.save_rgb)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.save_depth)
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.save_realsense_pcd)
        
        # ROS 토픽 구독 (Velodyne)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.save_velodyne_pcd)

        # 데이터 저장 요청 토픽 구독 (control_module에서 전달)
        rospy.Subscriber("/sensor_command", String, self.sensor_command_callback)
        
        # 저장 요청 상태
        self.rgb_requested = False
        self.depth_requested = False
        self.realsense_pcd_requested = False
        self.velodyne_pcd_requested = False

    def sensor_command_callback(self, msg):
        command = msg.data
        if "save" in command:
            rospy.loginfo("Saving data...")
            self.rgb_requested = True
            self.depth_requested = True
            self.realsense_pcd_requested = True
            self.velodyne_pcd_requested

    def get_timestamped_filename(self, name):
        """현재 날짜와 시간을 기반으로 파일명 생성"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # 형식: YYYYMMDD_HHMMSS
        return os.path.join(self.save_dir, f"{timestamp}_{name}")
    def save_rgb(self, msg):
        if self.rgb_requested:
            try:
                rospy.loginfo("Saving RGB image...")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                rgb_filename = self.get_timestamped_filename("rgb_image.png")
                cv2.imwrite(rgb_filename, cv_image)
                rospy.loginfo(f"RGB image saved: {rgb_filename}")
                self.rgb_requested = False
            except CvBridgeError as e:
                rospy.logerr(f"Failed to save RGB image: {e}")
    def save_depth(self, msg):
        if self.depth_requested:
            try:
                rospy.loginfo("Saving Depth image...")
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                # 데이터 형식 확인 및 정규화
                if depth_image.dtype != np.uint16:
                    rospy.logwarn("Depth image is not 16-bit. Converting to 16-bit.")
                    depth_image = (depth_image * 1000).astype(np.uint16)  # 예: 32FC1일 경우

                # 정규화 (0~255 범위로 변환)
                depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_normalized = depth_normalized.astype(np.uint8)

                # 파일 저장
                depth_filename = self.get_timestamped_filename("depth_image.png")
                cv2.imwrite(depth_filename, depth_normalized)
                rospy.loginfo(f"Depth image saved: {depth_filename}")

                # 저장 요청 플래그 해제
                self.depth_requested = False

            except CvBridgeError as e:
                rospy.logerr(f"Failed to save Depth image: {e}")
            except Exception as e:
                rospy.logerr(f"Unexpected error while saving Depth image: {e}")
    def save_realsense_pcd(self, msg):
        if self.realsense_pcd_requested:
            try:
                rospy.loginfo("Saving Point Cloud (RealSense)...")
                cloud_points = []
                for point in point_cloud2.read_points(msg, skip_nans=True):
                    cloud_points.append([point[0], point[1], point[2]])
                
                # Open3D로 PCD 파일 저장
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points))
                pcd_filename = self.get_timestamped_filename("realsense_point_cloud.pcd")
                o3d.io.write_point_cloud(pcd_filename, pcd)
                rospy.loginfo(f"Point Cloud (RealSense) saved: {pcd_filename}")
                self.realsense_pcd_requested = False
            except Exception as e:
                rospy.logerr(f"Failed to save Point Cloud (RealSense): {e}")
    def save_velodyne_pcd(self, msg):
        if self.velodyne_pcd_requested:
            try:
                rospy.loginfo("Saving Point Cloud (Velodyne)...")
                cloud_points = []
                for point in point_cloud2.read_points(msg, skip_nans=True):
                    cloud_points.append([point[0], point[1], point[2]])
                
                # Open3D로 PCD 파일 저장
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points))
                pcd_filename = self.get_timestamped_filename("velodyne_point_cloud.pcd")
                o3d.io.write_point_cloud(pcd_filename, pcd)
                rospy.loginfo(f"Point Cloud (Velodyne) saved: {pcd_filename}")
                self.velodyne_pcd_requested = False
            except Exception as e:
                rospy.logerr(f"Failed to save Point Cloud (Velodyne): {e}")

if __name__ == "__main__":
    recorder = RealSenseVelodyneRecorder()
    rospy.spin()
    main()
