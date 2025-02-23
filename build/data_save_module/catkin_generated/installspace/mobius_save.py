import base64
import cv_bridge
import rospy
import requests
import cv2
import numpy as np
import open3d as o3d
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from datetime import datetime  # 타임스탬프 추가
from std_msgs.msg import String


class DataUploader:
    def __init__(self):
        rospy.init_node("mobius_save", anonymous=True)

        self.bridge = cv_bridge.CvBridge()
        self.latest_rgb_bytes = None
        self.latest_depth_bytes = None
        self.latest_realsense_pcd_bytes = None
        self.latest_velodyne_pcd_bytes = None

        self.mobius_url = "http://114.71.220.59:7579/Mobius/AIoT_Test"

        self.headers = {
            "X-M2M-Origin": "S",
            "X-M2M-RI": "12345",
            "Content-Type": "application/json;ty=4",
            "Accept": "application/json",
        }

        rospy.Subscriber("/camera/color/image_raw", Image, self.handle_rgb)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.handle_depth)
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.handle_realsense_pcd)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.handle_velodyne_pcd)
        rospy.Subscriber("/sensor_command", String, self.sensor_command_callback)
        
        
    def sensor_command_callback(self, msg):
        command = msg.data
        if "save" in command:
            rospy.loginfo("Success Connection...")
            self.upload_all()

    def handle_rgb(self, msg):
        try:
            self.latest_rgb_img = base64.b64encode(self.bridge.imgmsg_to_cv2(msg, "bgr8")).decode("utf-8")
        except Exception as e:
            rospy.logerr("Failed to process RGB image: %s", str(e))

    def handle_depth(self, msg):
        try:
            self.latest_depth_img = base64.b64encode(self.bridge.imgmsg_to_cv2(msg, "16UC1")).decode("utf-8")
        except Exception as e:
            rospy.logerr("Failed to process Depth image: %s", str(e))

    def handle_realsense_pcd(self, msg):
        self.latest_realsense_pcd_bytes = self.convert_pcd_to_bytes(msg)

    def handle_velodyne_pcd(self, msg):
        self.latest_velodyne_pcd_bytes = self.convert_pcd_to_bytes(msg)

    def convert_pcd_to_bytes(self, msg):
        """PointCloud2 메시지를 PCD 바이너리 형식으로 변환하여 bytes로 반환"""
        try:
            # PointCloud2에서 NumPy 배열로 변환
            points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)

            if points.size == 0:
                rospy.logwarn("Received empty point cloud message")
                return None

            # PCD 헤더 생성
            header = f"""# .PCD v0.7 - Point Cloud Data file format
                VERSION 0.7
                FIELDS x y z
                SIZE 4 4 4
                TYPE F F F
                COUNT 1 1 1
                WIDTH {len(points)}
                HEIGHT 1
                VIEWPOINT 0 0 0 1 0 0 0
                POINTS {len(points)}
                DATA binary
            """

            header_bytes = header.encode("ascii")  # 헤더를 ASCII로 변환
            data_bytes = points.tobytes()  # NumPy 배열을 바이너리로 변환

            # PCD 바이너리 형식으로 반환
            return header_bytes + data_bytes  # 헤더 + 데이터 합치기
        except Exception as e:
            rospy.logerr(f"Failed to convert PCD: {e}")
            return None

    def create_container(self):
        """ 서버에 타임스탬프 기반 컨테이너 생성 """
        # 타임스탬프 기반으로 컨테이너 이름 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        container_name = f"{timestamp}"

        url = f"{self.mobius_url}"
        payload = {"m2m:cnt": {"rn": container_name, "mbs": 4000000000}}
        response = requests.post(url, json=payload, headers=self.headers)

        if response.status_code in [200, 201]:
            rospy.loginfo(f"Container {container_name} created successfully")
            return f"{url}/{container_name}"
        else:
            rospy.logerr(f"Failed to create container: {response.text}")
            return None


    def upload_all(self):
        if self.latest_rgb_img is None or self.latest_depth_img is None:
            rospy.logwarn("No RGB or Depth data available for upload")
            return

        container_url = self.create_container()
        if not container_url:
            return

        data_rgb_image = {
            "m2m:cin": {
                "con": self.latest_rgb_img,
                "cnf": "image/png",
                "rn": "rgb_image",
            }
        }
        response = requests.post(container_url, json=data_rgb_image, headers=self.headers)
        if response.status_code == 201:
            rospy.loginfo("RGB image uploaded successfully")
        else:
            rospy.logerr(f"Failed to upload RGB image: {response.text}")

        
        data_depth_image = {
            "m2m:cin": {
                "con": self.latest_depth_img,
                "cnf": "image/png",
                "rn": "depth_image",
            }
        }
        response = requests.post(container_url, json=data_depth_image, headers=self.headers)
        if response.status_code == 201:
            rospy.loginfo("Depth image uploaded successfully")
        else:
            rospy.logerr(f"Failed to upload Depth image: {response.text}")

        data_realsense_pcd = {
            "m2m:cin": {
                "con": self.latest_depth_img,
                "cnf": "application/octet-stream",
                "rn": "realsense_pcd",
            }
        }
        if self.latest_realsense_pcd_bytes:
            response = requests.post(container_url, json=data_realsense_pcd, headers=self.headers)
            if response.status_code == 201:
                rospy.loginfo("Realsense PCD uploaded successfully")
            else:
                rospy.logerr(f"Failed to upload Realsense PCD: {response.text}")

        data_velodyne_pcd = {
            "m2m:cin": {
                "con": self.latest_depth_img,
                "cnf": "application/octet-stream",
                "rn": "velodyne_pcd",
            }
        }
        if self.latest_velodyne_pcd_bytes:
            response = requests.post(container_url, json=data_velodyne_pcd, headers=self.headers)
            if response.status_code == 201:
                rospy.loginfo("Velodyne PCD uploaded successfully")
            else:
                rospy.logerr(f"Failed to upload Velodyne PCD: {response.text}")


if __name__ == "__main__":
    uploader = DataUploader()
    rospy.spin()
