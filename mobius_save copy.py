import requests
import json
import time
import os
import base64


# 기본 서버 URL
server_base_url = "http://114.71.220.59:7579/Mobius/AIoT_Test"

headers = {
    "X-M2M-Origin": "S",
    "X-M2M-RI": "12345",
    "Content-Type": "application/json;ty=4",
    "Accept": "application/json"
}

# 컨테이너 생성 함수
def create_container(container_name):
    url = f"{server_base_url}"
    
    # 컨테이너 메타데이터 설정
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

# 파일 업로드 함수
def upload_file(container_url, file_path):
    if not os.path.exists(file_path):
        print(f"File '{file_path}' does not exist.")
        return

    # 파일 이름과 확장자 추출
    file_name, file_extension = os.path.splitext(os.path.basename(file_path))
    
    # 파일 확장자에 따라 콘텐츠 타입 설정
    if file_extension.lower() == '.pcd':
        content_type = 'application/octet-stream'
        resource_name = 'pcd'
    elif file_extension.lower() == '.tiff':
        content_type = 'image/tiff'
        resource_name = 'png'
    else:
        print(f"Unsupported file type: {file_extension}")

        return
    

    with open(file_path, 'rb') as file:
        file_content = file.read()
        encoded_content = base64.b64encode(file_content).decode('utf-8')  # 바이너리 데이터를 Base64로 인코딩

        data = {
            "m2m:cin": {
                "con": encoded_content, 
                "cnf": content_type,  
                "rn": resource_name,  
            }
        }
        
        response = requests.post(container_url, headers=headers, json=data)
        if response.status_code == 201:
            print(f"File '{file_path}' uploaded successfully to container '{container_url}'.")
        else:
            print(f"Failed to upload file '{file_path}' to container '{container_url}'. Status code: {response.status_code}, Response: {response.text}")

# 메인 함수
def main():
    # 현재 시간을 기반으로 컨테이너 이름 생성
    timestamp = int(time.time())
    container_name = f"{timestamp}"
    
    # 컨테이너 URL 설정
    container_url = f"{server_base_url}/{container_name}".rstrip('/')
    print(f"Container URL: {container_url}")  # 컨테이너 URL 출력
    
    # 파일 경로 설정
    pcd_file_path = "/home/ubicomp/DataPipeline/live_pcd/1723645054041006.pcd"
    rgbd_file_path = "/home/ubicomp/realsense/captured_image_0.png"

    # 컨테이너 생성
    if create_container(container_name):
        # 파일 업로드
        upload_file(container_url, pcd_file_path)
        upload_file(container_url, rgbd_file_path)

# 메인 함수 실행
if __name__ == "__main__":
    main()