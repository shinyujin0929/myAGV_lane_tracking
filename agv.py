import threading
import time
import cv2
import cv2.aruco as aruco
import numpy as np
import socket
from pymycobot.myagv import MyAgv

# Thread Lock for AGV actions
state = threading.Lock()

# Constants
MIN_CONTOUR_AREA = 500
LARGE_KERNEL = np.ones((5, 5), np.uint8)
OFFSET_THRESHOLD = 120
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
ARUCO_PARAMETERS = aruco.DetectorParameters()
HOST = "172.30.1.56"
PORT = 9007



class AGVSocketClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        try:
            self.client_socket.connect((self.host, self.port))
            print(f"Connected to server at {self.host}:{self.port}")
        except Exception as e:
            print(f"Connection failed: {e}")

    def send_command(self, command):
        try:
            self.client_socket.sendall(command.encode())
            print(f"Sent command: {command}")
        except Exception as e:
            print(f"Send data error: {e}")

    def recv_command(self):
        try:
            response = self.client_socket.recv(1024).decode()
            print(f"Server response: {response}")
            return response
        except Exception as e:
            print(f"Receive data error: {e}")
            return None

    def close(self):
        self.client_socket.close()


class AGVController:
    def __init__(self, port, baudrate):
        self.agv = MyAgv(port, baudrate)
        self.agv_client = AGVSocketClient(HOST, PORT)
        self.stop_signal = False

    def stop(self):
        self.agv.stop()
        print("정지 신호 탐지")
    
    def loading_process(self):
        self.agv.stop()
        time.sleep(1)
        self.agv.pan_right(35, 0.8)
        self.agv.clockwise_rotation(80, 3.1)
        self.agv.stop()
        time.sleep(1)
        print("적재 destination 신호")
        self.agv_client.connect()
        self.agv_client.send_command("Arrived")

        if self.agv_client.recv_command() == "START":
            self.agv.pan_right(35, 0.8)
            return 

    def unloading_process(self):
        self.agv.stop()
        time.sleep(1)
        self.agv.pan_left(35, 0.8)
        self.agv.clockwise_rotation(80, 3.1)
        self.agv.stop()
        time.sleep(1)
        print("하역 destination 신호")
        #self.agv_client.connect()
        self.agv_client.send_command("Arrived")

        if self.agv_client.recv_command() == "START":
            self.agv.pan_left(35, 0.8)
            return 
    
    def finish_process(self):
        self.agv.stop()
        time.sleep(1)
        self.agv.pan_left(35,1.5)
        self.agv.counterclockwise_rotation(80, 3.1)
        self.agv.stop()
        print("출발지 도착 신호")



    def agv_control(self, offset, last_offset):
        with state:
            if offset is None:
                offset = last_offset
                if offset is None:
                    print("라인 정보 없음")
                    return

            # 이동 논리
            if abs(offset) > OFFSET_THRESHOLD:
                speed_factor = min(4.0, abs(offset) / 100)
                rotation_speed = int(np.clip(30 * speed_factor, 1, 127))
                if offset < 0:
                    print(f"왼쪽 회전: {rotation_speed}")
                    self.agv.counterclockwise_rotation(rotation_speed , 0.2)    #15
                else:
                    print(f"오른쪽 회전: {rotation_speed}")
                    self.agv.clockwise_rotation(rotation_speed , 0.2)
            else:
                speed_factor = max(0.8, min(1.0, abs(offset) / 150))
                go_ahead_speed = int(np.clip(15 / speed_factor, 1, 127))
                print(f"직진: {go_ahead_speed}")
                self.agv.go_ahead(min(go_ahead_speed + 5, 127), 0.15)   #60 (40->20)


class CameraHandler:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        

    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("카메라 오류 발생")
            return None
        return frame

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


class ArUcoDetector:
    def __init__(self):
        self.agv_contorller = AGVController("/dev/ttyAMA2", 115200)
        self.camera_matrix = np.load(r"project3_aruco/Image/camera_matrix.npy")
        self.dist_coeffs = np.load(r"project3_aruco/Image/dist_coeffs.npy")
        self.stop_signal = False
        self.destination1_signal = False
        self.destiantion2_signal = False
        self.finish_signal = False
        self.skip_frames_count = 0  # 건너뛸 프레임 수
        self.skip_frames_limit = 7  # 건너뛸 프레임 수 제한

    def detect_markers(self, frame):
        # 마커 인식 후 건너뛰는 상태 처리
        if self.skip_frames_count > 0:
            self.skip_frames_count -= 1
            #print(f"Skipping frame: {self.skip_frames_count} remaining")
            return  # 건너뛰기 중이면 처리하지 않음

        # 아루코 마커 탐지
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        if ids is not None:
            for i in range(len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.07, self.camera_matrix, self.dist_coeffs)
                distance = np.linalg.norm(tvec)
                aruco.drawDetectedMarkers(frame, corners)
   
                if distance <= 0.4:
                    if ids[i][0] in [7, 3, 4, 6]:
                        self.stop_signal = True
                        self.agv_contorller.stop()
                        time.sleep(12.5)
                        self.stop_signal = False
                    elif ids[i][0] in [2, 11] and not self.destination1_signal:     #2 spare:11
                        self.destination1_signal = True
                        self.agv_contorller.loading_process()
                    elif ids[i][0] in [5, 12] and not self.destiantion2_signal:     #5 spare:12
                        self.destiantion2_signal = True
                        self.agv_contorller.unloading_process()
                    elif ids[i][0] in [0] and not self.finish_signal:
                        self.finish_signal = True
                        self.agv_contorller.finish_process()
                    


                # 마커 인식 후 프레임 건너뛰기 시작
                self.skip_frames_count = self.skip_frames_limit
                #print("ArUco marker detected. Skipping next frames.")
        else:
            self.destination1_signal = False
            self.destiantion2_signal = False
            self.finish_signal = False



class LineDetector:
    def __init__(self):
        self.last_offset = None

    def process_frame(self, frame):
        height, width, _ = frame.shape
        roi = frame[3* height // 4:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 50, 50], dtype=np.uint8)
        upper_blue = np.array([130, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        morphed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, LARGE_KERNEL)

        contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(largest_contour)
                line_center = x + w // 2
                offset = line_center - (width // 2)

                # 라인 인식 시각화
                cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.line(roi, (line_center, y), (line_center, y + h), (255, 0, 0), 2)
                return offset
        return None


class MainSystem:
    def __init__(self):
        self.agv_controller = AGVController("/dev/ttyAMA2", 115200)
        self.socket_client = AGVSocketClient(HOST, PORT)
        self.camera_handler = CameraHandler()
        self.aruco_detector = ArUcoDetector()
        self.line_detector = LineDetector()

    def main_loop(self):
        try:
            while True:
                frame = self.camera_handler.read_frame()
                if frame is None:
                    break

                self.aruco_detector.detect_markers(frame)

                offset = self.line_detector.process_frame(frame)
                if offset is not None:
                    self.line_detector.last_offset = offset

                if not self.agv_controller.stop_signal:
                    agv_control_thread = threading.Thread(target= self.agv_controller.agv_control, args=(offset, self.line_detector.last_offset))
                    agv_control_thread.start()
                    agv_control_thread.join()
                    #self.agv_controller.agv_control(offset, self.line_detector.last_offset)

                cv2.imshow("Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.agv_controller.agv.stop()
                    break
        finally:
            self.camera_handler.release()
            self.socket_client.close()

    def run(self):
        self.main_loop()

if __name__ == "__main__":
    system = MainSystem()
    system.run()
