from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QWidget, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
import cv2
import sys


# Worker Thread for AGV and Camera Processing
class AGVWorker(QThread):
    frame_ready = pyqtSignal(QImage)

    def __init__(self, system):
        super().__init__()
        self.system = system
        self.running = True

    def run(self):
        while self.running:
            # 1. 카메라 프레임 읽기
            frame = self.system.camera_handler.read_frame()
            if frame is not None:
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, channel = rgb_frame.shape
                bytes_per_line = 3 * width
                q_image = QImage(rgb_frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
                self.frame_ready.emit(q_image)

            # 2. AGV 관련 동작 수행
            self.system.aruco_detector.detect_markers(frame)
            offset = self.system.line_detector.process_frame(frame)

            if offset is not None:
                self.system.line_detector.last_offset = offset

            if not self.system.agv_controller.stop_signal:
                self.system.agv_controller.agv_control(offset, self.system.line_detector.last_offset)

    def stop(self):
        self.running = False
        self.quit()
        self.wait()


# Main Application
class AGVControlApp(QMainWindow):
    def __init__(self, system):
        super().__init__()
        self.system = system
        self.init_ui()
        self.worker_thread = None  # Initialize with no worker thread

    def init_ui(self):
        self.setWindowTitle("AGV Control System")
        self.setGeometry(100, 100, 800, 600)

        # Apply QSS
        self.setStyleSheet(self.get_stylesheet())

        # Video display
        self.video_label = QLabel(self)
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("border: 2px solid #4CAF50; border-radius: 10px; background-color: #000000;")

        # Buttons
        self.load_button = QPushButton("작업 Start")
        self.load_button.clicked.connect(self.start_system)

        #self.unload_button = QPushButton("하역 Start")
        #self.unload_button.clicked.connect(self.start_system)

        self.stop_button = QPushButton("긴급정지")
        self.stop_button.clicked.connect(self.stop_system)
        self.stop_button.setEnabled(False)
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #FF4C4C;  /* 빨간색 */
                color: white;
                border: none;
                border-radius: 10px;
                padding: 10px 20px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #FF0000;  /* 더 진한 빨간색 */
            }
            QPushButton:disabled {
                background-color: #FFA3A3;  /* 연한 빨간색 */
                color: #FFFFFF;
            }
        """)

        self.quit_button = QPushButton("Quit")
        self.quit_button.clicked.connect(self.quit_app)
        self.quit_button.setStyleSheet("""
            QPushButton {
                background-color: #808080;  /* 회색 */
                color: white;
                border: none;
                border-radius: 10px;
                padding: 10px 20px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #696969;  /* 더 진한 회색 */
            }
            QPushButton:disabled {
                background-color: #C0C0C0;  /* 연한 회색 */
                color: #FFFFFF;
            }
        """)

        # Layouts
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.load_button)
        #button_layout.addWidget(self.unload_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.quit_button)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)  # Center-align the video label
        main_layout.addLayout(button_layout)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def get_stylesheet(self):
        return """
        QMainWindow {
            background-color: #2d2d2d;
            color: #ffffff;
        }
        QLabel {
            color: #ffffff;
            font-size: 16px;
        }
        QPushButton {
            background-color: #4CAF50;
            color: #ffffff;
            border: none;
            border-radius: 10px;
            padding: 10px 20px;
            font-size: 14px;
        }
        QPushButton:hover {
            background-color: #45a049;
        }
        QPushButton:disabled {
            background-color: #a5d6a7;
            color: #8a8a8a;
        }
        """

    def start_system(self):
        # Reset AGV stop_signal to False
        self.system.agv_controller.stop_signal = False

        # Enable/Disable buttons
        self.load_button.setEnabled(False)
        #self.unload_button.setEnabled(False)
        self.stop_button.setEnabled(True)

        # If a worker thread already exists, stop it before creating a new one
        if self.worker_thread and self.worker_thread.isRunning():
            self.worker_thread.stop()

        # Create a new worker thread and start it
        self.worker_thread = AGVWorker(self.system)
        self.worker_thread.frame_ready.connect(self.update_frame)
        self.worker_thread.start()

    def stop_system(self):
        # Set stop_signal to True to halt AGV operations
        self.system.agv_controller.stop_signal = True

        # Enable/Disable buttons
        self.load_button.setEnabled(True)
        #self.unload_button.setEnabled(True)
        self.stop_button.setEnabled(False)

        # Stop the worker thread
        if self.worker_thread:
            self.worker_thread.stop()

    def quit_app(self):
        # Stop the worker thread if it exists
        if self.worker_thread:
            self.worker_thread.stop()
        self.close()

    def update_frame(self, q_image):
        self.video_label.setPixmap(QPixmap.fromImage(q_image))


if __name__ == "__main__":
    from project3_agv import MainSystem  # Replace with your system class import

    app = QApplication(sys.argv)
    main_system = MainSystem()
    agv_app = AGVControlApp(main_system)
    agv_app.show()
    sys.exit(app.exec_())
