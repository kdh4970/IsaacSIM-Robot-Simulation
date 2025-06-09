from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QProcess
from PyQt5.QtGui import QFont
# class PresetSelector(QMainWindow):
#     def __init__(self,lst):
#         super().__init__()
#         self.preset_list = lst
#         self.env = None
#         self.robot = None
#         self.select()

#     def select(self):
#         inputDialog = QInputDialog(self)
#         inputDialog.setWindowTitle("Environment Choice")
#         inputDialog.setLabelText("Which preset would you like to use?")
#         inputDialog.setComboBoxItems(self.preset_list)
#         inputDialog.setGeometry(1000, 600, 500, 400)

#         if inputDialog.exec() == QInputDialog.Accepted:
#             item = inputDialog.textValue()
#             print(f'Selected preset: {item}')
#             self.env, self.robot = item.split(" + ")
#         else:
#             print('Canceled')


from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QComboBox, QLabel, QPushButton

class PresetSelector(QDialog):
    def __init__(self,
        env_list: list,
        robot_list: list,
        lidar_list: list,
        sensor_config: dict
    ) -> None:
        super().__init__()
        self.env_combo = QComboBox()
        self.robot_combo = QComboBox()
        self.lidar_combo = QComboBox()
        self.sensor_buttons = {}
        self.sensor_states = {}
        self.sensor_config = sensor_config
        
        self.env_combo.addItems(env_list)
        self.robot_combo.addItems(robot_list)
        self.lidar_combo.addItems(lidar_list)
        
        self.initUI()
    
    def initUI(self) -> None:
        layout = QVBoxLayout()
        
        # 환경 선택
        env_layout = QHBoxLayout()
        env_label = QLabel("Environment:")
        env_label.setStyleSheet("font-size: 16px;")
        env_label.setMinimumHeight(30)
        env_layout.addWidget(env_label)
        env_layout.addWidget(self.env_combo)
        layout.addLayout(env_layout)
        
        # 로봇 선택
        robot_layout = QHBoxLayout()
        robot_label = QLabel("Robot:")
        robot_label.setStyleSheet("font-size: 16px;")
        robot_label.setMinimumHeight(30)
        robot_layout.addWidget(robot_label)
        robot_layout.addWidget(self.robot_combo)
        layout.addLayout(robot_layout)
        
        # 센서 설정 그룹박스
        sensor_group = QGroupBox("Sensor Configuration")
        sensor_group.setStyleSheet("""
            QGroupBox {
                font-size: 18px;
                font-weight: bold;
            }
        """)
        sensor_layout = QVBoxLayout()
        
        for sensor_name, is_enabled in self.sensor_config.items():
            # 센서 이름과 버튼을 가로로 배치
            sensor_row = QHBoxLayout()
            
            # 센서 이름 라벨 (왼쪽)
            sensor_label = QLabel(sensor_name + ":")
            sensor_label.setMinimumWidth(100)
            sensor_label.setStyleSheet("font-size: 16px;")
            sensor_row.addWidget(sensor_label)
            
            # 스페이서 추가 (중간 공간)
            sensor_row.addStretch()
            
            # 센서 버튼 (오른쪽)
            button = QPushButton()
            button.setMinimumWidth(100)
            button.setMinimumHeight(30)
            
            # 초기 상태 설정
            self.sensor_states[sensor_name] = is_enabled
            self.update_button_style(button, is_enabled)
            
            # 버튼 클릭 이벤트 연결
            button.clicked.connect(lambda checked, name=sensor_name: self.toggle_sensor(name))
            
            self.sensor_buttons[sensor_name] = button
            sensor_row.addWidget(button)
            
            sensor_layout.addLayout(sensor_row)
            
            # 라이다 센서인 경우 라이다 모델 선택 콤보박스 추가
            if sensor_name == "Lidar":
                lidar_model_layout = QHBoxLayout()
                lidar_model_label = QLabel("  Model:")
                lidar_model_label.setStyleSheet("font-size: 16px;")
                lidar_model_layout.addWidget(lidar_model_label)
                lidar_model_layout.addStretch()
                lidar_model_layout.addWidget(self.lidar_combo)
                sensor_layout.addLayout(lidar_model_layout)
                
                # 초기 상태에 따라 라이다 모델 콤보박스 활성화/비활성화
                self.lidar_combo.setEnabled(is_enabled)
        
        sensor_group.setLayout(sensor_layout)
        layout.addWidget(sensor_group)
        
        # 확인 버튼
        ok_button = QPushButton('Launch IsaacSIM')
        ok_button.setStyleSheet("""
            QPushButton {
                font-size: 16px;
            }
        """)
        ok_button.clicked.connect(self.accept)
        ok_button.setMinimumHeight(30)
        layout.addWidget(ok_button)
        
        self.setLayout(layout)
        self.setWindowTitle("Simulation Options")
    
    def update_button_style(self, button, is_enabled) -> None:
        """버튼 스타일 업데이트"""
        if is_enabled:
            button.setText("ENABLED")
            button.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #45a049;
                    border-radius: 5px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
            """)
        else:
            button.setText("DISABLED")
            button.setStyleSheet("""
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #da190b;
                    border-radius: 5px;
                }
                QPushButton:hover {
                    background-color: #da190b;
                }
            """)
    
    def toggle_sensor(self, sensor_name) -> None:
        """센서 상태 토글"""
        current_state = self.sensor_states[sensor_name]
        new_state = not current_state
        self.sensor_states[sensor_name] = new_state
        
        button = self.sensor_buttons[sensor_name]
        self.update_button_style(button, new_state)
        
        # 라이다 센서인 경우 라이다 모델 콤보박스 활성화/비활성화
        if sensor_name == "Lidar":
            self.lidar_combo.setEnabled(new_state)
            if not new_state:
                # 라이다가 비활성화되면 첫 번째 항목으로 리셋
                self.lidar_combo.setCurrentIndex(0)
    
    def get_selections(self) -> tuple:
        # 라이다가 비활성화된 경우 라이다 모델은 None으로 반환
        lidar_model = self.lidar_combo.currentText() if self.sensor_states.get("Lidar", False) else None
        
        return (self.env_combo.currentText(), 
                self.robot_combo.currentText(), 
                self.sensor_states.copy(),
                lidar_model)