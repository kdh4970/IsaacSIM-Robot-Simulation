from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

class PresetSelector(QDialog):
    def __init__(self, env_list, robot_list, lidar_list, sensor_config):
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

        # 성능 옵션 및 dt 입력창 추가
        self.performance_combo = QComboBox()
        self.performance_combo.addItems(['Sync with Real-time', 'Best performance'])
        self.physics_per_second_edit = QLineEdit()
        self.physics_per_second_edit.setText("100")
        self.render_per_second_edit = QLineEdit()
        self.render_per_second_edit.setText("30")

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # 환경 선택
        env_layout = QHBoxLayout()
        env_label = QLabel("Environment:")
        env_label.setStyleSheet("font-size: 16px;")
        env_layout.addWidget(env_label)
        env_layout.addWidget(self.env_combo)
        layout.addLayout(env_layout)

        # 로봇 선택
        robot_layout = QHBoxLayout()
        robot_label = QLabel("Robot:")
        robot_label.setStyleSheet("font-size: 16px;")
        robot_layout.addWidget(robot_label)
        robot_layout.addWidget(self.robot_combo)
        layout.addLayout(robot_layout)

        perf_layout = QHBoxLayout()
        perf_label = QLabel("Performance mode:")
        perf_label.setStyleSheet("font-size: 16px;")
        perf_layout.addWidget(perf_label)
        perf_layout.addWidget(self.performance_combo)
        layout.addLayout(perf_layout)


        dt_layout = QHBoxLayout()
        dt_label1 = QLabel("Physics steps per second:")
        dt_label1.setStyleSheet("font-size: 16px;")
        dt_layout.addWidget(dt_label1)
        self.physics_per_second_edit.setFixedWidth(50)
        dt_layout.addWidget(self.physics_per_second_edit)
        layout.addLayout(dt_layout)
        
        dt_layout2 = QHBoxLayout()
        dt_label2 = QLabel("Render steps per second:")
        dt_label2.setStyleSheet("font-size: 16px;")
        dt_layout2.addWidget(dt_label2)
        self.render_per_second_edit.setFixedWidth(50)
        dt_layout2.addWidget(self.render_per_second_edit)
        layout.addLayout(dt_layout2)


        # 센서 설정 그룹박스
        sensor_group = QGroupBox("Sensor Configuration")
        sensor_group.setStyleSheet("""
            QGroupBox {
                font-size: 18px;
                font-weight: bold;
            }
        """)
        sensor_layout = QVBoxLayout()


        # 센서 버튼 및 라이다 모델
        for sensor_name, is_enabled in self.sensor_config.items():
            sensor_row = QHBoxLayout()
            sensor_label = QLabel(sensor_name + ":")
            sensor_label.setMinimumWidth(100)
            sensor_label.setStyleSheet("font-size: 16px;")
            sensor_row.addWidget(sensor_label)
            sensor_row.addStretch()
            button = QPushButton()
            button.setMinimumWidth(100)
            button.setMinimumHeight(30)
            self.sensor_states[sensor_name] = is_enabled
            self.update_button_style(button, is_enabled)
            button.clicked.connect(lambda checked, name=sensor_name: self.toggle_sensor(name))
            self.sensor_buttons[sensor_name] = button
            sensor_row.addWidget(button)
            sensor_layout.addLayout(sensor_row)

            if sensor_name == "Lidar":
                lidar_model_layout = QHBoxLayout()
                lidar_model_label = QLabel("  Model:")
                lidar_model_label.setStyleSheet("font-size: 16px;")
                lidar_model_layout.addWidget(lidar_model_label)
                lidar_model_layout.addStretch()
                lidar_model_layout.addWidget(self.lidar_combo)
                sensor_layout.addLayout(lidar_model_layout)
                self.lidar_combo.setEnabled(is_enabled)

        sensor_group.setLayout(sensor_layout)
        layout.addWidget(sensor_group)

        # 확인 버튼
        ok_button = QPushButton('Launch IsaacSIM')
        ok_button.setStyleSheet("font-size: 16px;")
        ok_button.clicked.connect(self.accept)
        ok_button.setMinimumHeight(30)
        layout.addWidget(ok_button)

        self.setLayout(layout)
        self.setWindowTitle("Simulation Options")

        # (1) 로봇 콤보박스 변경 시 TfOdom 처리
        self.robot_combo.currentTextChanged.connect(self.handle_robot_change)
        self.handle_robot_change(self.robot_combo.currentText())

    def handle_robot_change(self, robot_name):
        # TfOdom 버튼 비활성화 및 DISABLED 표시
        if "TfOdom" in self.sensor_buttons:
            btn = self.sensor_buttons["TfOdom"]
            if robot_name == "unitree_g1":
                btn.setEnabled(False)
                self.sensor_states["TfOdom"] = False
                self.update_button_style(btn, False)
            else:
                btn.setEnabled(True)
                # 기존 상태로 복원
                self.update_button_style(btn, self.sensor_states["TfOdom"])

    # ... (update_button_style, toggle_sensor 등 나머지 함수는 기존과 동일)

    def get_selections(self):
        lidar_model = self.lidar_combo.currentText() if self.sensor_states.get("Lidar", False) else None
        return (self.env_combo.currentText(),
                self.robot_combo.currentText(),
                self.performance_combo.currentText(),
                int(self.physics_per_second_edit.text()),
                int(self.render_per_second_edit.text()),
                self.sensor_states.copy(),
                lidar_model)
    
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