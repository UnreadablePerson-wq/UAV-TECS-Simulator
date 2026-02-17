# -*- coding: utf-8 -*-
"""
Главное окно программы
Тут вся основная логика симуляции
"""

import numpy as np
from scipy.integrate import solve_ivp
from PyQt5.QtWidgets import (QMainWindow, QVBoxLayout, QHBoxLayout,
                             QSlider, QLabel, QWidget, QPushButton,
                             QApplication, QGroupBox)
from PyQt5.QtCore import Qt, QTimer

from core.vehicle_state import VehicleState
from core.vehicle_dynamics import VehicleDynamics
from core.tecs_controller import TECSController
from core.quaternion_utils import quaternion_to_euler, euler_to_quaternion, quaternion_to_rotation_matrix
from visualization.viewer_3d import VehicleVisualizer
from visualization.plot_utils import plot_results
from gui.control_panel import ControlPanel

class SimulationWindow(QMainWindow):
    """Главное окно"""
    
    def __init__(self):
        super().__init__()
        self.current_time = 0.0
        self.max_time = 0.0
        self.dmax_time = 10.0
        self.dt = 0.02
        self.history_size = 0
        self.iter = 0
        
        self.state = VehicleState()
        self.dynamics = VehicleDynamics()
        self.state_history = None
        
        # контроллер TECS
        self.tecs_controller = TECSController(mass=self.dynamics.mass)
        self.tecs_enabled = False
        
        self._init_ui()
        self._init_simulation()
    
    def _init_ui(self):
        """Собираем интерфейс"""
        self.setWindowTitle("Симулятор БПЛА - Практика 4 (TECS)")
        self.setGeometry(100, 100, 1400, 900)
        
        central_widget = QWidget()
        main_layout = QHBoxLayout(central_widget)
        self.setCentralWidget(central_widget)
        
        # левая часть - 3D
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        self._create_3d_viewer(left_layout)
        self._create_state_label(left_layout)
        
        main_layout.addWidget(left_widget, stretch=3)
        
        # правая часть - панель управления
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        self.control_panel = ControlPanel(self.dynamics)
        self.control_panel.create_control_panel(right_layout)
        self._create_control_buttons(right_layout)
        self._create_sliders(right_layout)
        
        main_layout.addWidget(right_widget, stretch=1)
    
    def _create_3d_viewer(self, layout):
        """Создаем окно 3D"""
        import pyqtgraph.opengl as gl
        import pyqtgraph as pg
        
        self.viewer = gl.GLViewWidget()
        self.viewer.setCameraPosition(distance=200, elevation=30, azimuth=-90)
        self.viewer.setBackgroundColor('k')
        layout.addWidget(self.viewer, stretch=3)
        
        # сетка для ориентации
        grid = gl.GLGridItem()
        grid.setSize(500, 500, 1)
        grid.setSpacing(50, 50, 1)
        self.viewer.addItem(grid)
        
        # оси
        self.viewer.addItem(gl.GLAxisItem(size=pg.QtGui.QVector3D(20, 20, 20)))
        
        self.visualizer = VehicleVisualizer(self.state, self.viewer)
    
    def _create_state_label(self, layout):
        """Строка состояния с параметрами"""
        self.state_label = QLabel("Инициализация...")
        layout.addWidget(self.state_label)
    
    def _create_control_buttons(self, layout):
        """Кнопки управления симуляцией"""
        buttons_layout = QHBoxLayout()
        
        self.start_button = QPushButton("Старт")
        self.start_button.clicked.connect(self.start_simulation)
        buttons_layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Стоп")
        self.stop_button.clicked.connect(self.stop_simulation)
        buttons_layout.addWidget(self.stop_button)
        
        self.plot_button = QPushButton("Графики")
        self.plot_button.clicked.connect(self.show_plots)
        buttons_layout.addWidget(self.plot_button)
        
        self.reset_button = QPushButton("Сброс")
        self.reset_button.clicked.connect(self.reset_simulation)
        buttons_layout.addWidget(self.reset_button)
        
        layout.addLayout(buttons_layout)
    
    def _create_sliders(self, layout):
        """Слайдеры для начальных условий"""
        sliders_group = QGroupBox("Начальные условия")
        sliders_layout = QVBoxLayout(sliders_group)
        
        self.sliders = {}
        slider_config = {
            "north": ("Север", -100, 100),
            "east": ("Восток", -100, 100),
            "altitude": ("Высота", -200, 0),
            "phi": ("Крен (°)", -180, 180),
            "theta": ("Тангаж (°)", -180, 180),
            "psi": ("Рыскание (°)", -180, 180)
        }
        
        for param, (label_text, min_val, max_val) in slider_config.items():
            row = QHBoxLayout()
            label = QLabel(label_text)
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            
            if param == 'altitude':
                slider.setValue(int(self.state.altitude))
            elif param == 'phi':
                slider.setValue(int(np.degrees(self.state.phi)))
            elif param == 'theta':
                slider.setValue(int(np.degrees(self.state.theta)))
            elif param == 'psi':
                slider.setValue(int(np.degrees(self.state.psi)))
            else:
                slider.setValue(int(getattr(self.state, param)))
            
            slider.valueChanged.connect(self.update_from_sliders)
            
            row.addWidget(label)
            row.addWidget(slider)
            sliders_layout.addLayout(row)
            self.sliders[param] = slider
        
        layout.addWidget(sliders_group)
    
    def _init_simulation(self):
        """Настройка таймера"""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.setInterval(int(self.dt * 1000))
        self._update_history_size()
    
    def _update_history_size(self):
        """Вычисляем размер массива для истории"""
        self.history_size = int(self.dmax_time / self.dt) + 1
        self.state_history = np.zeros((16, self.history_size))
    
    def _update_state_label(self):
        """Обновляем информацию в строке состояния"""
        try:
            tecs_status = "ВКЛ" if self.tecs_enabled else "ВЫКЛ"
            
            if hasattr(self.control_panel, 'tecs_altitude_spinbox'):
                tecs_alt_cmd = self.control_panel.tecs_altitude_spinbox.value()
                tecs_vel_cmd = self.control_panel.tecs_velocity_spinbox.value()
            else:
                tecs_alt_cmd = 0
                tecs_vel_cmd = 0
            
            text = (f"Время: {self.current_time:.2f} с | "
                    f"Высота: {-self.state.altitude:.1f} м | "
                    f"Скорость: {self.state.Va:.1f} м/с | "
                    f"α: {np.degrees(self.state.alpha):.1f}° | "
                    f"β: {np.degrees(self.state.beta):.1f}° | "
                    f"TECS: {tecs_status} | "
                    f"Задание: h={-tecs_alt_cmd:.0f}м, V={tecs_vel_cmd:.0f}м/с")
        except:
            text = (f"Время: {self.current_time:.2f} с | "
                    f"Высота: {-self.state.altitude:.1f} м | "
                    f"Скорость: {self.state.Va:.1f} м/с")
        
        self.state_label.setText(text)
    
    def update_from_sliders(self):
        """Обновляем положение по слайдерам"""
        self.state.north = self.sliders["north"].value()
        self.state.east = self.sliders["east"].value()
        self.state.altitude = self.sliders["altitude"].value()
        self.state.phi = np.radians(self.sliders["phi"].value())
        self.state.theta = np.radians(self.sliders["theta"].value())
        self.state.psi = np.radians(self.sliders["psi"].value())
        self.state.e = euler_to_quaternion(self.state.phi, self.state.theta, self.state.psi)
        
        self.visualizer.update(self.state)
    
    def update_simulation(self):
        """Главный цикл симуляции - выполняется каждый шаг"""
        if self.iter >= self.history_size:
            self.timer.stop()
            return
        
        # обновляем ветер
        self.dynamics.update_wind_gusts(self.dt)
        
        # проверяем состояние TECS
        if hasattr(self.control_panel, 'tecs_enable_checkbox'):
            self.tecs_enabled = self.control_panel.tecs_enable_checkbox.isChecked()
            self.tecs_controller.enabled = self.tecs_enabled
            
            # обновляем коэффициенты если поменяли
            if hasattr(self.control_panel, 'tecs_kp_E_spinbox'):
                self.tecs_controller.set_gains(
                    kp_E=self.control_panel.tecs_kp_E_spinbox.value(),
                    ki_E=self.control_panel.tecs_ki_E_spinbox.value(),
                    kp_B=self.control_panel.tecs_kp_B_spinbox.value(),
                    ki_B=self.control_panel.tecs_ki_B_spinbox.value()
                )
        
        # если TECS включен - он управляет
        if self.tecs_enabled and hasattr(self.control_panel, 'tecs_altitude_spinbox'):
            alt_cmd = self.control_panel.tecs_altitude_spinbox.value()
            vel_cmd = self.control_panel.tecs_velocity_spinbox.value()
            
            throttle_cmd, pitch_cmd = self.tecs_controller.update(
                Va=self.state.Va,
                altitude=self.state.altitude,
                Va_cmd=vel_cmd,
                alt_cmd=alt_cmd,
                dt=self.dt
            )
            
            # простой П-регулятор для внутреннего контура
            pitch_error = pitch_cmd - self.state.theta
            delta_e_cmd = np.clip(-2.0 * pitch_error, np.radians(-25), np.radians(25))
            
            self.dynamics.delta[1] = delta_e_cmd
            self.dynamics.delta[3] = throttle_cmd
            
            # показываем в UI что получилось
            if 'delta_e' in self.control_panel.control_controls:
                self.control_panel.control_controls['delta_e'].setValue(np.degrees(delta_e_cmd))
            if 'throttle' in self.control_panel.control_controls:
                self.control_panel.control_controls['throttle'].setValue(throttle_cmd * 100)
        
        # сохраняем в историю
        if self.iter < self.history_size:
            self._save_state_to_history()
        
        # текущее состояние
        current_state = np.array([
            self.state.north, self.state.east, self.state.altitude,
            self.state.u, self.state.v, self.state.w,
            self.state.p, self.state.q, self.state.r,
            self.state.e[0], self.state.e[1], self.state.e[2], self.state.e[3]
        ])
        
        # интегрируем
        try:
            solution = solve_ivp(
                self.dynamics.derivatives_dt,
                [self.current_time, self.current_time + self.dt],
                current_state,
                method='RK45',
                t_eval=[self.current_time + self.dt],
                rtol=1e-6,
                atol=1e-8
            )
            new_state = solution.y[:, -1]
        except Exception as e:
            print(f"Ошибка при интегрировании: {e}")
            self.timer.stop()
            return
        
        self._update_state_from_solution(new_state)
        
        # ветер для визуализации
        R = quaternion_to_rotation_matrix(self.state.e)
        wind_inertial = self.dynamics.wind_ned + R @ self.dynamics.gust_body
        
        self.visualizer.update(self.state, wind_inertial)
        self._update_state_label()
        
        self.current_time += self.dt
        self.iter += 1
    
    def _save_state_to_history(self):
        """Сохраняем текущее состояние в массив истории"""
        if self.iter < self.history_size:
            state_vars = [
                self.state.north, self.state.east, self.state.altitude,
                self.state.phi, self.state.theta, self.state.psi,
                self.state.u, self.state.v, self.state.w,
                self.state.p, self.state.q, self.state.r,
                self.state.e[0], self.state.e[1], self.state.e[2], self.state.e[3]
            ]
            
            for i, var in enumerate(state_vars):
                self.state_history[i][self.iter] = var
    
    def _update_state_from_solution(self, new_state):
        """Обновляем переменные состояния из решения ОДУ"""
        self.state.north, self.state.east, self.state.altitude = new_state[0:3]
        self.state.u, self.state.v, self.state.w = new_state[3:6]
        self.state.p, self.state.q, self.state.r = new_state[6:9]
        
        # нормализуем кватернион
        e = new_state[9:13]
        e_norm = np.sqrt(e[0]**2 + e[1]**2 + e[2]**2 + e[3]**2)
        if e_norm > 0:
            self.state.e = e / e_norm
        else:
            self.state.e = np.array([1.0, 0.0, 0.0, 0.0])
        
        self.state.phi, self.state.theta, self.state.psi = quaternion_to_euler(self.state.e)
        
        # обновляем аэродинамические параметры
        state_vec = np.concatenate([new_state[:9], self.state.e])
        _, _, alpha, beta, Va, _ = self.dynamics.calculate_aerodynamic_forces(state_vec)
        self.state.alpha = alpha
        self.state.beta = beta
        self.state.Va = Va
    
    def start_simulation(self):
        """Запуск"""
        if not self.timer.isActive():
            self.max_time = self.current_time + self.dmax_time
            self.iter = 0
            self._update_history_size()
            self.timer.start()
    
    def stop_simulation(self):
        """Остановка"""
        self.timer.stop()
    
    def reset_simulation(self):
        """Сброс всего"""
        self.timer.stop()
        self.state.reset()
        
        self.dynamics = VehicleDynamics()
        
        if hasattr(self, 'control_panel'):
            self.control_panel.dynamics = self.dynamics
            self.control_panel.reset_all_to_defaults()
        
        self.tecs_controller.reset()
        self.tecs_enabled = False
        
        if hasattr(self, 'sliders'):
            self.sliders["north"].setValue(0)
            self.sliders["east"].setValue(0)
            self.sliders["altitude"].setValue(int(self.state.altitude))
            self.sliders["phi"].setValue(int(np.degrees(self.state.phi)))
            self.sliders["theta"].setValue(int(np.degrees(self.state.theta)))
            self.sliders["psi"].setValue(int(np.degrees(self.state.psi)))
        
        self.current_time = 0.0
        self.max_time = 0.0
        self.iter = 0
        self._update_history_size()
        
        self.visualizer.update(self.state)
        self._update_state_label()
    
    def show_plots(self):
        """Показываем графики"""
        self.timer.stop()
        if self.state_history is not None and self.iter > 0:
            plot_results(0, self.dt, self.current_time, self.state_history[:, :self.iter])