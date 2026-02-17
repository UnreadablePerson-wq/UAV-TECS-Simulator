# -*- coding: utf-8 -*-
"""
Панель управления - все ползунки и кнопки в одном месте
"""

import numpy as np
from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QSlider, QLabel, 
                             QDoubleSpinBox, QPushButton, QGroupBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer

class ControlPanel:
    """Все элементы управления"""
    
    def __init__(self, dynamics):
        self.dynamics = dynamics
        self.control_controls = {}
        self.wind_controls = {}
        self.moments_controls = {}
        
        self.test_timer = QTimer()
        self.test_timer.timeout.connect(self._reset_test_values)
    
    def create_control_panel(self, layout):
        """Собираем панельку"""
        
        # ---------- TECS ----------
        tecs_group = QGroupBox("Управление полной энергией (TECS)")
        tecs_layout = QVBoxLayout(tecs_group)
        
        tecs_enable_row = QHBoxLayout()
        self.tecs_enable_checkbox = QCheckBox("Включить TECS")
        tecs_enable_row.addWidget(self.tecs_enable_checkbox)
        tecs_layout.addLayout(tecs_enable_row)
        
        # задание высоты и скорости
        setpoint_row1 = QHBoxLayout()
        setpoint_row1.addWidget(QLabel("Заданная высота (м):"))
        self.tecs_altitude_spinbox = QDoubleSpinBox()
        self.tecs_altitude_spinbox.setRange(-500, 500)
        self.tecs_altitude_spinbox.setValue(-100)
        self.tecs_altitude_spinbox.setSingleStep(10)
        setpoint_row1.addWidget(self.tecs_altitude_spinbox)
        tecs_layout.addLayout(setpoint_row1)
        
        setpoint_row2 = QHBoxLayout()
        setpoint_row2.addWidget(QLabel("Заданная скорость (м/с):"))
        self.tecs_velocity_spinbox = QDoubleSpinBox()
        self.tecs_velocity_spinbox.setRange(10, 50)
        self.tecs_velocity_spinbox.setValue(25)
        self.tecs_velocity_spinbox.setSingleStep(1)
        setpoint_row2.addWidget(self.tecs_velocity_spinbox)
        tecs_layout.addLayout(setpoint_row2)
        
        # коэффициенты
        gains_row1 = QHBoxLayout()
        gains_row1.addWidget(QLabel("Kp_E:"))
        self.tecs_kp_E_spinbox = QDoubleSpinBox()
        self.tecs_kp_E_spinbox.setRange(0, 0.1)
        self.tecs_kp_E_spinbox.setValue(0.015)
        self.tecs_kp_E_spinbox.setSingleStep(0.001)
        gains_row1.addWidget(self.tecs_kp_E_spinbox)
        tecs_layout.addLayout(gains_row1)
        
        gains_row2 = QHBoxLayout()
        gains_row2.addWidget(QLabel("Ki_E:"))
        self.tecs_ki_E_spinbox = QDoubleSpinBox()
        self.tecs_ki_E_spinbox.setRange(0, 0.01)
        self.tecs_ki_E_spinbox.setValue(0.002)
        self.tecs_ki_E_spinbox.setSingleStep(0.0005)
        gains_row2.addWidget(self.tecs_ki_E_spinbox)
        tecs_layout.addLayout(gains_row2)
        
        gains_row3 = QHBoxLayout()
        gains_row3.addWidget(QLabel("Kp_B:"))
        self.tecs_kp_B_spinbox = QDoubleSpinBox()
        self.tecs_kp_B_spinbox.setRange(0, 0.2)
        self.tecs_kp_B_spinbox.setValue(0.05)
        self.tecs_kp_B_spinbox.setSingleStep(0.005)
        gains_row3.addWidget(self.tecs_kp_B_spinbox)
        tecs_layout.addLayout(gains_row3)
        
        gains_row4 = QHBoxLayout()
        gains_row4.addWidget(QLabel("Ki_B:"))
        self.tecs_ki_B_spinbox = QDoubleSpinBox()
        self.tecs_ki_B_spinbox.setRange(0, 0.02)
        self.tecs_ki_B_spinbox.setValue(0.005)
        self.tecs_ki_B_spinbox.setSingleStep(0.001)
        gains_row4.addWidget(self.tecs_ki_B_spinbox)
        tecs_layout.addLayout(gains_row4)
        
        # кнопки для тестов
        test_tecs_layout = QHBoxLayout()
        test_step_alt_btn = QPushButton("Тест: ступенька высоты")
        test_step_alt_btn.clicked.connect(self.test_tecs_step_altitude)
        test_step_vel_btn = QPushButton("Тест: ступенька скорости")
        test_step_vel_btn.clicked.connect(self.test_tecs_step_velocity)
        test_both_btn = QPushButton("Тест: оба канала")
        test_both_btn.clicked.connect(self.test_tecs_both)
        
        test_tecs_layout.addWidget(test_step_alt_btn)
        test_tecs_layout.addWidget(test_step_vel_btn)
        test_tecs_layout.addWidget(test_both_btn)
        tecs_layout.addLayout(test_tecs_layout)
        
        layout.addWidget(tecs_group)
        
        # ---------- Ручное управление ----------
        control_group = QGroupBox("Ручное управление")
        control_layout = QVBoxLayout(control_group)
        
        control_config = [
            ("throttle", "Дроссель (%)", 0, 100, 70),
            ("delta_a", "Элероны (°)", -25, 25, 0),
            ("delta_e", "Руль высоты (°)", -25, 25, -3),
            ("delta_r", "Руль направления (°)", -25, 25, 0),
        ]
        
        for key, label_text, min_val, max_val, default_val in control_config:
            row = QHBoxLayout()
            label = QLabel(f"{label_text}:")
            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setValue(default_val)
            spinbox.setSingleStep(1)
            spinbox.valueChanged.connect(self.update_controls)
            
            row.addWidget(label)
            row.addWidget(spinbox)
            control_layout.addLayout(row)
            self.control_controls[key] = spinbox
        
        # тестовые кнопки
        test_layout = QHBoxLayout()
        test_elevator_btn = QPushButton("Тест: руль высоты")
        test_elevator_btn.clicked.connect(self.test_elevator)
        test_aileron_btn = QPushButton("Тест: элероны")
        test_aileron_btn.clicked.connect(self.test_aileron)
        test_rudder_btn = QPushButton("Тест: руль напр.")
        test_rudder_btn.clicked.connect(self.test_rudder)
        
        test_layout.addWidget(test_elevator_btn)
        test_layout.addWidget(test_aileron_btn)
        test_layout.addWidget(test_rudder_btn)
        control_layout.addLayout(test_layout)
        
        layout.addWidget(control_group)
        
        # ---------- Ветер ----------
        wind_group = QGroupBox("Ветер (м/с)")
        wind_layout = QVBoxLayout(wind_group)
        
        wind_config = [
            ("wind_n", "Ветер North", -20, 20, 0),
            ("wind_e", "Ветер East", -20, 20, 0),
            ("wind_d", "Ветер Down", -20, 20, 0),
        ]
        
        for key, label_text, min_val, max_val, default_val in wind_config:
            row = QHBoxLayout()
            label = QLabel(f"{label_text}:")
            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setValue(default_val)
            spinbox.setSingleStep(1)
            spinbox.valueChanged.connect(self.update_wind)
            
            row.addWidget(label)
            row.addWidget(spinbox)
            wind_layout.addLayout(row)
            self.wind_controls[key] = spinbox
        
        layout.addWidget(wind_group)
        
        # ---------- Внешние моменты ----------
        moment_group = QGroupBox("Внешние моменты (тест, Н·м)")
        moment_layout = QVBoxLayout(moment_group)
        
        moment_config = [
            ("Mx", "Mx (крен)", -50, 50, 0),
            ("My", "My (тангаж)", -50, 50, 0),
            ("Mz", "Mz (рыск)", -50, 50, 0),
        ]
        
        self.moments_controls = {}
        for key, label_text, min_val, max_val, default_val in moment_config:
            row = QHBoxLayout()
            label = QLabel(f"{label_text}:")
            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setValue(default_val)
            spinbox.setSingleStep(1)
            spinbox.valueChanged.connect(self.update_moments)
            
            row.addWidget(label)
            row.addWidget(spinbox)
            moment_layout.addLayout(row)
            self.moments_controls[key] = spinbox
        
        # кнопки для тестов моментов
        moment_test_layout = QHBoxLayout()
        test_mx_btn = QPushButton("Тест: Mx=10")
        test_mx_btn.clicked.connect(lambda: self.test_moment('Mx', 10))
        test_my_btn = QPushButton("Тест: My=10")
        test_my_btn.clicked.connect(lambda: self.test_moment('My', 10))
        test_mz_btn = QPushButton("Тест: Mz=10")
        test_mz_btn.clicked.connect(lambda: self.test_moment('Mz', 10))
        
        moment_test_layout.addWidget(test_mx_btn)
        moment_test_layout.addWidget(test_my_btn)
        moment_test_layout.addWidget(test_mz_btn)
        moment_layout.addLayout(moment_test_layout)
        
        layout.addWidget(moment_group)
    
    def update_controls(self):
        """Обновляем отклонения рулей"""
        throttle = self.control_controls['throttle'].value() / 100.0
        delta_a = np.radians(self.control_controls['delta_a'].value())
        delta_e = np.radians(self.control_controls['delta_e'].value())
        delta_r = np.radians(self.control_controls['delta_r'].value())
        
        self.dynamics.delta = np.array([delta_a, delta_e, delta_r, throttle])
    
    def update_wind(self):
        """Обновляем ветер"""
        wind_n = self.wind_controls['wind_n'].value()
        wind_e = self.wind_controls['wind_e'].value()
        wind_d = self.wind_controls['wind_d'].value()
        
        self.dynamics.wind_ned = np.array([wind_n, wind_e, wind_d])
    
    def update_moments(self):
        """Обновляем внешние моменты"""
        Mx = self.moments_controls['Mx'].value()
        My = self.moments_controls['My'].value()
        Mz = self.moments_controls['Mz'].value()
        
        self.dynamics.ext_moments = np.array([Mx, My, Mz])
    
    # тестовые функции
    def test_elevator(self):
        self.control_controls['delta_e'].setValue(10)
        self.test_timer.start(2000)
    
    def test_aileron(self):
        self.control_controls['delta_a'].setValue(10)
        self.test_timer.start(2000)
    
    def test_rudder(self):
        self.control_controls['delta_r'].setValue(10)
        self.test_timer.start(2000)
    
    def test_moment(self, axis, value):
        if axis in self.moments_controls:
            self.moments_controls[axis].setValue(value)
            self.test_timer.start(2000)
    
    def test_tecs_step_altitude(self):
        current_alt = self.tecs_altitude_spinbox.value()
        self.tecs_altitude_spinbox.setValue(current_alt - 50)
        self.test_timer.start(5000)
    
    def test_tecs_step_velocity(self):
        current_vel = self.tecs_velocity_spinbox.value()
        self.tecs_velocity_spinbox.setValue(current_vel + 5)
        self.test_timer.start(5000)
    
    def test_tecs_both(self):
        current_alt = self.tecs_altitude_spinbox.value()
        current_vel = self.tecs_velocity_spinbox.value()
        self.tecs_altitude_spinbox.setValue(current_alt - 30)
        self.tecs_velocity_spinbox.setValue(current_vel + 3)
        self.test_timer.start(7000)
    
    def _reset_test_values(self):
        """Возвращаем всё обратно"""
        self.test_timer.stop()
        
        if 'delta_e' in self.control_controls:
            self.control_controls['delta_e'].setValue(-3)
        if 'delta_a' in self.control_controls:
            self.control_controls['delta_a'].setValue(0)
        if 'delta_r' in self.control_controls:
            self.control_controls['delta_r'].setValue(0)
        
        for axis in ['Mx', 'My', 'Mz']:
            if axis in self.moments_controls:
                self.moments_controls[axis].setValue(0)
    
    def reset_all_to_defaults(self):
        """Сброс всех настроек"""
        if 'throttle' in self.control_controls:
            self.control_controls['throttle'].setValue(70)
        if 'delta_a' in self.control_controls:
            self.control_controls['delta_a'].setValue(0)
        if 'delta_e' in self.control_controls:
            self.control_controls['delta_e'].setValue(-3)
        if 'delta_r' in self.control_controls:
            self.control_controls['delta_r'].setValue(0)
        
        for axis in ['wind_n', 'wind_e', 'wind_d']:
            if axis in self.wind_controls:
                self.wind_controls[axis].setValue(0)
        
        for axis in ['Mx', 'My', 'Mz']:
            if axis in self.moments_controls:
                self.moments_controls[axis].setValue(0)
        
        if hasattr(self, 'tecs_altitude_spinbox'):
            self.tecs_altitude_spinbox.setValue(-100)
        if hasattr(self, 'tecs_velocity_spinbox'):
            self.tecs_velocity_spinbox.setValue(25)
        if hasattr(self, 'tecs_kp_E_spinbox'):
            self.tecs_kp_E_spinbox.setValue(0.015)
        if hasattr(self, 'tecs_ki_E_spinbox'):
            self.tecs_ki_E_spinbox.setValue(0.002)
        if hasattr(self, 'tecs_kp_B_spinbox'):
            self.tecs_kp_B_spinbox.setValue(0.05)
        if hasattr(self, 'tecs_ki_B_spinbox'):
            self.tecs_ki_B_spinbox.setValue(0.005)
        if hasattr(self, 'tecs_enable_checkbox'):
            self.tecs_enable_checkbox.setChecked(False)
        
        self.update_controls()
        self.update_wind()
        self.update_moments()