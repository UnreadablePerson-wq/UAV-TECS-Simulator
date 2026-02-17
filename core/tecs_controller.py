# -*- coding: utf-8 -*-
"""
TECS (Total Energy Control System) - управление полной энергией
Идея: дроссель управляет полной энергией, тангаж - распределением
между потенциальной и кинетической
"""

import numpy as np

class TECSController:
    """Реализация управления полной энергией"""
    
    def __init__(self, mass=11.0, gravity=9.81):
        self.mass = mass
        self.g = gravity
        
        # коэффициенты подбирал методом научного тыка
        self.kp_E = 0.015      # пропорциональный по полной энергии
        self.ki_E = 0.002      # интегральный
        self.kd_E = 0.001      # дифференциальный
        
        self.kp_B = 0.05       # для баланса энергий
        self.ki_B = 0.005
        self.kd_B = 0.003
        
        # накопленные ошибки
        self.integral_E = 0.0
        self.integral_B = 0.0
        self.prev_E_error = 0.0
        self.prev_B_error = 0.0
        
        # ограничения
        self.max_throttle = 1.0
        self.min_throttle = 0.0
        self.max_pitch_cmd = np.radians(30.0)
        self.min_pitch_cmd = np.radians(-30.0)
        
        # защита от больших ошибок
        self.max_height_error = 50.0
        
        # балансировка
        self.trim_throttle = 0.7
        self.trim_pitch = np.radians(4.5)
        
        # фильтр для производных
        self.derivative_filter_alpha = 0.1
        
        self.enabled = False
        
        # для отладки
        self.energy_history = []
        self.balance_history = []
        self.throttle_history = []
        self.pitch_history = []
    
    def update(self, Va, altitude, Va_cmd, alt_cmd, dt):
        """
        Рассчитываем команды на основе текущего состояния и заданий
        altitude - в NED координатах (вниз положительно)
        """
        if not self.enabled:
            return self.trim_throttle, self.trim_pitch
        
        # переводим высоту в обычную систему (вверх положительно)
        h = -altitude
        h_cmd = -alt_cmd if alt_cmd < 0 else alt_cmd
        
        # насыщаем ошибку по высоте
        h_error = h_cmd - h
        h_error_sat = np.clip(h_error, -self.max_height_error, self.max_height_error)
        
        # полная энергия: потенциальная + кинетическая
        E_current = self.mass * self.g * h + 0.5 * self.mass * Va**2
        E_desired = self.mass * self.g * h_cmd + 0.5 * self.mass * Va_cmd**2
        E_error = E_desired - E_current
        
        # баланс энергий (разность) - для распределения
        B_current = self.mass * self.g * h - 0.5 * self.mass * Va**2
        B_desired = self.mass * self.g * h_cmd - 0.5 * self.mass * Va_cmd**2
        B_error = B_desired - B_current
        
        # накапливаем ошибки
        self.integral_E += E_error * dt
        self.integral_B += B_error * dt
        
        # антивиндовка
        self.integral_E = np.clip(self.integral_E, -50.0, 50.0)
        self.integral_B = np.clip(self.integral_B, -20.0, 20.0)
        
        # производные ошибок
        dE_error = (E_error - self.prev_E_error) / dt if dt > 0 else 0
        dB_error = (B_error - self.prev_B_error) / dt if dt > 0 else 0
        
        # фильтрация
        dE_error_filt = self.derivative_filter_alpha * dE_error + (1 - self.derivative_filter_alpha) * 0
        dB_error_filt = self.derivative_filter_alpha * dB_error + (1 - self.derivative_filter_alpha) * 0
        
        self.prev_E_error = E_error
        self.prev_B_error = B_error
        
        # ПИД для дросселя
        throttle_cmd = (self.trim_throttle + 
                       self.kp_E * E_error + 
                       self.ki_E * self.integral_E + 
                       self.kd_E * dE_error_filt)
        
        # ПИД для тангажа
        pitch_cmd = (self.trim_pitch + 
                    self.kp_B * B_error + 
                    self.ki_B * self.integral_B + 
                    self.kd_B * dB_error_filt)
        
        # ограничиваем команды
        throttle_cmd = np.clip(throttle_cmd, self.min_throttle, self.max_throttle)
        pitch_cmd = np.clip(pitch_cmd, self.min_pitch_cmd, self.max_pitch_cmd)
        
        # логируем
        self.energy_history.append(E_error)
        self.balance_history.append(B_error)
        self.throttle_history.append(throttle_cmd)
        self.pitch_history.append(pitch_cmd)
        
        # не храним бесконечно
        if len(self.energy_history) > 1000:
            self.energy_history = self.energy_history[-1000:]
            self.balance_history = self.balance_history[-1000:]
            self.throttle_history = self.throttle_history[-1000:]
            self.pitch_history = self.pitch_history[-1000:]
        
        return throttle_cmd, pitch_cmd
    
    def reset(self):
        """Сброс накопленных ошибок"""
        self.integral_E = 0.0
        self.integral_B = 0.0
        self.prev_E_error = 0.0
        self.prev_B_error = 0.0
        self.energy_history = []
        self.balance_history = []
        self.throttle_history = []
        self.pitch_history = []
    
    def enable(self):
        self.enabled = True
        self.reset()
    
    def disable(self):
        self.enabled = False
    
    def set_gains(self, kp_E=None, ki_E=None, kd_E=None, 
                  kp_B=None, ki_B=None, kd_B=None):
        """Обновление коэффициентов регуляторов"""
        if kp_E is not None: self.kp_E = kp_E
        if ki_E is not None: self.ki_E = ki_E
        if kd_E is not None: self.kd_E = kd_E
        if kp_B is not None: self.kp_B = kp_B
        if ki_B is not None: self.ki_B = ki_B
        if kd_B is not None: self.kd_B = kd_B