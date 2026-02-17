# -*- coding: utf-8 -*-
"""
Класс состояния самолета
Тут просто храним все переменные, никакой магии
"""

import numpy as np
from core.quaternion_utils import euler_to_quaternion

class VehicleState:
    """Состояние БПЛА в текущий момент времени"""
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        """Сброс к начальным условиям (балансировка для горизонтального полета)"""
        self.north = 0.0
        self.east = 0.0
        self.altitude = -100.0  # NED координаты: вниз положительно, значит высота 100м
        
        # балансировочный угол для скорости 25 м/с
        self.phi = 0.0
        self.theta = np.radians(4.5)
        self.psi = 0.0
        
        # начальная скорость с учетом угла атаки
        Va = 25.0
        alpha = np.radians(4.5)
        self.u = Va * np.cos(alpha)
        self.v = 0.0
        self.w = Va * np.sin(alpha)
        
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        
        self.e = euler_to_quaternion(self.phi, self.theta, self.psi)
        
        # для удобства (чтоб каждый раз не считать)
        self.alpha = alpha
        self.beta = 0.0
        self.Va = Va