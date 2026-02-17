# -*- coding: utf-8 -*-
"""
Динамика самолета Aerosonde
Все коэффициенты из методички, модель срыва тоже оттуда
"""

import numpy as np
from core.quaternion_utils import quaternion_to_rotation_matrix

class VehicleDynamics:
    """Модель динамики полета"""
    
    def __init__(self):
        # масс-инерционные (из документации на Aerosonde)
        self.mass = 11.0
        self.Jx = 0.824
        self.Jy = 1.135
        self.Jz = 1.759
        self.Jxz = 0.12
        
        # геометрия
        self.S_wing = 0.55
        self.b = 2.90
        self.c = 0.19
        self.rho = 1.268  # плотность воздуха
        
        # двигатель (эмпирические коэффициенты)
        self.V_max = 44.4
        self.D_prop = 0.508
        self.K_Q = 0.0659
        self.C_T2 = -0.1079
        self.C_T1 = -0.06044
        self.C_T0 = 0.09357
        
        # аэродинамические коэффициенты (из приложения Г методички)
        # продольные
        self.C_L_0 = 0.23
        self.C_L_alpha = 5.61
        self.C_L_q = 7.95
        self.C_L_de = 0.13
        
        self.C_D_0 = 0.043
        self.C_D_alpha = 0.030
        self.C_D_q = 0.0
        self.C_D_de = 0.0135
        
        self.C_m_0 = 0.0135
        self.C_m_alpha = -2.74
        self.C_m_q = -38.21
        self.C_m_de = -0.99
        
        # боковые
        self.C_Y_0 = 0.0
        self.C_Y_beta = -0.83
        self.C_Y_p = 0.0
        self.C_Y_r = 0.0
        self.C_Y_delta_a = 0.075
        self.C_Y_delta_r = 0.19
        
        self.C_ell_0 = 0.0
        self.C_ell_beta = -0.13
        self.C_ell_p = -0.51
        self.C_ell_r = 0.25
        self.C_ell_delta_a = 0.17
        self.C_ell_delta_r = 0.0024
        
        self.C_n_0 = 0.0
        self.C_n_beta = 0.073
        self.C_n_p = -0.069
        self.C_n_r = -0.095
        self.C_n_delta_a = -0.011
        self.C_n_delta_r = -0.069
        
        # модель срыва
        self.M_trans = 50.0
        self.alpha0 = 0.47
        
        # начальные отклонения рулей (балансировка)
        self.delta = np.array([0.0, np.radians(-3.0), 0.0, 0.7])
        
        # ветер
        self.wind_ned = np.array([0.0, 0.0, 0.0])
        self.gust_body = np.array([0.0, 0.0, 0.0])
        self.gust_tau = 1.0
        self.gust_sigma = np.array([1.0, 0.5, 0.3])
        
        # для тестов - только моменты, силы не добавляем
        self.ext_moments = np.array([0.0, 0.0, 0.0])
        
        self._update_gamma_parameters()
    
    def _update_gamma_parameters(self):
        """Вспомогательные параметры для уравнений моментов"""
        self.Gamma = self.Jx * self.Jz - self.Jxz ** 2
        self.Gamma1 = self.Jxz * (self.Jx - self.Jy + self.Jz) / self.Gamma
        self.Gamma2 = (self.Jz * (self.Jz - self.Jy) + self.Jxz ** 2) / self.Gamma
        self.Gamma3 = self.Jz / self.Gamma
        self.Gamma4 = self.Jxz / self.Gamma
        self.Gamma5 = (self.Jz - self.Jx) / self.Jy
        self.Gamma6 = self.Jxz / self.Jy
        self.Gamma7 = ((self.Jx - self.Jy) * self.Jx + self.Jxz ** 2) / self.Gamma
        self.Gamma8 = self.Jx / self.Gamma
    
    def update_wind_gusts(self, dt):
        """Моделируем порывы ветра как случайный процесс первого порядка"""
        if dt <= 0:
            return
        
        a = np.exp(-dt / self.gust_tau)
        self.gust_body = (a * self.gust_body + 
                         np.random.randn(3) * self.gust_sigma * np.sqrt(dt))
    
    def calculate_aerodynamic_forces(self, state_vec):
        """Считаем аэродинамические силы и моменты по текущему состоянию"""
        pn, pe, pd, u, v, w, p, q, r, e0, ex, ey, ez = state_vec
        
        # нормализуем кватернион (на всякий случай)
        e_norm = np.sqrt(e0**2 + ex**2 + ey**2 + ez**2)
        if e_norm > 0:
            e0, ex, ey, ez = e0/e_norm, ex/e_norm, ey/e_norm, ez/e_norm
        
        # матрица перехода из тела в инерциальную
        R = quaternion_to_rotation_matrix(np.array([e0, ex, ey, ez]))
        
        # учитываем ветер
        wind_body = R.T @ self.wind_ned + self.gust_body
        
        # относительная скорость (с учетом ветра)
        u_r = u - wind_body[0]
        v_r = v - wind_body[1]
        w_r = w - wind_body[2]
        
        # воздушная скорость и углы атаки/скольжения
        Va = np.sqrt(u_r**2 + v_r**2 + w_r**2)
        if Va > 0.1:
            alpha = np.arctan2(w_r, u_r)
            beta = np.arcsin(np.clip(v_r / Va, -1.0, 1.0))
        else:
            alpha = 0.0
            beta = 0.0
            Va = 0.1
        
        q_bar = 0.5 * self.rho * Va**2  # скоростной напор
        
        # тяга двигателя
        throttle = self.delta[3]
        Omega = (self.V_max * throttle / self.K_Q) if self.K_Q > 0 else 0
        thrust = 0
        if Omega > 10:
            n_rps = Omega / (2 * np.pi)
            J_op = Va / (n_rps * self.D_prop + 0.001)
            C_T = self.C_T2 * J_op**2 + self.C_T1 * J_op + self.C_T0
            thrust = self.rho * n_rps**2 * self.D_prop**4 * C_T
            if thrust < 0:
                thrust = 0
        
        # отклонения рулей
        delta_a = self.delta[0]
        delta_e = self.delta[1]
        delta_r = self.delta[2]
        
        # модель срыва (чтоб при больших углах атаки не было фигни)
        sigma_a = (1 + np.exp(-self.M_trans * (alpha - self.alpha0)) + 
                  np.exp(self.M_trans * (alpha + self.alpha0))) / \
                 ((1 + np.exp(-self.M_trans * (alpha - self.alpha0))) * 
                  (1 + np.exp(self.M_trans * (alpha + self.alpha0))))
        
        # коэффициент подъемной силы
        CL_lin = self.C_L_0 + self.C_L_alpha * alpha
        CL_stall = 2 * np.sign(alpha) * np.sin(alpha)**2 * np.cos(alpha)
        CL = (1 - sigma_a) * CL_lin + sigma_a * CL_stall
        CL += self.C_L_q * (self.c / (2 * Va)) * q + self.C_L_de * delta_e
        
        # коэффициент сопротивления
        CD = (self.C_D_0 + self.C_D_alpha * abs(alpha) + 
              self.C_D_q * (self.c / (2 * Va)) * q + 
              self.C_D_de * abs(delta_e))
        
        # аэродинамические силы в связанной СК
        FL = q_bar * self.S_wing * CL
        FD = q_bar * self.S_wing * CD
        
        # проецируем на оси
        fx_aero = -FD * np.cos(alpha) + FL * np.sin(alpha) + thrust
        fz_aero = -FD * np.sin(alpha) - FL * np.cos(alpha)
        
        # боковая сила
        CY = (self.C_Y_0 + self.C_Y_beta * beta + 
              self.C_Y_p * (self.b / (2 * Va)) * p + 
              self.C_Y_r * (self.b / (2 * Va)) * r + 
              self.C_Y_delta_a * delta_a + 
              self.C_Y_delta_r * delta_r)
        fy_aero = q_bar * self.S_wing * CY
        
        # моменты
        Cm = (self.C_m_0 + self.C_m_alpha * alpha + 
              self.C_m_q * (self.c / (2 * Va)) * q + 
              self.C_m_de * delta_e)
        
        Cell = (self.C_ell_0 + self.C_ell_beta * beta + 
                self.C_ell_p * (self.b / (2 * Va)) * p + 
                self.C_ell_r * (self.b / (2 * Va)) * r + 
                self.C_ell_delta_a * delta_a + 
                self.C_ell_delta_r * delta_r)
        
        Cn = (self.C_n_0 + self.C_n_beta * beta + 
              self.C_n_p * (self.b / (2 * Va)) * p + 
              self.C_n_r * (self.b / (2 * Va)) * r + 
              self.C_n_delta_a * delta_a + 
              self.C_n_delta_r * delta_r)
        
        M_roll = q_bar * self.S_wing * self.b * Cell
        M_pitch = q_bar * self.S_wing * self.c * Cm
        M_yaw = q_bar * self.S_wing * self.b * Cn
        
        # сила тяжести в связанной СК
        gravity_body = R.T @ np.array([0.0, 0.0, self.mass * 9.81])
        
        # итоговые силы и моменты
        forces = np.array([
            fx_aero + gravity_body[0],
            fy_aero + gravity_body[1],
            fz_aero + gravity_body[2]
        ])
        
        moments = np.array([
            M_roll + self.ext_moments[0],
            M_pitch + self.ext_moments[1],
            M_yaw + self.ext_moments[2]
        ])
        
        # суммарный ветер для визуализации
        wind_total_ned = self.wind_ned + R @ self.gust_body
        
        return forces, moments, alpha, beta, Va, wind_total_ned
    
    def derivatives_dt(self, t, state):
        """Правые части уравнений движения"""
        pn, pe, pd, u, v, w, p, q, r, e0, ex, ey, ez = state
        
        # нормализация кватерниона
        e_norm = np.sqrt(e0**2 + ex**2 + ey**2 + ez**2)
        if e_norm != 0:
            e0, ex, ey, ez = e0/e_norm, ex/e_norm, ey/e_norm, ez/e_norm
        
        # считаем аэродинамику
        forces, moments, _, _, _, _ = self.calculate_aerodynamic_forces(state)
        
        # производные положения
        rot_mat = quaternion_to_rotation_matrix(np.array([e0, ex, ey, ez]))
        dpos_dt = rot_mat @ np.array([u, v, w])
        
        # производные линейных скоростей
        du_dt = r*v - q*w + forces[0]/self.mass
        dv_dt = p*w - r*u + forces[1]/self.mass
        dw_dt = q*u - p*v + forces[2]/self.mass
        
        # производные кватерниона
        de_dt = 0.5 * np.array([
            [0, -p, -q, -r],
            [p, 0, r, -q],
            [q, -r, 0, p],
            [r, q, -p, 0]
        ]) @ np.array([e0, ex, ey, ez])
        
        # производные угловых скоростей
        dp_dt = (self.Gamma1 * p * q - self.Gamma2 * q * r +
                 self.Gamma3 * moments[0] + self.Gamma4 * moments[2])
        dq_dt = (self.Gamma5 * p * r - self.Gamma6 * (p**2 - r**2) +
                 moments[1] / self.Jy)
        dr_dt = (self.Gamma7 * p * q - self.Gamma1 * q * r +
                 self.Gamma4 * moments[0] + self.Gamma8 * moments[2])
        
        return np.array([
            dpos_dt[0], dpos_dt[1], dpos_dt[2],
            du_dt, dv_dt, dw_dt,
            dp_dt, dq_dt, dr_dt,
            de_dt[0], de_dt[1], de_dt[2], de_dt[3]
        ])