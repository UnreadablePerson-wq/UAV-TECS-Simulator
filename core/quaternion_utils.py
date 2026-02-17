# -*- coding: utf-8 -*-
"""
Утилиты для работы с кватернионами
Честно переписано из методички, там формулы стандартные
"""

import numpy as np

def quaternion_to_euler(e):
    """Конвертируем кватернион в углы Эйлера"""
    e0, ex, ey, ez = e[0], e[1], e[2], e[3]

    phi = np.arctan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex**2 + ey**2))
    theta_part = 2 * (e0 * ey - ex * ez)
    if abs(theta_part) >= 1:
        theta = np.copysign(np.pi / 2, theta_part)
    else:
        theta = np.arcsin(theta_part)
    psi = np.arctan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey**2 + ez**2))

    return phi, theta, psi

def euler_to_quaternion(phi, theta, psi):
    """Обратное преобразование - из углов в кватернион"""
    c_psi2 = np.cos(psi * 0.5)
    s_psi2 = np.sin(psi * 0.5)
    c_theta2 = np.cos(theta * 0.5)
    s_theta2 = np.sin(theta * 0.5)
    c_phi2 = np.cos(phi * 0.5)
    s_phi2 = np.sin(phi * 0.5)
    
    e0 = c_psi2 * c_theta2 * c_phi2 + s_psi2 * s_theta2 * s_phi2
    ex = c_psi2 * c_theta2 * s_phi2 - s_psi2 * s_theta2 * c_phi2
    ey = c_psi2 * s_theta2 * c_phi2 + s_psi2 * c_theta2 * s_phi2
    ez = s_psi2 * c_theta2 * c_phi2 - c_psi2 * s_theta2 * s_phi2

    return np.array([e0, ex, ey, ez])

def quaternion_to_rotation_matrix(e):
    """Получаем матрицу поворота из кватерниона"""
    e0, ex, ey, ez = e[0], e[1], e[2], e[3]

    r11 = e0**2 + ex**2 - ey**2 - ez**2
    r12 = 2 * (ex * ey - e0 * ez)
    r13 = 2 * (ex * ez + e0 * ey)
    r21 = 2 * (ex * ey + e0 * ez)
    r22 = e0**2 - ex**2 + ey**2 - ez**2
    r23 = 2 * (ey * ez - e0 * ex)
    r31 = 2 * (ex * ez - e0 * ey)
    r32 = 2 * (ey * ez + e0 * ex)
    r33 = e0**2 - ex**2 - ey**2 + ez**2

    return np.array([
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ])