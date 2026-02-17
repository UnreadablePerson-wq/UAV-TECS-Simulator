# -*- coding: utf-8 -*-
"""
Функции для построения графиков
"""

import numpy as np
import matplotlib.pyplot as plt

def plot_results(start_time, dt, end_time, state):
    """Рисуем графики всех параметров"""
    if state.shape[1] == 0:
        return
        
    t = np.linspace(start_time, end_time, state.shape[1])
    
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))
    fig.suptitle('Результаты моделирования', fontsize=16)
    
    # координаты
    axes[0, 0].plot(t, state[0], label='North', linewidth=1.5)
    axes[0, 0].plot(t, state[1], label='East', linewidth=1.5)
    axes[0, 0].plot(t, -state[2], label='Altitude', linewidth=1.5)
    axes[0, 0].set_title('Позиция')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # углы эйлера
    axes[0, 1].plot(t, np.degrees(state[3]), label='Крен', linewidth=1.5)
    axes[0, 1].plot(t, np.degrees(state[4]), label='Тангаж', linewidth=1.5)
    axes[0, 1].plot(t, np.degrees(state[5]), label='Рыскание', linewidth=1.5)
    axes[0, 1].set_title('Углы Эйлера (град)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # линейные скорости
    axes[0, 2].plot(t, state[6], label='u', linewidth=1.5)
    axes[0, 2].plot(t, state[7], label='v', linewidth=1.5)
    axes[0, 2].plot(t, state[8], label='w', linewidth=1.5)
    axes[0, 2].set_title('Скорости тела (м/с)')
    axes[0, 2].legend()
    axes[0, 2].grid(True)
    
    # угловые скорости
    axes[1, 0].plot(t, np.degrees(state[9]), label='p', linewidth=1.5)
    axes[1, 0].plot(t, np.degrees(state[10]), label='q', linewidth=1.5)
    axes[1, 0].plot(t, np.degrees(state[11]), label='r', linewidth=1.5)
    axes[1, 0].set_title('Угловые скорости (град/с)')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # воздушная скорость
    Va = np.sqrt(state[6]**2 + state[7]**2 + state[8]**2)
    axes[1, 1].plot(t, Va, 'r-', label='Va', linewidth=2)
    axes[1, 1].set_title('Воздушная скорость (м/с)')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # траектория
    axes[1, 2].plot(state[1], state[0], 'b-', linewidth=1.5)
    axes[1, 2].plot(state[1][0], state[0][0], 'go', markersize=8, label='старт')
    axes[1, 2].plot(state[1][-1], state[0][-1], 'rs', markersize=8, label='финиш')
    axes[1, 2].set_title('Траектория (вид сверху)')
    axes[1, 2].set_xlabel('East (м)')
    axes[1, 2].set_ylabel('North (м)')
    axes[1, 2].legend()
    axes[1, 2].grid(True)
    axes[1, 2].axis('equal')
    
    # кватернионы
    axes[2, 0].plot(t, state[12], label='e0', linewidth=1.5)
    axes[2, 0].plot(t, state[13], label='ex', linewidth=1.5)
    axes[2, 0].plot(t, state[14], label='ey', linewidth=1.5)
    axes[2, 0].plot(t, state[15], label='ez', linewidth=1.5)
    axes[2, 0].set_title('Кватернионы')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # норма кватерниона
    e_norm = np.sqrt(state[12]**2 + state[13]**2 + state[14]**2 + state[15]**2)
    axes[2, 1].plot(t, e_norm, 'r-', linewidth=2)
    axes[2, 1].axhline(y=1.0, color='g', linestyle='--', alpha=0.5, label='норма=1')
    axes[2, 1].set_title('Норма кватерниона')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    axes[2, 1].set_ylim([0.9, 1.1])
    
    # высота
    axes[2, 2].plot(t, -state[2], 'b-', linewidth=1.5)
    axes[2, 2].set_title('Высота (м)')
    axes[2, 2].set_xlabel('Время (с)')
    axes[2, 2].grid(True)
    
    plt.tight_layout()
    plt.show()