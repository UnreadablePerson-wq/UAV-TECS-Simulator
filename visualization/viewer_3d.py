# -*- coding: utf-8 -*-
"""
3D визуализация самолета
Рисуем примитивную модельку и ветер
"""

import numpy as np
import pyqtgraph.opengl as gl
from core.quaternion_utils import quaternion_to_rotation_matrix

class VehicleVisualizer:
    """Отрисовка самолетика в 3D"""
    
    def __init__(self, state, window, scale=10):
        self.unit_length = scale
        # матрица для перевода из NED в OpenGL координаты
        self.R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        
        # создаем mesh самолета
        self.points, self.indices, self.mesh_colors = self._create_vehicle_mesh()
        self.mesh_item = self._create_mesh_item(state)
        window.addItem(self.mesh_item)
        
        # линия для отображения ветра
        self.wind_line = gl.GLLinePlotItem()
        try:
            self.wind_line.setData(pos=np.array([[0,0,0],[0,0,0]]), color=(0,1,1,1), width=3)
            self.wind_line.setGLOptions('translucent')
        except:
            pass
        window.addItem(self.wind_line)
    
    def _create_vehicle_mesh(self):
        """Точки для модели самолета"""
        points = self.unit_length * np.array([
            [1.5, 0, 0], [0.75, 0.3, -0.3], [0.75, -0.3, -0.3],
            [0.75, -0.3, 0.3], [0.75, 0.3, 0.3], [-3, 0, 0],
            [0, 2, 0], [-1, 2, 0], [-1, -2, 0], [0, -2, 0],
            [-2.5, 1.2, 0], [-3, 1.2, 0], [-3, -1.2, 0], [-2.5, -1.2, 0],
            [-2.5, 0, 0], [-3, 0, -1]
        ]).T
        
        # треугольники
        indices = np.array([
            [0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1],
            [5, 1, 2], [5, 2, 3], [5, 3, 4], [5, 4, 1],
            [6, 7, 9], [7, 9, 8],
            [10, 11, 13], [11, 13, 12],
            [5, 14, 15]
        ])
        
        # цвет оранжевый
        colors = np.array([[0.9, 0.5, 0, 1]] * len(indices))
        
        return points, indices, colors
    
    def _create_mesh_item(self, state):
        """Создаем mesh объект в начальном положении"""
        position = np.array([[state.north], [state.east], [state.altitude]])
        R_bi = quaternion_to_rotation_matrix(state.e)
        
        rotated_points = np.dot(R_bi, self.points)
        translated_points = rotated_points + position
        translated_points = np.dot(self.R_ned, translated_points)
        
        mesh = self._points_to_mesh(translated_points, self.indices)
        
        return gl.GLMeshItem(vertexes=mesh, vertexColors=self.mesh_colors,
                             drawEdges=True, smooth=False, computeNormals=False)
    
    def _points_to_mesh(self, points, indices):
        """Преобразуем точки в треугольную сетку"""
        points = points.T
        mesh = np.array([[points[indices[0,0]], points[indices[0,1]], points[indices[0,2]]]])
        
        for i in range(1, indices.shape[0]):
            triangle = np.array([[points[indices[i,0]], points[indices[i,1]], points[indices[i,2]]]])
            mesh = np.concatenate((mesh, triangle), axis=0)
        
        return mesh
    
    def update(self, state, wind_inertial=None):
        """Обновляем положение самолета и ветер"""
        position = np.array([[state.north], [state.east], [state.altitude]])
        R_bi = quaternion_to_rotation_matrix(state.e)
        
        rotated_points = np.dot(R_bi, self.points)
        translated_points = rotated_points + position
        translated_points = np.dot(self.R_ned, translated_points)
        
        mesh = self._points_to_mesh(translated_points, self.indices)
        self.mesh_item.setMeshData(vertexes=mesh, vertexColors=self.mesh_colors)
        
        # показываем ветер стрелочкой
        if wind_inertial is not None:
            scale_vis = 2.0
            start = np.array([state.north, state.east, state.altitude])
            end = start + wind_inertial * scale_vis
            
            p0 = np.array([start[0], start[1], -start[2]])
            p1 = np.array([end[0], end[1], -end[2]])
            pts = np.vstack((p0, p1))
            
            try:
                self.wind_line.setData(pos=pts, color=(0,1,1,1), width=3)
            except:
                pass