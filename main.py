#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Симулятор БПЛА с управлением полной энергией (TECS)
Практическая работа №4
Запуск программы
"""

import sys
import os
from pathlib import Path

# Добавляем корневую папку проекта в путь поиска модулей
# Это решит проблемы с импортами
root_dir = Path(__file__).parent.absolute()
if str(root_dir) not in sys.path:
    sys.path.insert(0, str(root_dir))

from PyQt5.QtWidgets import QApplication
from gui.main_window import SimulationWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SimulationWindow()
    window.show()
    sys.exit(app.exec_())