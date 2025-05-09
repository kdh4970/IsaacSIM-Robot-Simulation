from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QProcess
from PyQt5.QtGui import QFont
import sys
from time import perf_counter

class IntervalChecker:
    def __init__(self, text= None):
        self.text = text
        self.elapsed = None

    def __enter__(self):
        self.start = perf_counter()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        end = perf_counter()
        self.elapsed = end - self.start
        if self.text:print(f"During {self.text} : {1000 * self.elapsed:.3f}ms")

class PresetSelector(QMainWindow):
    def __init__(self,lst):
        super().__init__()
        self.preset_list = lst
        self.env = None
        self.robot = None
        self.select()

    def select(self):
        inputDialog = QInputDialog(self)
        inputDialog.setWindowTitle("Environment Choice")
        inputDialog.setLabelText("Which preset would you like to use?")
        inputDialog.setComboBoxItems(self.preset_list)
        inputDialog.setGeometry(1000, 600, 500, 400)

        if inputDialog.exec() == QInputDialog.Accepted:
            item = inputDialog.textValue()
            print(f'Selected preset: {item}')
            self.env, self.robot = item.split(" + ")
        else:
            print('Canceled')