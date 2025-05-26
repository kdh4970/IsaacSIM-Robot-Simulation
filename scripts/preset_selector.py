from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QProcess
from PyQt5.QtGui import QFont
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
