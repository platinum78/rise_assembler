from PyQt5.QtWidgets import QDialog, QApplication, QMainWindow
from ui.kinematics_setup import *
from ui.current_task import *
from ui.mainwindow import *
import os, sys

class ControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.show()

app = QApplication(sys.argv)
w = ControlPanel()
w.show()
sys.exit(app.exec_())