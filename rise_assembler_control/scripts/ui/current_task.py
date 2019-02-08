# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\ui\current_task.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_dialog_current_task(object):
    def setupUi(self, dialog_current_task):
        dialog_current_task.setObjectName("dialog_current_task")
        dialog_current_task.resize(800, 430)
        dialog_current_task.setMinimumSize(QtCore.QSize(800, 430))
        dialog_current_task.setMaximumSize(QtCore.QSize(800, 430))
        dialog_current_task.setBaseSize(QtCore.QSize(800, 430))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        dialog_current_task.setFont(font)
        dialog_current_task.setSizeGripEnabled(False)
        self.pushButton = QtWidgets.QPushButton(dialog_current_task)
        self.pushButton.setGeometry(QtCore.QRect(280, 380, 241, 31))
        self.pushButton.setObjectName("pushButton")
        self.groupBox = QtWidgets.QGroupBox(dialog_current_task)
        self.groupBox.setGeometry(QtCore.QRect(20, 20, 331, 341))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        self.groupBox.setFont(font)
        self.groupBox.setObjectName("groupBox")
        self.milestone_01 = QtWidgets.QLabel(self.groupBox)
        self.milestone_01.setGeometry(QtCore.QRect(20, 30, 291, 41))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(9)
        self.milestone_01.setFont(font)
        self.milestone_01.setStyleSheet("border-color: black;\n"
"border-width: 1px;\n"
"border-style: solid;")
        self.milestone_01.setAlignment(QtCore.Qt.AlignCenter)
        self.milestone_01.setObjectName("milestone_01")
        self.milestone_06 = QtWidgets.QLabel(self.groupBox)
        self.milestone_06.setGeometry(QtCore.QRect(20, 280, 291, 41))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(9)
        self.milestone_06.setFont(font)
        self.milestone_06.setStyleSheet("border-color: black;\n"
"border-width: 1px;\n"
"border-style: solid;")
        self.milestone_06.setAlignment(QtCore.Qt.AlignCenter)
        self.milestone_06.setObjectName("milestone_06")
        self.milestone_02 = QtWidgets.QLabel(self.groupBox)
        self.milestone_02.setGeometry(QtCore.QRect(20, 80, 291, 41))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(9)
        self.milestone_02.setFont(font)
        self.milestone_02.setStyleSheet("border-color: black;\n"
"border-width: 1px;\n"
"border-style: solid;")
        self.milestone_02.setAlignment(QtCore.Qt.AlignCenter)
        self.milestone_02.setObjectName("milestone_02")
        self.milestone_03 = QtWidgets.QLabel(self.groupBox)
        self.milestone_03.setGeometry(QtCore.QRect(20, 130, 291, 41))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(9)
        self.milestone_03.setFont(font)
        self.milestone_03.setStyleSheet("border-color: black;\n"
"border-width: 1px;\n"
"border-style: solid;")
        self.milestone_03.setAlignment(QtCore.Qt.AlignCenter)
        self.milestone_03.setObjectName("milestone_03")
        self.milestone_04 = QtWidgets.QLabel(self.groupBox)
        self.milestone_04.setGeometry(QtCore.QRect(20, 180, 291, 41))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(9)
        self.milestone_04.setFont(font)
        self.milestone_04.setStyleSheet("border-color: black;\n"
"border-width: 1px;\n"
"border-style: solid;")
        self.milestone_04.setAlignment(QtCore.Qt.AlignCenter)
        self.milestone_04.setObjectName("milestone_04")
        self.milestone_05 = QtWidgets.QLabel(self.groupBox)
        self.milestone_05.setGeometry(QtCore.QRect(20, 230, 291, 41))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(9)
        self.milestone_05.setFont(font)
        self.milestone_05.setStyleSheet("border-color: black;\n"
"border-width: 1px;\n"
"border-style: solid;")
        self.milestone_05.setAlignment(QtCore.Qt.AlignCenter)
        self.milestone_05.setObjectName("milestone_05")
        self.groupBox_2 = QtWidgets.QGroupBox(dialog_current_task)
        self.groupBox_2.setGeometry(QtCore.QRect(370, 20, 411, 341))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        self.groupBox_2.setFont(font)
        self.groupBox_2.setObjectName("groupBox_2")
        self.progressBar_overall = QtWidgets.QProgressBar(self.groupBox_2)
        self.progressBar_overall.setGeometry(QtCore.QRect(20, 90, 371, 21))
        self.progressBar_overall.setProperty("value", 24)
        self.progressBar_overall.setObjectName("progressBar_overall")
        self.label = QtWidgets.QLabel(self.groupBox_2)
        self.label.setGeometry(QtCore.QRect(20, 60, 371, 21))
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.label_8 = QtWidgets.QLabel(self.groupBox_2)
        self.label_8.setGeometry(QtCore.QRect(20, 180, 371, 21))
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.progressBar_current = QtWidgets.QProgressBar(self.groupBox_2)
        self.progressBar_current.setGeometry(QtCore.QRect(20, 230, 371, 21))
        self.progressBar_current.setProperty("value", 24)
        self.progressBar_current.setObjectName("progressBar_current")
        self.label_9 = QtWidgets.QLabel(self.groupBox_2)
        self.label_9.setGeometry(QtCore.QRect(20, 200, 371, 21))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.label_9.setFont(font)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")

        self.retranslateUi(dialog_current_task)
        QtCore.QMetaObject.connectSlotsByName(dialog_current_task)

    def retranslateUi(self, dialog_current_task):
        _translate = QtCore.QCoreApplication.translate
        dialog_current_task.setWindowTitle(_translate("dialog_current_task", "Dialog"))
        self.pushButton.setText(_translate("dialog_current_task", "Cancel Current Task"))
        self.groupBox.setTitle(_translate("dialog_current_task", "Milestones"))
        self.milestone_01.setText(_translate("dialog_current_task", "Reading Assembly Manual"))
        self.milestone_06.setText(_translate("dialog_current_task", "Assembly Complete!"))
        self.milestone_02.setText(_translate("dialog_current_task", "Extracting Image Features"))
        self.milestone_03.setText(_translate("dialog_current_task", "Parsing Assembly Procedure"))
        self.milestone_04.setText(_translate("dialog_current_task", "Assembly in Progress"))
        self.milestone_05.setText(_translate("dialog_current_task", "Verifying Integrity of Assembly"))
        self.groupBox_2.setTitle(_translate("dialog_current_task", "Progress"))
        self.label.setText(_translate("dialog_current_task", "Overall Progress"))
        self.label_8.setText(_translate("dialog_current_task", "Current Task"))
        self.label_9.setText(_translate("dialog_current_task", "Initializing..."))

