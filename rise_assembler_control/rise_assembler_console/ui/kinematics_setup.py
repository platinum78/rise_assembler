# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\kinematics_setup.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_kinematics_setup_dialog(object):
    def setupUi(self, kinematics_setup_dialog):
        kinematics_setup_dialog.setObjectName("kinematics_setup_dialog")
        kinematics_setup_dialog.resize(510, 477)
        self.buttonBox = QtWidgets.QDialogButtonBox(kinematics_setup_dialog)
        self.buttonBox.setGeometry(QtCore.QRect(300, 430, 191, 31))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.frame = QtWidgets.QFrame(kinematics_setup_dialog)
        self.frame.setGeometry(QtCore.QRect(20, 20, 231, 191))
        self.frame.setStyleSheet("background-color: #eeeeee;\n"
"border-color: gray;\n"
"border-style: solid;\n"
"border-width: 1px;")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(20, 10, 191, 31))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(11)
        self.label.setFont(font)
        self.label.setStyleSheet("background-color: none;\n"
"border-style: none;")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.horizontalSlider = QtWidgets.QSlider(self.frame)
        self.horizontalSlider.setGeometry(QtCore.QRect(20, 150, 191, 21))
        self.horizontalSlider.setStyleSheet("background-color: none;\n"
"border-style: none;")
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.frame_2 = QtWidgets.QFrame(kinematics_setup_dialog)
        self.frame_2.setGeometry(QtCore.QRect(260, 20, 231, 191))
        self.frame_2.setStyleSheet("background-color: #eeeeee;\n"
"border-color: gray;\n"
"border-style: solid;\n"
"border-width: 1px;")
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.label_2 = QtWidgets.QLabel(self.frame_2)
        self.label_2.setGeometry(QtCore.QRect(20, 10, 191, 31))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(11)
        self.label_2.setFont(font)
        self.label_2.setStyleSheet("background-color: none;\n"
"border-style: none;")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalSlider_2 = QtWidgets.QSlider(self.frame_2)
        self.horizontalSlider_2.setGeometry(QtCore.QRect(20, 150, 191, 21))
        self.horizontalSlider_2.setStyleSheet("background-color: none;\n"
"border-style: none;")
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.frame_3 = QtWidgets.QFrame(kinematics_setup_dialog)
        self.frame_3.setGeometry(QtCore.QRect(20, 220, 231, 191))
        self.frame_3.setStyleSheet("background-color: #eeeeee;\n"
"border-color: gray;\n"
"border-style: solid;\n"
"border-width: 1px;")
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.label_3 = QtWidgets.QLabel(self.frame_3)
        self.label_3.setGeometry(QtCore.QRect(10, 10, 211, 31))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(11)
        self.label_3.setFont(font)
        self.label_3.setStyleSheet("background-color: none;\n"
"border-style: none;")
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.horizontalSlider_3 = QtWidgets.QSlider(self.frame_3)
        self.horizontalSlider_3.setGeometry(QtCore.QRect(20, 150, 191, 21))
        self.horizontalSlider_3.setStyleSheet("background-color: none;\n"
"border-style: none;")
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.frame_4 = QtWidgets.QFrame(kinematics_setup_dialog)
        self.frame_4.setGeometry(QtCore.QRect(260, 220, 231, 191))
        self.frame_4.setStyleSheet("background-color: #eeeeee;\n"
"border-color: gray;\n"
"border-style: solid;\n"
"border-width: 1px;")
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.label_4 = QtWidgets.QLabel(self.frame_4)
        self.label_4.setGeometry(QtCore.QRect(30, 10, 181, 31))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(11)
        self.label_4.setFont(font)
        self.label_4.setStyleSheet("background-color: none;\n"
"border-style: none;")
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.horizontalSlider_4 = QtWidgets.QSlider(self.frame_4)
        self.horizontalSlider_4.setGeometry(QtCore.QRect(20, 150, 191, 21))
        self.horizontalSlider_4.setStyleSheet("background-color: none;\n"
"border-style: none;")
        self.horizontalSlider_4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_4.setObjectName("horizontalSlider_4")

        self.retranslateUi(kinematics_setup_dialog)
        self.buttonBox.accepted.connect(kinematics_setup_dialog.accept)
        self.buttonBox.rejected.connect(kinematics_setup_dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(kinematics_setup_dialog)

    def retranslateUi(self, kinematics_setup_dialog):
        _translate = QtCore.QCoreApplication.translate
        kinematics_setup_dialog.setWindowTitle(_translate("kinematics_setup_dialog", "Kinematics Parameters Setup"))
        self.label.setText(_translate("kinematics_setup_dialog", "Velocity Multiples"))
        self.label_2.setText(_translate("kinematics_setup_dialog", "Path Resolution"))
        self.label_3.setText(_translate("kinematics_setup_dialog", "Path Overlap Constant"))
        self.label_4.setText(_translate("kinematics_setup_dialog", "Undefined"))

