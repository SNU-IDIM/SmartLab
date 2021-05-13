#! /usr/bin/env python
# -*- coding: utf-8 -*-
import ast
import json
import os
import sys
import time
from threading import Thread
import numpy as np
from PyQt5 import QtCore, uic
from PyQt5.QtCore import Qt, QThread, QUrl, pyqtSignal, pyqtSlot
from PyQt5.QtGui import *
# from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import *

from SmartLab_Client import SmartLabClient

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from Cam_Streaming_Client import Cam_Streaming_Client
from SqlHelper import SqlHelper
import pafy


global streaming
streaming = Cam_Streaming_Client(cam_list=['overview', 'cobot'])
img_con_flag = False

image_cobot    = np.empty((480, 640, 3))
image_overview = np.empty((480, 640, 3))

global streaming_mode
streaming_mode = 0


class Roboteefview(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        global img_con_flag
        global streaming
        self.streaming = streaming
        while True:
            if self.streaming.image_cobot[0][0][0] != 0.0:
                img_con_flag = True
            if img_con_flag is True:
                self.rgbImage = self.streaming.image_cobot
                h, w, ch = self.rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(self.rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                
                p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)
            else:
                print("waiting for flag  " +str(img_con_flag))

class QCameraStreaming(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        global img_con_flag
        global streaming
        self.streaming = streaming
        while True:
            global streaming_mode

            if self.streaming.image_overview[0][0][0] != 0.0:
                img_con_flag = True
            if img_con_flag is True:
                if streaming_mode == 0:
                    self.rgbImage = self.streaming.image_overview
                elif streaming_mode == 1:
                    self.rgbImage = self.streaming.image_cobot
                h, w, ch = self.rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(self.rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                
                p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)
            else:
                print("waiting for flag  " +str(img_con_flag))


class QUpdateDeviceInfo(QThread):
    changeDeviceInfo = pyqtSignal(str)

    def __init__(self, parent=None):
        super(QUpdateDeviceInfo, self).__init__(parent)

    def run(self):
        self.sql = SqlHelper(host='192.168.60.21', username='wjYun', password='0000', port=3306, database='SmartLab')
        while True:
            try:
                device_info = self.sql.select('device_info', conds="id=(SELECT MAX(id) FROM device_info)")[0]
                device_info['time_stamp'] = str(device_info['time_stamp'])
                self.changeDeviceInfo.emit(json.dumps(device_info))
                time.sleep(3.0)
            except:
                print("[ERROR] Device information update error !!!")
                pass

class QUpdateTestInfo(QThread):
    changeTestInfo = pyqtSignal(str)

    def __init__(self, parent=None):
        super(QUpdateTestInfo, self).__init__(parent)

    def run(self):
        global smartlab_cmd
        self.smartlab_cmd = smartlab_cmd
        self.sql = SqlHelper(host='192.168.60.21', username='wjYun', password='0000', port=3306, database='SmartLab')
        while True:
            try:
                fields = ['subject_name', 'Status', 'Thickness', 'Length', 'Width', 'E_modulus', 'U_stress']
                header_id = self.smartlab_cmd['setup_doe']['header_id']
                test_info = self.sql.select('result', fields=fields, conds='subject_name like "%{}%"'.format(header_id))
                self.changeTestInfo.emit(json.dumps(test_info))
                time.sleep(3.0)
            except:
                print("[ERROR] Device information update error !!!")
                pass

class QSendCommand(QThread):
    send = pyqtSignal()

    def __init__(self, parent=None):
        super(QSendCommand, self).__init__(parent)

    def run(self):
        global init_flag
        global smartlab_cmd
        global smartlab
        self.smartlab = smartlab
        while True:
            self.init_flag = init_flag
            self.smartlab_cmd = smartlab_cmd
            try:
                if self.init_flag == True:
                    print("[DEBUG] SmartLab command to server: \n{}".format(self.smartlab_cmd))
                    init_flag = False
                    response = self.smartlab.send(self.smartlab_cmd)
                    smartlab_cmd['test_step'] = -1 if self.smartlab_cmd['test_step'] == 1 else 1
                    print("[DEBUG] Response from SmartLab server: \n{}".format(response))
                    self.send.emit()
                    time.sleep(1.0)

            except:
                print("[ERROR] Device information update error !!!")
                pass


class SmartLAB_GUI(QMainWindow, QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi("SmartLab_GUI.ui", self)
        QDialog().setFixedSize(self.size())

        self.init_flag = False
        global init_flag
        init_flag = self.init_flag
        

        self.sql = SqlHelper(host='192.168.60.21', username='wjYun', password='0000', port=3306, database='SmartLab')


        self.smartlab = SmartLabClient(ip='192.168.60.21')
        global smartlab
        smartlab = self.smartlab

        self.smartlab_cmd = dict()
        self.smartlab_cmd['test_mode'] = 'auto'
        self.smartlab_cmd['test_step'] = -1
        self.smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1']
        self.smartlab_cmd['setup_doe'] = dict()

        global smartlab_cmd
        smartlab_cmd = self.smartlab_cmd


        self.btn_doe_create.clicked.connect(self.cb_btn_doe_create)
        self.btn_control_run.clicked.connect(self.cb_btn_control_run)
        self.btn_control_pause.clicked.connect(self.cb_btn_control_pause)
        self.btn_control_stop.clicked.connect(self.cb_btn_control_stop)
        self.btn_exp_export.clicked.connect(self.cb_btn_exp_export)
        self.cbx_control.currentIndexChanged.connect(self.cb_cbx_control)
        self.btn_doe_apply.clicked.connect(self.cb_btn_doe_apply)


        qthread_device = QUpdateDeviceInfo(self)
        qthread_device.changeDeviceInfo.connect(self.setDeviceTable)
        qthread_device.start()

        qthread_test = QUpdateTestInfo(self)
        qthread_test.changeTestInfo.connect(self.setTestTable)
        qthread_test.start()

        qthread_cmd = QSendCommand(self)
        qthread_cmd.send.connect(self.sendCommand)
        qthread_cmd.start()

        qthread_streaming = QCameraStreaming(self)
        qthread_streaming.changePixmap.connect(self.setImageStreaming)
        qthread_streaming.start()


        # self.cam360_screen.setStyleSheet("background-color: rgb(84, 84, 84);")
        # self.webview = QWebEngineView(self.cam360_screen)
        # self.webview.setUrl(QUrl("https://www.youtube.com/watch?v=avmSkeJwT0o"))
        # self.webview.setGeometry(0, 0, 500, 300)


            
    @pyqtSlot()
    def sendCommand(self):
        print('[DEBUG] SmartLab command is sent to the Server.')

    @pyqtSlot(str)
    def setDeviceTable(self, device_info_str):
        device_info = json.loads(device_info_str)
        self.updateDeviceTable(device_info)

    @pyqtSlot(str)
    def setTestTable(self, test_info_str):
        test_info = json.loads(test_info_str)
        self.updateTestTable(test_info)

    @pyqtSlot(QImage)
    def setImageStreaming(self, image):
        global streaming_mode
        self.image_overview.setPixmap(QPixmap.fromImage(image))

    @pyqtSlot(QImage)
    def setImage_cobot_eef(self, image):
        self.image_cobot_eef.setPixmap(QPixmap.fromImage(image))

    @pyqtSlot(QImage)
    def setImage_cam_360(self, image):
        self.cam360_screen.setPixmap(QPixmap.fromImage(image))

    def updateTestTable(self, test_info):
        n_col = len(list(test_info[0]))
        n_row = len(test_info)

        self.table_exp_info.setRowCount(n_row)
        self.table_exp_info.setColumnCount(n_col)
        self.table_exp_info.setHorizontalHeaderLabels((list(test_info[0].keys())))
    
        for row, item_list in enumerate(test_info):
            for col, key in enumerate(item_list):
                item = list(test_info[row].values())[col]
                newitem = QTableWidgetItem(str(item_list[key]))
                self.table_exp_info.setItem(row, col, newitem)

    def updateDeviceTable(self, device_status):
        for device_id in device_status:
            try:
                test_info = device_status[device_id]
                test_info = json.loads(test_info)
            
                if   device_id.find('amr') != -1:
                    self.table_amr.setColumnCount(1)
                    self.table_amr.setRowCount(len(test_info))
                    self.table_amr.setVerticalHeaderLabels((list(test_info.keys())))
                    for row, key in enumerate(test_info):
                        newitem = QTableWidgetItem(str(test_info[key]))
                        self.table_amr.setItem(row, 0, newitem)
                elif device_id.find('cobot') != -1:
                    self.table_cobot.setColumnCount(1)
                    self.table_cobot.setRowCount(len(test_info))
                    self.table_cobot.setVerticalHeaderLabels((list(test_info.keys())))
                    for row, key in enumerate(test_info):
                        newitem = QTableWidgetItem(str(test_info[key]))
                        self.table_cobot.setItem(row, 0, newitem)
                elif device_id.find('MS') != -1:
                    self.table_omm.setColumnCount(1)
                    self.table_omm.setRowCount(len(test_info))
                    self.table_omm.setVerticalHeaderLabels((list(test_info.keys())))
                    for row, key in enumerate(test_info):
                        newitem = QTableWidgetItem(str(test_info[key]))
                        self.table_omm.setItem(row, 0, newitem)
                elif device_id.find('instron') != -1:
                    self.table_instron.setColumnCount(1)
                    self.table_instron.setRowCount(len(test_info))
                    self.table_instron.setVerticalHeaderLabels((list(test_info.keys())))
                    for row, key in enumerate(test_info):
                        newitem = QTableWidgetItem(str(test_info[key]))
                        self.table_instron.setItem(row, 0, newitem)
                elif device_id.find('printer1') != -1:
                    self.table_printer_1.setColumnCount(1)
                    self.table_printer_1.setRowCount(len(test_info))
                    self.table_printer_1.setVerticalHeaderLabels((list(test_info.keys())))
                    for row, key in enumerate(test_info):
                        newitem = QTableWidgetItem(str(test_info[key]))
                        self.table_printer_1.setItem(row, 0, newitem)
                elif device_id.find('printer2') != -1:
                    self.table_printer_2.setColumnCount(1)
                    self.table_printer_2.setRowCount(len(test_info))
                    self.table_printer_2.setVerticalHeaderLabels((list(test_info.keys())))
                    for row, key in enumerate(test_info):
                        newitem = QTableWidgetItem(str(test_info[key]))
                        self.table_printer_2.setItem(row, 0, newitem)
                elif device_id.find('printer3') != -1:
                    self.table_printer_3.setColumnCount(1)
                    self.table_printer_3.setRowCount(len(test_info))
                    self.table_printer_3.setVerticalHeaderLabels((list(test_info.keys())))
                    for row, key in enumerate(test_info):
                        newitem = QTableWidgetItem(str(test_info[key]))
                        self.table_printer_3.setItem(row, 0, newitem)
                elif device_id.find('printer4') != -1:
                    self.table_printer_4.setColumnCount(1)
                    self.table_printer_4.setRowCount(len(test_info))
                    self.table_printer_4.setVerticalHeaderLabels((list(test_info.keys())))
                    for row, key in enumerate(test_info):
                        newitem = QTableWidgetItem(str(test_info[key]))
                        self.table_printer_4.setItem(row, 0, newitem)
            except:
                pass

    def cb_btn_exp_export(self):
        print("[DEBUG] 'Experiment - Export' button clicked !!! (TBD)")


    def cb_btn_control_run(self):
        print("[DEBUG] 'Control - Run' button clicked !!! (TBD)")

        self.init_flag = True
        global init_flag
        init_flag = self.init_flag
        print(init_flag)

        self.smartlab_cmd['test_step'] = 1
        global smartlab_cmd
        smartlab_cmd = self.smartlab_cmd

        print("[DEBUG] SmartLab Command: \n{}".format(self.smartlab_cmd))
        
    
    def cb_btn_control_pause(self):
        print("[DEBUG] 'Control - Pause' button clicked !!! (TBD)")
        global streaming_mode
        streaming_mode = 0

    def cb_btn_control_stop(self):
        print("[DEBUG] 'Control - Pause' button clicked !!! (TBD)")
        global streaming_mode
        streaming_mode = 1
        
    def cb_btn_doe_apply(self):
        self.txt_doe_exp_name.setText(self.DOE_window.doe['header_id'])
        self.txt_doe_exp_type.setText(self.DOE_window.doe['experiment_type'])
        self.txt_doe_doe_type.setText(self.DOE_window.doe['doe_type'])
        self.txt_doe_doe_desc.setText(str(self.DOE_window.doe['factors'])+'\noptions : ' + str(self.DOE_window.doe['option']))
        self.smartlab_cmd['setup_doe'] = self.DOE_window.doe
        global smartlab_cmd
        smartlab_cmd = self.smartlab_cmd
        
    def cb_btn_doe_create(self):
        self.DOE_window = DOE_Window()
        self.DOE_window.exec()


    def cb_cbx_control(self):
        self.smartlab_cmd['test_mode'] = self.cbx_control.currentText()
        global smartlab_cmd
        smartlab_cmd = self.smartlab_cmd
        print("[DEBUG] Execution mode: {}".format(self.smartlab_cmd['test_mode']))



class DOE_Window(QDialog) :
    def __init__(self) :
        super().__init__()
        uic.loadUi("DoE_Create.ui", self)
        # self.setupUi(self)
        self.cbx_doe_type.textActivated.connect(self.cb_cbx_doe_type)
        self.btn_doe_ok.clicked.connect(self.cb_btn_doe_ok)

    def cb_cbx_doe_type(self, QString):
        if QString == 'GENERALIZED_FACTORIAL':
            self.groupBox_options.show()
        else:
            self.groupBox_options.hide()

    def cb_btn_doe_ok(self):
        self.doe = dict()
        self.doe['header_id'] = self.txt_exp_name.text()
        self.doe['experiment_type'] = self.cbx_exp_type.currentText()
        self.doe['doe_type']        = self.cbx_doe_type.currentText()
        factors = list()
        options = list()
        if self.checkBox_linedistance.isChecked():
            factor=dict()
            factor['factor_name'] = self.checkBox_linedistance.text()
            factor['factor_range'] = self.txt_linedistance_range.text().split(',')
            factors.append(factor)
            options.append(self.txt_linedistance_option.text().split(','))
        if self.checkBox_linewidth.isChecked():
            factor=dict()
            factor['factor_name'] = self.checkBox_linewidth.text()
            factor['factor_range'] = self.txt_linewidth_range.text().split(',')
            factors.append(factor)
            options.append(self.txt_linewidth_option.text().split(','))
        if self.checkBox_layerheight.isChecked():
            factor=dict()
            factor['factor_name'] = self.checkBox_layerheight.text()
            factor['factor_range'] = self.txt_layerheight_range.text().split(',')
            factors.append(factor)
            options.append(self.txt_layerheight_option.text().split(','))
        if self.checkBox_rasterangle.isChecked():
            factor=dict()
            factor['factor_name'] = self.checkBox_rasterangle.text()
            # factor['factor_range'] = self.txt_rasterangle_range.text()
            factors.append(factor)
            options.append(self.txt_rasterangle_option.text().split('/'))
        if self.checkBox_infillpatern.isChecked():
            factor=dict()
            factor['factor_name'] = self.checkBox_linedistance.text()
            # factor['factor_range'] = self.cbx_infill_pattern.currentText()
            factors.append(factor)
            # options.append(self.txt_linewidth_option.text())
        self.doe['factors'] = factors
        self.doe['option'] = options

        print(self.doe)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SmartLAB_GUI()
    gui.show()
    app.exec_()
    print('good')
