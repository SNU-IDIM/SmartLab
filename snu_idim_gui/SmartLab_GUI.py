#! /usr/bin/env python
# -*- coding: utf-8 -*-
import ast
import json
import os
import sys
import time
from threading import Thread
import base64
import numpy as np
from PIL import Image
import io
from PyQt5 import QtCore, uic
from PyQt5.QtCore import Qt, QThread, QUrl, pyqtSignal, pyqtSlot
from PyQt5.QtGui import *
# from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import *
import webbrowser

from SmartLab_Client import SmartLabClient

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from Cam_Streaming_Client import Cam_Streaming_Client
from SqlHelper import SqlHelper
import pafy


global streaming
streaming = Cam_Streaming_Client(cam_list=['overview', 'cobot_eef', 'cobot_front'])
img_con_flag = False

image_cobot_eef    = np.empty((480, 640, 3))
image_overview = np.empty((480, 640, 3))

global streaming_mode
streaming_mode = 'overview'

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
                if streaming_mode == 'overview':
                    self.rgbImage = self.streaming.image_overview
                elif streaming_mode == 'cobot_eef':
                    self.rgbImage = self.streaming.image_cobot_eef
                elif streaming_mode == 'cobot_front':
                    self.rgbImage = self.streaming.image_cobot_front
                h, w, ch = self.rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(self.rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                
                p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)
                time.sleep(0.1)
            else:
                # print("waiting for flag  " +str(img_con_flag))
                pass


class QUpdateDeviceInfo(QThread):
    changeDeviceInfo = pyqtSignal(str)

    def __init__(self, parent=None):
        super(QUpdateDeviceInfo, self).__init__(parent)

    def run(self):
        self.sql = SqlHelper(host='192.168.0.88', username='wjYun', password='0000', port=3306, database='SmartLab')
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
        self.sql = SqlHelper(host='192.168.0.88', username='wjYun', password='0000', port=3306, database='SmartLab')
        while True:
            try:
                fields = ['subject_name', 'Status', 'Thickness', 'Length', 'Width', 'E_modulus', 'U_stress']
                header_id = self.smartlab_cmd['setup_doe']['header_id']
                test_info = self.sql.select('result', fields=fields, conds='subject_name like "%{}%"'.format(header_id))
                self.changeTestInfo.emit(json.dumps(test_info))
                time.sleep(3.0)
            except:
                print("[ERROR] Test information update error !!!")
                pass

class QUpdateSystemInfo(QThread):
    changeSystemInfo = pyqtSignal(str)

    def __init__(self, parent=None):
        super(QUpdateSystemInfo, self).__init__(parent)

    def run(self):
        self.sql = SqlHelper(host='192.168.0.88', username='wjYun', password='0000', port=3306, database='SmartLab')
        while True:
            try:
                system_info = self.sql.select('system_status', conds="id=(SELECT MAX(id) FROM system_status)")[0]
                print(system_info)
                self.changeSystemInfo.emit(json.dumps(system_info))
                time.sleep(3.0)
            except:
                print("[ERROR] System information update error !!!")
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
        

        self.sql = SqlHelper(host='192.168.0.88', username='wjYun', password='0000', port=3306, database='SmartLab')

        self.smartlab = SmartLabClient(ip='192.168.0.88')
        global smartlab
        smartlab = self.smartlab

        self.test_info = list()

        self.smartlab_cmd = dict()
        self.smartlab_cmd['test_mode'] = 'auto'
        self.smartlab_cmd['test_step'] = -1
        self.smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer2', 'printer3']
        self.smartlab_cmd['setup_doe'] = dict()

        global smartlab_cmd
        smartlab_cmd = self.smartlab_cmd

        self.system_status = dict()
        self.system_status['control_mode'] = 'auto'
        self.system_status['control_step'] = '0'
        self.system_status['control_status'] = 'Waiting'


        self.btn_doe_create.clicked.connect(self.cb_btn_doe_create)
        self.btn_control_run.clicked.connect(self.cb_btn_control_run)
        self.btn_control_pause.clicked.connect(self.cb_btn_control_pause)
        self.btn_control_stop.clicked.connect(self.cb_btn_control_stop)
        self.btn_exp_export.clicked.connect(self.cb_btn_exp_export)
        self.cbx_control.currentIndexChanged.connect(self.cb_cbx_control)
        #self.btn_doe_apply.clicked.connect(self.cb_btn_doe_apply)
        self.btn_exp_detail.clicked.connect(self.cb_btn_exp_detail)
        self.btn_overview.clicked.connect(self.cb_btn_overview)
        self.btn_cobot_eef.clicked.connect(self.cb_btn_cobot_eef)
        self.btn_cobot_front.clicked.connect(self.cb_btn_cobot_front)
        self.btn_360cam.clicked.connect(self.cb_btn_360cam)
        self.btn_camera_popup.clicked.connect(self.cb_btn_popup)

        self.showlogo()

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

        qthread_sys_status = QUpdateSystemInfo(self)
        qthread_sys_status.changeSystemInfo.connect(self.setSystemStatus)
        qthread_sys_status.start()
        
        # self.cam360_screen.setStyleSheet("background-color: rgb(84, 84, 84);")
        # self.webview = QWebEngineView(self.cam360_screen)
        # self.webview.setUrl(QUrl("https://www.youtube.com/watch?v=avmSkeJwT0o"))
        # self.webview.setGeometry(0, 0, 500, 300)
            
    @pyqtSlot()
    def sendCommand(self):
        print('[DEBUG] SmartLab command is sent to the Server.')

    @pyqtSlot(str)
    def setSystemStatus(self, system_info_str):
        self.system_status = json.loads(system_info_str)
        self.txt_system_step.setText(str(self.system_status['control_step']))
        self.updateSystemStep(self.system_status['control_step'])
        n_test_total = len(self.test_info)
        n_test_done = 0
        for test_info in self.test_info:
            if test_info['Status'] == 'Done':
                n_test_done += 1
        try:
            progress = (n_test_done / n_test_total) * 100.0
            self.pBar_system.setValue(progress)
        except:
            progress = 0
            self.pBar_system.setValue(0)

        self.txt_system_progress.setText('{}%\n({}/{} runs)'.format(round(progress, 1), n_test_done, n_test_total))
        if self.smartlab_cmd['test_mode'] == 'auto':
            self.btn_control_run.setEnabled(True)
            self.txt_system_mode.setText('Auto')
            self.txt_system_guide.setText('The system is running automatically !')
        elif self.smartlab_cmd['test_mode'] == 'step':
            self.txt_system_mode.setText('Step')
            if self.system_status['control_status'] == 'Waiting':
                self.btn_control_run.setEnabled(True)
                self.txt_system_guide.setText('Please click the \'RUN\' button !')
            else:
                self.txt_system_guide.setText('Please wait until the task is done !')
                self.btn_control_run.setEnabled(False)


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
        self.image_streaming.setPixmap(QPixmap.fromImage(image))

    def updateSystemStep(self, step):
        step = int(step)
        if step == 0:
            self.txt_system_status.setText('Fabricating specimens')
        elif step == 1:
            self.txt_system_status.setText('Get printing bed from 3D printer')
        elif step == 2:
            self.txt_system_status.setText('Measuring dimension of the specimen')
        elif step == 3:
            self.txt_system_status.setText('Detaching the specimen')
        elif step == 4:
            self.txt_system_status.setText('Feeding specimen to Universal testing machine')
        elif step == 5:
            self.txt_system_status.setText('Experiment setting')
        elif step == 6:
            self.txt_system_status.setText('Conducting an experiment (Tensile test)')

    def updateTestTable(self, test_info):
        self.test_info = test_info
        n_col = len(list(test_info[0]))
        n_row = len(test_info)

        self.table_exp_info.setRowCount(n_row)
        self.table_exp_info.setColumnCount(n_col)
        fields = ['subject_name', 'Status', 'Thickness', 'Length', 'Width', 'E_modulus', 'U_stress']
        self.table_exp_info.setHorizontalHeaderLabels(fields)
    
        for row, item_list in enumerate(test_info):
            for col, key in enumerate(fields):
                item = test_info[row][key] if test_info[row][key] != None else ''
                newitem = QTableWidgetItem(str(item))
                self.table_exp_info.setItem(row, col, newitem)

    def updateDeviceTable(self, device_status):
        for device_id in device_status:
            try:
                test_info = device_status[device_id]
                test_info = json.loads(test_info)
                if   device_id.find('amr') != -1:
                    self.AMR_status.setText(str(test_info['status']))
                    self.AMR_task.setText(str(test_info['current_work']))
                    self.AMR_SN.setText(str(test_info['subject_name'])) ##TBD
                elif device_id.find('cobot') != -1:
                    self.Cobot_status.setText(str(test_info['status']))
                    self.Cobot_current.setText(str(test_info['current_work'])) 
                    self.Cobot_comp.setText(str(test_info["compressor"]))
                    self.Cobot_gripper.setText(str(test_info['gripper_type']))
                elif device_id.find('MS') != -1:
                    self.OMM_SN.setText(str(test_info["subject_name"]))
                    self.OMM_status.setText(str(test_info['status']))
                elif device_id.find('instron') != -1:
                    self.Instron_SN.setText(str(test_info["subject_name"]))
                    self.Instron_status.setText(str(test_info['status']))
                elif device_id.find('printer1') != -1:
                    self.TDP1_SN.setText(str(test_info["subject_name"]))
                    self.TDP1_status.setText(str(test_info['status']))
                    self.TDP1_bed.setText(str(test_info['bed_temperature']))
                    self.TDP1_nozzle.setText(str(test_info['nozzle_temperature']))
                    self.TDP1_time.setText(str(test_info['time_left']))
                elif device_id.find('printer2') != -1:
                    self.TDP2_SN.setText(str(test_info["subject_name"]))
                    self.TDP2_status.setText(str(test_info['status']))
                    self.TDP2_bed.setText(str(test_info['bed_temperature']))
                    self.TDP2_nozzle.setText(str(test_info['nozzle_temperature']))
                    self.TDP2_time.setText(str(test_info['time_left']))
                elif device_id.find('printer3') != -1:
                    self.TDP3_SN.setText(str(test_info["subject_name"]))
                    self.TDP3_status.setText(str(test_info['status']))
                    self.TDP3_bed.setText(str(test_info['bed_temperature']))
                    self.TDP3_nozzle.setText(str(test_info['nozzle_temperature']))
                    self.TDP3_time.setText(str(test_info['time_left']))
                elif device_id.find('printer4') != -1:
                    self.TDP4_SN.setText(str(test_info["subject_name"]))
                    self.TDP4_status.setText(str(test_info['status']))
                    self.TDP4_bed.setText(str(test_info['bed_temperature']))
                    self.TDP4_nozzle.setText(str(test_info['nozzle_temperature']))
                    self.TDP1_time.setText(str(test_info['time_left']))
            except:
                pass


    def cb_btn_exp_detail(self):
        self.graphicsItem = None
        sql = SqlHelper(host='192.168.0.88', username='wjYun', password='0000', port=3306, database='SmartLab')

        selectedRow = self.table_exp_info.currentRow()
        if selectedRow != -1:
            print('selectedRow', selectedRow)
            # selectedIdx = self.table_exp_info.selectedIndexes()

            selected_item = self.table_exp_info.item(selectedRow, 0)
            selected_subject_name = selected_item.text()
            print('[DEBUG]',selected_subject_name)
            # selected_subject_name = 'calb'  ##for debug
            factor_list = list()
            factor_status_list = list()
            fields = ['Status', 'Thickness', 'Length', 'Width', 'E_modulus', 'U_stress', 'Plot']

            factors = self.smartlab_cmd['setup_doe']['factors']
            factor_count = len(factors)
            for i in range(factor_count):
                fields.append(factors[i]['factor_name'])
                factor_list.append(factors[i]['factor_name'])
                
            selected_item_info = sql.select(tablename='result', fields=fields, conds="subject_name=\'{}\'".format(selected_subject_name))[0]
            sql.close()

            Plot = selected_item_info['Plot']
            selected_item_info.pop('Plot')
            fields.remove('Plot')

            self.information_widget.setRowCount(len(fields))
            self.information_widget.setVerticalHeaderLabels(fields)
            for index, key in enumerate(fields):
                item = selected_item_info[key]
                print(item)
                QItem = QTableWidgetItem(str(item))
                self.information_widget.setItem(index, 0, QItem)

            if Plot is not None:
                Graph_binary    = base64.b64decode(Plot)
                image           = Image.open(io.BytesIO(Graph_binary))
                qPixmapVar = QPixmap()
                qPixmapVar.loadFromData(Graph_binary)
                qPixmapVar = qPixmapVar.scaled(self.graph.width(), self.graph.height())
                self.graph.setPixmap(qPixmapVar)

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
        

    def cb_btn_overview(self):
        global streaming_mode
        streaming_mode = 'overview'

    def cb_btn_cobot_eef(self):
        global streaming_mode
        streaming_mode = 'cobot_eef'

    def cb_btn_cobot_front(self):
        global streaming_mode
        streaming_mode = 'cobot_front'
    
    def cb_btn_360cam(self):
        webbrowser.open('http://www.youtube.com/channel/UCec1lf2ZtDUx1Qd7S0mKC3g/live')
    
    def cb_btn_control_pause(self):
        print("[DEBUG] 'Control - Pause' button clicked !!! (TBD)")

    def cb_btn_control_stop(self):
        print("[DEBUG] 'Control - Stop' button clicked !!! (TBD)")
        
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
        self.DOE_window.show()
        self.DOE_window.btn_doe_ok.clicked.connect(self.cb_btn_doe_apply)


    def cb_cbx_control(self):
        self.smartlab_cmd['test_mode'] = self.cbx_control.currentText()
        global smartlab_cmd
        smartlab_cmd = self.smartlab_cmd
        print("[DEBUG] Execution mode: {}".format(self.smartlab_cmd['test_mode']))

    def showlogo(self):
        SNU_image = QPixmap()
        SNU_image.load('./src/snu.png')
        SNU_image = SNU_image.scaled(self.logo_SNU.width(), self.logo_SNU.height())
        IDIM_image = QPixmap()
        IDIM_image.load('./src/idim.png')
        IDIM_image = IDIM_image.scaled(self.logo_IDIM.width(), self.logo_IDIM.height())

        self.logo_IDIM.setPixmap(IDIM_image)
        self.logo_SNU.setPixmap(SNU_image)

    def cb_btn_popup(self):
        self.Popup_window = Popup_Window()
        self.Popup_window.show()

        qthread_streaming_popup = QCameraStreaming(self)
        qthread_streaming_popup.changePixmap.connect(self.setImageStreamingPopup)
        qthread_streaming_popup.start()

    @pyqtSlot(QImage)
    def setImageStreamingPopup(self, image):
        self.Popup_window.image_streaming.setPixmap(QPixmap.fromImage(image))
        

class Popup_Window(QDialog):
    def __init__(self) :
        super().__init__()
        uic.loadUi("Popup.ui", self)

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
            #factor['factor_name'] = self.checkBox_linedistance.text()
            factor['factor_name'] = 'infill_line_distance'
            factor['factor_range'] = self.txt_linedistance_range.text().split(',')
            factors.append(factor)
            options.append(self.txt_linedistance_option.text().split(','))
        if self.checkBox_linewidth.isChecked():
            factor=dict()
            #factor['factor_name'] = self.checkBox_linewidth.text()
            factor['factor_name'] = 'infill_line_width'
            factor['factor_range'] = self.txt_linewidth_range.text().split(',')
            factors.append(factor)
            options.append(self.txt_linewidth_option.text().split(','))
        if self.checkBox_layerheight.isChecked():
            factor=dict()
            #factor['factor_name'] = self.checkBox_layerheight.text()
            factor['factor_name'] = 'layer_height'
            factor['factor_range'] = self.txt_layerheight_range.text().split(',')
            factors.append(factor)
            options.append(self.txt_layerheight_option.text().split(','))
        if self.checkBox_rasterangle.isChecked():
            factor=dict()
            #factor['factor_name'] = self.checkBox_rasterangle.text()
            factor['factor_name'] = 'infill_angles'
            # factor['factor_range'] = self.txt_rasterangle_range.text()
            factors.append(factor)
            options.append(self.txt_rasterangle_option.text().split('/'))
        if self.checkBox_infillpatern.isChecked():
            factor=dict()
            #factor['factor_name'] = self.checkBox_linedistance.text()
            factor['factor_name'] = 'infill_pattern'
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
