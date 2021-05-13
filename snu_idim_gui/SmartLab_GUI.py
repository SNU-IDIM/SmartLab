#! /usr/bin/env python
# -*- coding: utf-8 -*-
import ast
import json
import os
import sys
import time
from threading import Thread
import base64
from PIL import Image
import io
import cv2
from PyQt5 import QtCore, uic
from PyQt5.QtCore import Qt, QThread, QUrl, pyqtSignal, pyqtSlot
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import *

from SmartLab_Client import SmartLabClient

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from Cam_Streaming_Client import Cam_Streaming_Client
from SqlHelper import SqlHelper
import pafy
# class QtThread(QThread):
#     changePixmap = pyqtSignal(QImage)

#     def run(self):
#         cap = cv2.VideoCapture(0)
#         while True:
#             ret, frame = cap.read()
#             if ret:
#                 # https://stackoverflow.com/a/55468544/6622587
#                 rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#                 h, w, ch = rgbImage.shape
#                 bytesPerLine = ch * w
#                 convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
#                 p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
#                 self.changePixmap.emit(p)


class Overview(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if ret:
                # https://stackoverflow.com/a/55468544/6622587
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)



class Roboteefview(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        cap = cv2.VideoCapture(1)
        while True:
            ret, frame = cap.read()
            if ret:
                # https://stackoverflow.com/a/55468544/6622587
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)

class cam360(QThread):
    changePixmap = pyqtSignal(QImage)

    def run(self):
        url = 'https://www.youtube.com/watch?v=avmSkeJwT0o'
        '''
        while True:
            vPafy = pafy.new(url)
            play = vPafy.getbest(preftype="mp4")
            cap = cv2.VideoCapture(play.url)
            # cap = cv2.VideoCapture(play.url)
            ret, frame = cap.read()
            if ret:
                # https://stackoverflow.com/a/55468544/6622587
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
                p = convertToQtFormat.scaled(360, 240, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)
        '''


class SmartLAB_GUI(QMainWindow, QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi("SmartLab_GUI.ui", self)
        QDialog().setFixedSize(self.size())

        self.init_flag = False

        self.sql = SqlHelper(host='192.168.0.88', username='wjYun', password='0000', port=3306, database='SmartLab')
        self.thread_server = Thread(target=self.updateStatus)
        self.thread_server.start()

        self.smartlab = SmartLabClient(ip='192.168.60.21')
        self.streaming = Cam_Streaming_Client(ip='192.168.60.21', cam_list=['cobot'])


        self.smartlab_cmd = dict()
        self.smartlab_cmd['test_mode'] = 'auto'
        self.smartlab_cmd['test_step'] = -1
        self.smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1', 'printer2', 'printer3']
        self.smartlab_cmd['setup_doe'] = dict()


        self.btn_doe_create.clicked.connect(self.cb_btn_doe_create)
        self.btn_control_run.clicked.connect(self.cb_btn_control_run)
        self.btn_control_pause.clicked.connect(self.cb_btn_control_pause)
        self.btn_control_stop.clicked.connect(self.cb_btn_control_stop)
        self.btn_exp_export.clicked.connect(self.cb_btn_exp_export)
        self.cbx_control.currentIndexChanged.connect(self.cb_cbx_control)
        self.btn_doe_apply.clicked.connect(self.cb_btn_doe_apply)
        self.btn_exp_detail.clicked.connect(self.cb_btn_exp_detail)

        # create a label
        # self.label = QLabel(self)
        # self.label.move(280, 120)
        # self.label.resize(640, 480)

        self.showlogo()

        #The camera thread setting function
        # th = QtThread(self)
        # th.changePixmap.connect(self.setImageOverview)
        # th.start()

        ov_cam =Overview(self)
        ov_cam.changePixmap.connect(self.setImageOverview)
        ov_cam.start()


        rb_cam =Roboteefview(self)
        rb_cam.changePixmap.connect(self.setImage_cobot_eef)
        rb_cam.start()

        cam_360 =cam360(self)
        cam_360.changePixmap.connect(self.setImage_cam_360)
        cam_360.start()

        # self.cam360_screen.setStyleSheet("background-color: rgb(84, 84, 84);")
        # self.webview = QWebEngineView(self.cam360_screen)
        # self.webview.setUrl(QUrl("https://www.youtube.com/watch?v=avmSkeJwT0o"))
        # self.webview.setGeometry(0, 0, 500, 300)

    
    def updateDeviceTable(self, device_status):
        for device_id in device_status:
            try:
                test_info = device_status[device_id]
                test_info = json.loads(test_info)
            
                if   device_id.find('amr') != -1:
                    self.AMR_status.setText(str(test_info['status']))
                    self.AMR_SN.setText(str(test_info['subject_name'])) ####TBD####
                    self.AMR_task.setText(str(test_info['current_work'])) ####TBD####
                elif device_id.find('cobot') != -1:
                    self.Cobot_comp.setText(str(test_info["compressor"]))
                    self.Cobot_current.setText(str(test_info['pose'])) 
                    self.Cobot_gripper.setText(str(test_info['gripper_type']))
                    self.Cobot_status.setText(str(test_info['recent_work'])) ####TBD####
                elif device_id.find('MS') != -1:
                    self.OMM_SN.setText(str(test_info["recent_work"]))####TBD####
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


    @pyqtSlot(QImage)
    def setImageOverview(self, image):
        self.image_overview.setPixmap(QPixmap.fromImage(image))

    def setImage_cobot_eef(self, image):
        self.image_cobot_eef.setPixmap(QPixmap.fromImage(image))


    def setImage_cam_360(self, image):
        self.cam360_screen.setPixmap(QPixmap.fromImage(image))

    def updateStatus(self):
        '''
            1. Experiment status update 부분
            2. Device status update 부분
            3. Progree 관련 부분(progress bar 등)
        '''
        while True:
            try:
                try:
                    device_info = self.sql.select('device_info', conds="id=(SELECT MAX(id) FROM device_info)")[0]
                    self.updateDeviceTable(device_info)
                except:
                    print("[ERROR] Device information update error !!!")
                    pass

                try:
                    fields = ['subject_name', 'Status', 'Thickness', 'Length', 'Width', 'E_modulus', 'U_stress']
                    header_id = self.smartlab_cmd['setup_doe']['header_id']
                    test_info = self.sql.select('result', fields=fields, conds='subject_name like "%{}%"'.format(header_id))
                    n_col = len(list(test_info[0]))
                    n_row = len(test_info)

                    self.table_exp_info.setRowCount(n_row)
                    self.table_exp_info.setColumnCount(n_col)
                    # vbox = QVBoxLayout(self)
                    # vbox.addWidget(self.table_exp_info)
                    print((list(test_info[0].keys())))
                    self.table_exp_info.setHorizontalHeaderLabels((list(test_info[0].keys())))
                
                    for row, item_list in enumerate(test_info):
                        for col, key in enumerate(item_list):
                            item = list(test_info[row].values())[col]
                            newitem = QTableWidgetItem(str(item_list[key]))
                            self.table_exp_info.setItem(row, col, newitem)
                except:
                    print('[ERROR] Experiment information update error !!!')
                
                if self.init_flag == True:
                    print("[DEBUG] SmartLab command to server: \n{}".format(self.smartlab_cmd))
                    self.init_flag = False
                    response = self.smartlab.send(self.smartlab_cmd)
                    self.smartlab_cmd['test_step'] = -1 if self.smartlab_cmd['test_step'] == 1 else 1
                    print("[DEBUG] Response from SmartLab server: \n{}".format(response))

            except:
                print("[ERROR] SmartLab information update error !!!")

            time.sleep(1.0)

    def cb_btn_exp_detail(self):
        self.graphicsItem = None
        sql = SqlHelper(host='192.168.0.88', username='wjYun', password='0000', port=3306, database='SmartLab')

        selectedRow = self.table_exp_info.currentRow()
        if selectedRow != -1:
            print('selectedRow', selectedRow)
            # selectedIdx = self.table_exp_info.selectedIndexes()

            selected_item = self.table_exp_info.item(selectedRow, 0)
            selected_subject_name = selected_item.text()
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
        self.smartlab_cmd['test_step'] = 1
        print("[DEBUG] SmartLab Command: \n{}".format(self.smartlab_cmd))
        
    
    def cb_btn_control_pause(self):
        print("[DEBUG] 'Control - Pause' button clicked !!! (TBD)")

    def cb_btn_control_stop(self):
        print("[DEBUG] 'Control - Pause' button clicked !!! (TBD)")
        
    def cb_btn_doe_apply(self):
        self.txt_doe_exp_name.setText(self.DOE_window.doe['header_id'])
        self.txt_doe_exp_type.setText(self.DOE_window.doe['experiment_type'])
        self.txt_doe_doe_type.setText(self.DOE_window.doe['doe_type'])
        self.txt_doe_doe_desc.setText(str(self.DOE_window.doe['factors'])+'\noptions : ' + str(self.DOE_window.doe['option']))
        self.smartlab_cmd['setup_doe'] = self.DOE_window.doe

    def cb_btn_doe_create(self):
        self.DOE_window = DOE_Window()
        self.DOE_window.exec()

        '''
        doe_factors = []
        doe_options = []

        for i in range(self.table_doe_factors.rowCount()):
            factor = dict()
            option = dict()
            factor['factor_name'] = self.table_doe_factors.item(i, 0).text()
            try:
                factor_range = ast.literal_eval(self.table_doe_factors.item(i, 1).text())
                factor['factor_range'] = factor_range
            except:
                pass
            option = ast.literal_eval(self.table_doe_factors.item(i, 2).text())
            doe_factors.append(factor)
            doe_options.append(option)

        self.smartlab_cmd['setup_doe']['header_id']       = self.txt_doe_exp_name.text()
        self.smartlab_cmd['setup_doe']['experiment_type'] = self.txt_doe_exp_type.text()
        self.smartlab_cmd['setup_doe']['doe_type']        = 3
        self.smartlab_cmd['setup_doe']['factors']         = doe_factors
        self.smartlab_cmd['setup_doe']['option']          = doe_options
        for key in self.smartlab_cmd:
            print("[DEBUG] {}: {}".format(key, self.smartlab_cmd[key]))
        '''


    def cb_cbx_control(self):
        self.smartlab_cmd['test_mode'] = self.cbx_control.currentText()
        print("[DEBUG] Execution mode: {}".format(self.smartlab_cmd['test_mode']))

    def showlogo(self):
        SNU_image = QPixmap()
        SNU_image.load('./src/snu.jpg')
        SNU_image = SNU_image.scaled(self.logo_SNU.width(), self.logo_SNU.height())
        IDIM_image = QPixmap()
        IDIM_image.load('./src/idim.png')
        IDIM_image = IDIM_image.scaled(self.logo_IDIM.width(), self.logo_IDIM.height())

        self.logo_IDIM.setPixmap(IDIM_image)
        self.logo_SNU.setPixmap(SNU_image)


    # def cbx_device_cb(self):
    #     try:
    #         device_id = self.cbx_device.currentText()
    #         if device_id.find('/') != -1:   device_id = device_id.split('/')[1]
    #         device_info = self.sql.select(tablename='device_info', fields=device_id)
    #         n_col = len(list(json.loads(device_info[0]).keys()))
    #         n_row = len(device_info)
            
    #         self.table_devcie_info.setRowCount(n_row)
    #         self.table_devcie_info.setColumnCount(n_col)
    #         self.table_devcie_info.setHorizontalHeaderLabels((list(json.loads(device_info[0]).keys())))
    #         for row, item_list in enumerate(device_info):
    #             for col, key in enumerate(json.loads(item_list)):
    #                 item = (list(json.loads(device_info[row]).values())[col])
    #                 newitem = QTableWidgetItem(str(item))
    #                 self.table_devcie_info.setItem(row, col, newitem)
    #     except:
    #         pass

    #     fields = ['subject_name', 'Status', 'Thickness', 'Length', 'Width', 'E_modulus', 'U_stress']
    #     header_id = self.smartlab_cmd['setup_doe']['header_id']
    #     test_info = self.sql.select('result', fields=fields, conds='subject_name like "%{}%"'.format(header_id))
    #     n_col = len(list(test_info[0]))
    #     n_row = len(test_info)
    #     self.table_test_info.setRowCount(n_row)
    #     self.table_test_info.setColumnCount(n_col)
    #     vbox = QVBoxLayout(self)
    #     vbox.addWidget(self.table_test_info)
    #     self.table_test_info.setHorizontalHeaderLabels((list(test_info[0].keys())))
    
    #     for row, item_list in enumerate(test_info):
    #         for col, key in enumerate(item_list):
    #             item = list(test_info[row].values())[col]
    #             newitem = QTableWidgetItem(str(item_list[key]))
    #             self.table_test_info.setItem(row, col, newitem)

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
