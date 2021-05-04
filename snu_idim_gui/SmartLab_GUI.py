#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os, time
import ast
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtCore
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtCore import QUrl
import json
import ast
from threading import Thread

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from SqlHelper import SqlHelper

from SmartLab_Client import SmartLabClient


class SmartLAB_GUI(QMainWindow, QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi("SmartLab_GUI.ui", self)
        QDialog().setFixedSize(self.size())

        self.init_flag = False

        self.sql = SqlHelper(host='192.168.60.21', username='wjYun', password='0000', port=3306, database='SmartLab')
        self.smartlab = SmartLabClient(ip='192.168.60.21')

        self.smartlab_cmd = dict()
        self.smartlab_cmd['test_mode'] = 'auto'
        self.smartlab_cmd['test_step'] = -1
        self.smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1', 'printer2', 'printer3']
        self.smartlab_cmd['setup_doe'] = dict()

        # self.btn_run.clicked.connect(self.btn_run_cb)
        self.btn_doe_create.clicked.connect(self.cb_btn_doe_create)
        self.btn_control_run.clicked.connect(self.cb_btn_control_run)
        self.btn_control_pause.clicked.connect(self.cb_btn_control_pause)
        self.btn_control_stop.clicked.connect(self.cb_btn_control_stop)
        self.btn_exp_export.clicked.connect(self.cb_btn_exp_export)
        self.cbx_control.currentIndexChanged.connect(self.cb_cbx_control)

        # self.widget_streaming.setStyleSheet("background-color: rgb(84, 84, 84);")
        # self.webview = QWebEngineView(self.widget_streaming)
        # self.webview.setUrl(QUrl("https://www.youtube.com/embed/t67_zAg5vvI?autoplay=1"))
        # self.webview.setGeometry(0, 0, 500, 300)

        self.thread_server = Thread(target=self.updateStatus)
        self.thread_server.start()

    
    def updateDeviceTable(self, device_status):
        for device_id in device_status:

            test_info = device_status[device_id]
            print(test_info)
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


    
    def updateStatus(self):
        '''
            1. Experiment status update 부분
            2. Device status update 부분
            3. Progree 관련 부분(progress bar 등)
        '''
        while True:
            try:
                # device_info = self.sql.select('device_info', conds="id=(SELECT MAX(id) FROM device_info)")[0]
                
                # for device_id in device_info:
                #     try:
                #         self.smartlab_cmd['setup_device'].index(device_id)
                #         device_status = device_info[device_id]
                #         # print("[DEBUG] {}: \n{}".format(device_id, device_status))

                #         self.updateDeviceTable(device_status)
                #     except:
                #         pass

                try:
                    fields = ['subject_name', 'Status', 'Thickness', 'Length', 'Width', 'E_modulus', 'U_stress']
                    header_id = self.smartlab_cmd['setup_doe']['header_id']
                    test_info = self.sql.select('result', fields=fields, conds='subject_name like "%{}%"'.format(header_id))
                    n_col = len(list(test_info[0]))
                    n_row = len(test_info)

                    self.table_exp_info.setRowCount(n_row)
                    self.table_exp_info.setColumnCount(n_col)

                    vbox = QVBoxLayout(self)
                    vbox.addWidget(self.table_exp_info)

                    self.table_exp_info.setHorizontalHeaderLabels((list(test_info[0].keys())))
                
                    for row, item_list in enumerate(test_info):
                        for col, key in enumerate(item_list):
                            item = list(test_info[row].values())[col]
                            newitem = QTableWidgetItem(str(item_list[key]))
                            self.table_exp_info.setItem(row, col, newitem)
                except:
                    print('bad1')
                
                if self.init_flag == True:
                    print(self.smartlab_cmd)
                    self.init_flag = False
                    response = self.smartlab.send(self.smartlab_cmd)
                    self.smartlab_cmd['test_step'] = -1 if self.smartlab_cmd['test_step'] == 1 else 1
                    print(response)
                    self.updateDeviceTable(response)
                    # try:
                    #     for device_id in response:
                    #         print(device_id)
                    # except:
                    #     print("DDDDDDDDDD")

            except:
                print("bad2")
            time.sleep(1.0)

    def cb_btn_exp_export(self):
        print("[DEBUG] 'Experiment - Export' button clicked !!! (TBD)")

    def cb_btn_control_run(self):
        print("[DEBUG] 'Control - Run' button clicked !!! (TBD)")
        self.init_flag = True
        self.smartlab_cmd['test_step'] = 1
    
    def cb_btn_control_pause(self):
        print("[DEBUG] 'Control - Pause' button clicked !!! (TBD)")

    def cb_btn_control_stop(self):
        print("[DEBUG] 'Control - Pause' button clicked !!! (TBD)")
        


    def cb_btn_doe_create(self):
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



    def cb_cbx_control(self):
        self.smartlab_cmd['test_mode'] = self.cbx_control.currentText()
        print("[DEBUG] Execution mode: {}".format(self.smartlab_cmd['test_mode']))


    def cbx_device_cb(self):
        try:
            device_id = self.cbx_device.currentText()
            if device_id.find('/') != -1:   device_id = device_id.split('/')[1]
            device_info = self.sql.select(tablename='device_info', fields=device_id)
            n_col = len(list(json.loads(device_info[0]).keys()))
            n_row = len(device_info)
            
            self.table_devcie_info.setRowCount(n_row)
            self.table_devcie_info.setColumnCount(n_col)
            self.table_devcie_info.setHorizontalHeaderLabels((list(json.loads(device_info[0]).keys())))
            for row, item_list in enumerate(device_info):
                for col, key in enumerate(json.loads(item_list)):
                    item = (list(json.loads(device_info[row]).values())[col])
                    newitem = QTableWidgetItem(str(item))
                    self.table_devcie_info.setItem(row, col, newitem)
        except:
            pass


    # def btn_run_cb(self):
    #     self.smartlab_cmd['setup_device'] = []
    #     if self.use_cobot.isChecked():    self.smartlab_cmd['setup_device'].append('R_001/cobot')
    #     if self.use_amr.isChecked():      self.smartlab_cmd['setup_device'].append('R_001/amr')
    #     if self.use_instron.isChecked():  self.smartlab_cmd['setup_device'].append('instron')
    #     if self.use_omm.isChecked():      self.smartlab_cmd['setup_device'].append('MS')
    #     if self.use_3dp_1.isChecked():    self.smartlab_cmd['setup_device'].append('printer1')
    #     if self.use_3dp_2.isChecked():    self.smartlab_cmd['setup_device'].append('printer2')
    #     if self.use_3dp_3.isChecked():    self.smartlab_cmd['setup_device'].append('printer3')
    #     if self.use_3dp_4.isChecked():    self.smartlab_cmd['setup_device'].append('printer4')

    #     self.cbx_device.clear()
    #     for device_id in self.smartlab_cmd['setup_device']:
    #         self.cbx_device.addItem(device_id)

        self.smartlab.send(self.smartlab_cmd)

        fields = ['subject_name', 'Status', 'Thickness', 'Length', 'Width', 'E_modulus', 'U_stress']
        header_id = self.smartlab_cmd['setup_doe']['header_id']
        test_info = self.sql.select('result', fields=fields, conds='subject_name like "%{}%"'.format(header_id))
        n_col = len(list(test_info[0]))
        n_row = len(test_info)

        self.table_test_info.setRowCount(n_row)
        self.table_test_info.setColumnCount(n_col)

        vbox = QVBoxLayout(self)
        vbox.addWidget(self.table_test_info)

        self.table_test_info.setHorizontalHeaderLabels((list(test_info[0].keys())))
    
        for row, item_list in enumerate(test_info):
            for col, key in enumerate(item_list):
                item = list(test_info[row].values())[col]
                newitem = QTableWidgetItem(str(item_list[key]))
                self.table_test_info.setItem(row, col, newitem)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SmartLAB_GUI()
    gui.show()
    app.exec_()
    print('good')