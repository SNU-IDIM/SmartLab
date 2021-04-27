#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os
import ast
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtCore
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtCore import QUrl
import json

sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../snu_idim_common/src")) )
from SqlHelper import SqlHelper

from SmartLab_Client import SmartLabClient


class SmartLAB_GUI(QMainWindow, QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi("SmartLab_GUI.ui", self)
        QDialog().setFixedSize(self.size())

        self.sql = SqlHelper(host='localhost', username='root', password='0000', port=3306, database='SmartLab')
        self.smartlab = SmartLabClient(ip='192.168.60.21')

        self.smartlab_cmd = dict()
        self.smartlab_cmd['test_mode'] = 'auto'
        self.smartlab_cmd['test_step'] = 0
        self.smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS']
        self.smartlab_cmd['setup_doe'] = {
                                    'header_id': 'DRY_TEST',
                                    'experiment_type': 'Tensile Test',
                                    'factors': [ {'factor_name': 'infill_line_distance', 'factor_range': [0.4, 0.45]},
                                                {'factor_name': 'infill_angles'}
                                            ],
                                    'doe_type': 3, # DOE_GENERALIZED_FACTORIAL=3
                                    'option': [ [0.4, 0.45], 
                                                ['0', '45,135', '0,90', '90']
                                            ],
                                    }

        self.table_doe.cellChanged.connect(self.table_doe_cb)
        self.btn_run.clicked.connect(self.btn_run_cb)

        self.execution_mode.currentIndexChanged.connect(self.cbx_exe_mode_cb)
        self.cbx_device.currentIndexChanged.connect(self.cbx_device_cb)
        for device_id in self.smartlab_cmd['setup_device']:
            self.cbx_device.addItem(device_id)
        
        self.widget_streaming.setStyleSheet("background-color: rgb(84, 84, 84);")
        self.webview = QWebEngineView(self.widget_streaming)
        self.webview.setUrl(QUrl("https://www.youtube.com/embed/t67_zAg5vvI?autoplay=1"))
        self.webview.setGeometry(0, 0, 500, 300)

        model = QStandardItemModel()
        for i in ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']:
            model.appendRow(QStandardItem(i))
        self.listView.setModel(model)

        # test_info = [{'some': 'any 1',  'some2': 'any 2',  'some3': 'any 3',   'some4': 'any 4',   'some5': 'any 5'},
        #              {'some': 'any 1a', 'some2': 'any 2a', 'some3': 'any 3a',  'some4': 'any 4a',  'some5': 'any 5a'},
        #              {'some': 'any 1b', 'some2': 'any 2b', 'some3': 'any 3b',  'some4': 'any 4b',  'some5': 'any 5b'}
        #             ]   

        # test_info = self.sql.select('result')
        # n_col = len(list(test_info[0]))
        # n_row = len(test_info)

        # self.table_test_info.setRowCount(n_row)
        # self.table_test_info.setColumnCount(n_col)

        # vbox = QVBoxLayout(self)
        # vbox.addWidget(self.table_test_info)

        # self.table_test_info.setHorizontalHeaderLabels((list(test_info[0].keys())))
    
        # for row, item_list in enumerate(test_info):
        #     for col, key in enumerate(item_list):
        #         item = list(test_info[row].values())[col]
        #         newitem = QTableWidgetItem(str(item_list[key]))
        #         self.table_test_info.setItem(row, col, newitem)


    def table_doe_cb(self):
        for i in range(self.table_doe.rowCount()):
            key = self.table_doe.verticalHeaderItem(i).text()
            try: # List, Dict type
                x = ast.literal_eval(self.table_doe.item(i, 0).text())
            except: # String type
                x = self.table_doe.item(i, 0).text()
            self.smartlab_cmd['setup_doe'][key] = x
        print(self.smartlab_cmd['setup_doe'])

        # try: # List, Dict type
        #     x = ast.literal_eval(self.table_doe.currentItem().text())
        # except: # String type
        #     x = self.table_doe.currentItem().text()
        # print(type(x), x)


    def cbx_exe_mode_cb(self):
        self.smartlab_cmd['test_mode'] = self.execution_mode.currentText()
        print(self.smartlab_cmd['test_mode'])


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


    def btn_run_cb(self):
        self.smartlab_cmd['setup_device'] = []
        if self.use_cobot.isChecked():    self.smartlab_cmd['setup_device'].append('R_001/cobot')
        if self.use_amr.isChecked():      self.smartlab_cmd['setup_device'].append('R_001/amr')
        if self.use_instron.isChecked():  self.smartlab_cmd['setup_device'].append('instron')
        if self.use_omm.isChecked():      self.smartlab_cmd['setup_device'].append('MS')
        if self.use_3dp_1.isChecked():    self.smartlab_cmd['setup_device'].append('printer1')
        if self.use_3dp_2.isChecked():    self.smartlab_cmd['setup_device'].append('printer2')
        if self.use_3dp_3.isChecked():    self.smartlab_cmd['setup_device'].append('printer3')
        if self.use_3dp_4.isChecked():    self.smartlab_cmd['setup_device'].append('printer4')

        self.cbx_device.clear()
        for device_id in self.smartlab_cmd['setup_device']:
            self.cbx_device.addItem(device_id)

        print("Execution mode: {}".format(self.smartlab_cmd['test_mode']))
        print("Execution step: {}".format(self.smartlab_cmd['test_step']))
        print("Device list: {}".format(self.smartlab_cmd['setup_device']))
        print("DoE: \n{}".format(self.smartlab_cmd['setup_doe']))
        # QMessageBox.about(self, "message", "2")

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