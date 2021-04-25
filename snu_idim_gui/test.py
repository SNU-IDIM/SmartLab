import sys
import ast
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtCore
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtCore import QUrl
from SqlHelper import SqlHelper
import json


class SmartLAB_GUI(QMainWindow, QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi("test.ui", self)
        QDialog().setFixedSize(self.size())

        self.sql = SqlHelper(host='localhost', username='root', password='0000', port=3306, database='SmartLab')

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
                                    

        # self.setupUi(self)
        self.btn_run.clicked.connect(self.btn_run_cb)

        # self.combo = QComboBox(self)
        self.execution_mode.currentIndexChanged.connect(self.execution_mode_cb)
        self.combo_device.currentIndexChanged.connect(self.combo_device_cb)
        for device_id in self.smartlab_cmd['setup_device']:
            self.combo_device.addItem(device_id)
        
        model = QStandardItemModel()
        for i in ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']:
            model.appendRow(QStandardItem(i))
        self.listView.setModel(model)


        self.table_doe.cellChanged.connect(self.table_doe_cb)


        # self.widget_streaming.setGeometry(QtCore.QRect(2, 0, 500, 300))
        self.widget_streaming.setStyleSheet("background-color: rgb(84, 84, 84);")
        self.webview = QWebEngineView(self.widget_streaming)
        self.webview.setUrl(QUrl("https://www.youtube.com/embed/t67_zAg5vvI?autoplay=1"))
        self.webview.setGeometry(0,0,500,300)


        # spisok = [{'some': 'any 1',  'some2': 'any 2',  'some3': 'any 3',  'some4': 'any 2',  'some5': 'any 3'},
        #           {'some': 'any 1a', 'some2': 'any 2a', 'some3': 'any 3a',  'some4': 'any 2',  'some5': 'any 3'},
        #           {'some': 'any 1b', 'some2': 'any 2b', 'some3': 'any 3b',  'some4': 'any 2',  'some5': 'any 3'}
        #          ]   

        # spisok = self.sql.select('device_info')

        # self.table.setRowCount(3)
        # self.table.setColumnCount(5)

        # vbox = QVBoxLayout(self)
        # vbox.addWidget(self.table)

        # self.table.setHorizontalHeaderLabels((list(spisok[0].keys())))

        # for row, item_list in enumerate(spisok):
        #     for col, key in enumerate(item_list):
        #         item = list(spisok[row].values())[col]
        #         print(item)
        #         newitem = QTableWidgetItem(str(item_list[key]))
        #         self.table.setItem(row, col, newitem)


    def table_doe_cb(self):
        # print(self.table_doe.columnCount())
        # print(self.table_doe.rowCount())

        for i in range(self.table_doe.rowCount()):
            key = self.table_doe.verticalHeaderItem(i).text()
            try: # List, Dict type
                x = ast.literal_eval(self.table_doe.item(i, 0).text())
            except: # String type
                x = self.table_doe.item(i, 0).text()
            
            self.smartlab_cmd['setup_doe'][key] = x
            # print("{}: {} (type: {})".format(self.table_doe.verticalHeaderItem(i).text(), x, type(x)))
        print(self.smartlab_cmd['setup_doe'])
            

        # print(self.table_doe.verticalHeaderItem(1).text())

        # columns = set(cell.column() for cell in self.table_doe.selectedIndexes())
        # labels = [self.table_doe.horizontalHeaderItem(c).text() for c in columns]
        # print(labels)


        # print(self.table_doe.currentColumn())
        # print(self.table_doe.item(1, self.table_doe.currentColumn()).text())
        try: # List, Dict type
            x = ast.literal_eval(self.table_doe.currentItem().text())
        except: # String type
            x = self.table_doe.currentItem().text()
        
        # print(type(x), x)


    def execution_mode_cb(self):
        # QLabel 에 표시
        self.smartlab_cmd['test_mode'] = self.execution_mode.currentText()
        # print(self.comboBox.currentText(), self.comboBox.currentIndex())
        print(self.smartlab_cmd['test_mode'])


    def combo_device_cb(self):
        try:
            device_id = self.combo_device.currentText()
            if device_id.find('/') != -1:   device_id = device_id.split('/')[1]
            spisok = self.sql.select(tablename='device_info', fields=device_id)
            n_col = len(list(json.loads(spisok[0]).keys()))
            n_row = len(spisok)
            
            self.table.setRowCount(n_row)
            self.table.setColumnCount(n_col)
            self.table.setHorizontalHeaderLabels((list(json.loads(spisok[0]).keys())))
            for row, item_list in enumerate(spisok):
                for col, key in enumerate(json.loads(item_list)):
                    item = (list(json.loads(spisok[row]).values())[col])
                    newitem = QTableWidgetItem(str(item))
                    self.table.setItem(row, col, newitem)
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

        self.combo_device.clear()
        for device_id in self.smartlab_cmd['setup_device']:
            self.combo_device.addItem(device_id)


        print("Execution mode: {}".format(self.smartlab_cmd['test_mode']))
        print("Execution step: {}".format(self.smartlab_cmd['test_step']))
        print("Device list: {}".format(self.smartlab_cmd['setup_device']))
        print("DoE: \n{}".format(self.smartlab_cmd['setup_doe']))
        # QMessageBox.about(self, "message", "2")



if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SmartLAB_GUI()
    gui.show()
    app.exec_()