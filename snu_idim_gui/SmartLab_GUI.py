import sys
import ast
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtCore
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtCore import QUrl, QThread, pyqtSignal, pyqtSlot, Qt
from SqlHelper import SqlHelper
import json
import ast
import cv2


class Thread(QThread):
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


class SmartLAB_GUI(QMainWindow, QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi("SmartLab_GUI.ui", self)
        QDialog().setFixedSize(self.size())

        self.sql = SqlHelper(host='localhost', username='root', password='0000', port=3306, database='SmartLab')

        self.smartlab_cmd = dict()
        self.smartlab_cmd['test_mode'] = 'auto'
        self.smartlab_cmd['test_step'] = 0
        self.smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1', 'printer2', 'printer3', 'printer4']
        self.smartlab_cmd['setup_doe'] = dict()

        # self.btn_run.clicked.connect(self.btn_run_cb)
        self.btn_doe_create.clicked.connect(self.cb_btn_doe_create)
        self.cbx_control.currentIndexChanged.connect(self.cb_cbx_control)

        # create a label
        # self.label = QLabel(self)
        # self.label.move(280, 120)
        # self.label.resize(640, 480)

        th = Thread(self)
        th.changePixmap.connect(self.setImage)
        th.start()

        # self.widget_streaming.setStyleSheet("background-color: rgb(84, 84, 84);")
        # self.webview = QWebEngineView(self.widget_streaming)
        # self.webview.setUrl(QUrl("https://www.youtube.com/embed/t67_zAg5vvI?autoplay=1"))
        # self.webview.setGeometry(0, 0, 500, 300)

    @pyqtSlot(QImage)
    def setImage(self, image):
        self.label.setPixmap(QPixmap.fromImage(image))
    
    def updateStatus(self):
        '''
            1. Experiment status update 부분
            2. Device status update 부분
            3. Progree 관련 부분(progress bar 등)
        '''
        # test_info = self.sql.select('device_info')
        # n_col = len(list(test_info[0]))
        # n_row = len(test_info)

        # self.table_exp_info.setRowCount(n_row)
        # self.table_exp_info.setColumnCount(n_col)

        # vbox = QVBoxLayout(self)
        # vbox.addWidget(self.table_exp_info)

        # self.table_exp_info.setHorizontalHeaderLabels((list(test_info[0].keys())))

        # for row, item_list in enumerate(test_info):
        #     for col, key in enumerate(item_list):
        #         item = list(test_info[row].values())[col]
        #         print(item)
        #         newitem = QTableWidgetItem(str(item_list[key]))
        #         self.table_exp_info.setItem(row, col, newitem)
        pass


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

        self.smartlab_cmd['header_id']       = self.txt_doe_exp_name.text()
        self.smartlab_cmd['experiment_type'] = self.txt_doe_exp_type.text()
        self.smartlab_cmd['doe_type']        = self.txt_doe_doe_type.text()
        self.smartlab_cmd['factors']         = doe_factors
        self.smartlab_cmd['option']          = doe_options
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



if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SmartLAB_GUI()
    gui.show()
    app.exec_()