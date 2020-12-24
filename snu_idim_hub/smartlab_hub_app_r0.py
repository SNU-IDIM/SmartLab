import sys

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import Qt

# https://freeprog.tistory.com/333

class smart_lab_hub_app(QtWidgets.QMainWindow):
    def __init__(self):
        super(smart_lab_hub_app, self).__init__()        
        uic.loadUi('smartlab_hub_app_r0.ui', self)

        self.set_table_layout()

        self.btn_refresh_device.clicked.connect(self.refresh_device_list)

        self.actionExit.triggered.connect(self.exit)

        self.show()
    
    def exit(self):
        self.close()

    def set_table_layout(self):
        # self.tbl_device_list.setSelectionMode()
        
        self.tbl_device_list.setColumnCount(6)

        self.tbl_device_list.setHorizontalHeaderLabels(['No.', 'Name', 'Type', 'Connection (IP address)', 'Status', 'More Info'])

        # source: https://kwonkyo.tistory.com/370
        header = self.tbl_device_list.horizontalHeader()
        table_width = header.width()
        width = []
        for column in range(header.count()):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeToContents)
            width.append(header.sectionSize(column))
        
        wfactor = table_width / sum(width)

        for column in range(header.count()):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.Interactive)
            header.resizeSection(column, width[column] * wfactor)

        self.tbl_device_list.verticalHeader().setVisible(False)

    def refresh_device_list(self):
        self.set_table_layout()

        self.tbl_device_list.setRowCount(3)

        device_info_ex = ['0', '3DP-1', '3D Printer', 'Offline(0.0.0.0:5001)', 'n/a']

        for index in range(len(device_info_ex)):
            item = QtWidgets.QTableWidgetItem(device_info_ex[index])
            item.setTextAlignment(Qt.AlignCenter)
            self.tbl_device_list.setItem(0, index, item)
        
        self.tbl_device_list.setCellWidget(0, index + 1, QtWidgets.QPushButton('...'))

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = smart_lab_hub_app()
    app.exec_()