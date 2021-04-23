import sys
import ast
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *


class SmartLAB_GUI(QMainWindow, QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi("test.ui", self)
        QDialog().setFixedSize(self.size())

        smartlab_cmd = dict()
        smartlab_cmd['test_mode'] = 'auto'
        smartlab_cmd['test_step'] = 0
        smartlab_cmd['setup_device'] = ['R_001/amr', 'R_001/cobot', 'instron', 'MS', 'printer1']
        smartlab_cmd['setup_doe'] = {
                                    'header_id': 'yun_210422',
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
        self.test1.clicked.connect(self.btn1_clicked)
        self.test2.clicked.connect(self.btn2_clicked)
        self.test3.clicked.connect(self.btn3_clicked)

        # self.combo = QComboBox(self)
        self.comboBox.currentIndexChanged.connect(self.combobox_select)
        self.combo_device.currentIndexChanged.connect(self.combo_device_cb)
        
        model = QStandardItemModel()
        for i in ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']:
            model.appendRow(QStandardItem(i))
        self.listView.setModel(model)


        self.table_doe.cellChanged.connect(self.table_doe_cb)

        # spisok = [{'some': 'any 1',  'some2': 'any 2',  'some3': 'any 3',  'some4': 'any 2',  'some5': 'any 3'},
        #           {'some': 'any 1a', 'some2': 'any 2a', 'some3': 'any 3a',  'some4': 'any 2',  'some5': 'any 3'},
        #           {'some': 'any 1b', 'some2': 'any 2b', 'some3': 'any 3b',  'some4': 'any 2',  'some5': 'any 3'}
        #          ]   

        # self.table.setRowCount(3)
        # self.table.setColumnCount(5)

        # vbox = QVBoxLayout(self)
        # vbox.addWidget(self.table)

        # self.table.setHorizontalHeaderLabels((list(spisok[0].keys())))

        # for row, item_list in enumerate(spisok):
        #     for col, key in enumerate(item_list):
        #         item = (list(spisok[row].values())[col])
        #         print(item)
        #         newitem = QTableWidgetItem(item_list[key])
        #         self.table.setItem(row, col, newitem)


    def table_doe_cb(self):
        # print(self.table_doe.columnCount())
        # print(self.table_doe.rowCount())

        for i in range(self.table_doe.rowCount()):
            try: # List, Dict type
                x = ast.literal_eval(self.table_doe.item(i, 0).text())
            except: # String type
                x = self.table_doe.item(i, 0).text()
            print("{}: {} (type: {})".format(self.table_doe.verticalHeaderItem(i).text(), x, type(x)))
            
            

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


    def combobox_select(self):
        # QLabel 에 표시
        print(self.comboBox.currentText())
        print(self.comboBox.currentIndex())

    def combo_device_cb(self):
        device_type = self.combo_device.currentText()
        spisok = [{'some': device_type,  'some2': 'any 2',  'some3': 'any 3',  'some4': 'any 2',  'some5': 'any 3'},
                  {'some': device_type, 'some2': 'any 2a', 'some3': 'any 3a',  'some4': 'any 2',  'some5': 'any 3'},
                  {'some': device_type, 'some2': 'any 2b', 'some3': 'any 3b',  'some4': 'any 2',  'some5': 'any 3'}
                 ]   

        self.table.setRowCount(3)
        self.table.setColumnCount(5)

        vbox = QVBoxLayout(self)
        vbox.addWidget(self.table)

        self.table.setHorizontalHeaderLabels((list(spisok[0].keys())))

        for row, item_list in enumerate(spisok):
            for col, key in enumerate(item_list):
                item = (list(spisok[row].values())[col])
                print(item)
                newitem = QTableWidgetItem(item_list[key])
                self.table.setItem(row, col, newitem)


    def btn1_clicked(self):
        QMessageBox.about(self, "message", "1")

    def btn2_clicked(self):
        QMessageBox.about(self, "message", "2")

    def btn3_clicked(self):
        QMessageBox.about(self, "message", "3")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SmartLAB_GUI()
    gui.show()
    app.exec_()