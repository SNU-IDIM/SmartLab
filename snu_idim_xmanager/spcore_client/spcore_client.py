import sys, time
from PyQt5.QtWidgets import *
from PyQt5 import uic


class spcore_dialog(QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi('spcore_client.ui', self)

        # Handler Initialize <MODE>
        self.btn_mode_eco.clicked.connect(self.eco_mode_cb)
        self.btn_mode_drive.clicked.connect(self.drive_mode_cb)
        self.btn_mode_draw.clicked.connect(self.draw_mode_cb)
        self.btn_mode_reset.clicked.connect(self.reset_mode_cb)

        # Handler Initialize <CONTROL>
        self.btn_cntl_up.clicked.connect(self.cntl_up_cb)
        self.btn_cntl_dwn.clicked.connect(self.cntl_down_cb)
        self.btn_cntl_left.clicked.connect(self.cntl_left_cb)
        self.btn_cntl_right.clicked.connect(self.cntl_right_cb)
        self.btn_cntl_stop.clicked.connect(self.cntl_stop_cb)

        # Handler Initialize <MAP>
        self.btn_map_save.clicked.connect(self.map_save_cb)
        self.btn_map_load.clicked.connect(self.map_load_cb)
        self.btn_map_delete.clicked.connect(self.map_delete_cb)

        # Handler Initialize <LOG>
        self.btn_log_clear.clicked.connect(self.log_clear_cb)

        # Handler Initialize <GOAL>
        self.btn_goal_set.clicked.connect(self.goal_set_cb)


    # Handler Definition <MODE>
    def eco_mode_cb(self):
        self.log_browser.append("eco")
    def drive_mode_cb(self):
        self.log_browser.append("drive")
    def draw_mode_cb(self):
        self.log_browser.append("draw")
    def reset_mode_cb(self):
        self.log_browser.append("reset")

    # Handler Definition <MODE>
    def cntl_up_cb(self):
        self.log_browser.append("cntl_up")
    def cntl_down_cb(self):
        self.log_browser.append("cntl_down")
    def cntl_left_cb(self):
        self.log_browser.append("cntl_left")
    def cntl_right_cb(self):
        self.log_browser.append("cntl_right")
    def cntl_stop_cb(self):
        self.log_browser.append("cntl_stop")

    # Handler Definition <MAP>
    def map_save_cb(self):
        self.map_list.addItem(QListWidgetItem("map%d"%self.map_list.__len__()))
        self.log_browser.append("map_save")
    def map_load_cb(self):
        self.log_browser.append("map_load")
    def map_delete_cb(self):
        cur_item = self.map_list.currentRow()
        self.map_list.takeItem(cur_item)
        self.log_browser.append("map_delete")

    # Handler Definition <LOG>
    def log_clear_cb(self):
        self.log_browser.clear()

    # Handler Definition <GOAL>
    def goal_set_cb(self):
        self.log_browser.append("%f, %f, %f"%(self.box_goal_x.value(), self.box_goal_y.value(), self.box_goal_th.value()))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    spcore_dialog = spcore_dialog()
    spcore_dialog.show()

    app.exec_()