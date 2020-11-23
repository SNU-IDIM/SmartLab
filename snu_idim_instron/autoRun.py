import cv2
import os
from PIL import ImageGrab
import numpy as np
import pytesseract
import pyautogui
import time
import pynput
from pynput import keyboard
from win32api import GetSystemMetrics
pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'



class idim_smart_lab():
    def __init__(self, machine_type):
        self.type = machine_type
        self.path = os.path.abspath(os.getcwd()) + "/" + self.type + "/"
        self.region_iconbar = (0, 1160, 1920, 40)
        self.region_capture = (0, 0, 0, 0)
        self.region_data    = (0, 0, 0, 0)
        self.flag_temp_pos = 1
        self.temp_pos1 = (0, 0)
        self.temp_pos2 = (0, 0)
        self.lefttop = (0, 0)
        self.rightbottom = pyautogui.size()
        self.flag_get_region = False   
        try:
            os.makedirs(self.type)
        except:
            pass
        

    def imageSearch(self, image_file, precision=0.8):
        img = pyautogui.screenshot()
        
        img_rgb = np.array(img)
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        template = cv2.imread(image_file, 0)
        
        if template != []:
            template.shape[::-1]
            res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            if max_val < precision:
                return [-1, -1]
        else:
            print("Failed to read image.")
            return [-1,-1]
        return max_loc

    def wait_for_image(self, image_name, duration=5.0):
        try:
            image_file = self.path + image_name + '.png'
            img = cv2.imread(image_file)
            height, width, channels = img.shape
            pos = self.imageSearch(image_file, precision=0.6)
            t_start = time.time()
            while (time.time() - t_start < duration and pos == [-1, -1]):
                pos = self.imageSearch(image_file, precision=0.6)
                print('Waiting for image ({})'.format(image_name))
        except:
            pass
        
        
    def move_and_click(self, x, y, click=0):
        try:
            if click == 0:
                pyautogui.moveTo(x, y)
                return 1
            elif click == 1:
                pyautogui.click(x, y)
                return 1
            elif click == 2:
                pyautogui.doubleClick(x, y)
                return 1
            else:
                return -1
            return 1
        except:
            print("ERROR: Icon not clicked !!!")
            return -1
        
    def move_to_text(self, text, click=0):
        image = pyautogui.screenshot()
        image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        image = np.array(image)
        
        d = pytesseract.image_to_data(image, output_type=pytesseract.Output.DICT)
        n_boxes = len(d['text'])
        # print(d['text'])
        for i in range(n_boxes):
            if int(d['conf'][i]) > 60 and d['text'][i] == text:
                (x, y, w, h) = (d['left'][i], d['top'][i], d['width'][i], d['height'][i])
                img = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.move_and_click(x+w/2, y+h/2, click=2)
                    
        # cv2.imshow('image', image)
        # cv2.waitKey(0)


    def move_to_icon(self, image_name, click=0, sleep=0):
        try:
            image_file = self.path + image_name + '.png'
            img = cv2.imread(image_file)
            height, width, channels = img.shape
            pos = self.imageSearch(image_file, precision=0.6)
            x = pos[0] + width / 2.0
            y = pos[1] + height / 2.0
#             x, y = pyautogui.locateCenterOnScreen(self.path + image_name + '.png')
            self.move_and_click(x, y, click=click)
            time.sleep(sleep)
            return 1
        except:
            print("ERROR: Icon not clicked !!!")
            return -1
    
    
    def move_to_desktop(self):
        self.move_and_click(W_-2, H_-2, click=1)
    
    def open_program(self):
        self.move_to_icon("icon", click=1)
        pass
    
    def close_program(self):
        self.move_to_icon("exit", click=1)
        pass
        
    def get_data(self, data_number):
        img_path = self.path + self.type + "_data_" + str(data_number) + ".png" 
        # print(img_path)
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        custom_config = "--psm 10 --oem 3 -c tessedit_char_whitelist=.0123456789"
        ocr_result = pytesseract.image_to_string(img, lang='eng', config=custom_config)
        # print(ocr_result)
        print(ocr_result[0:6])
        print(ocr_result[6:12])
        print(ocr_result[12:18])
        print(ocr_result[18:24])
        
        
    def execute(self, cmd_file):
        File = open(self.type + "/" + cmd_file, "r")
        while True:
            data = File.readline().split("\n")[0]
            if data == "":
                print("break"); break
                
            mode = data.split(',')[0]
            data = data.split(',')[1]
            
            if mode[0] == 'C':
                n_click = int(mode[1])
                self.wait_for_image(data, duration=10.0)
                print("Click icon")
                self.move_to_icon(data, click=n_click, sleep=0.5)
            elif mode[0] == 'W':
                duration = float(mode[1:])
                print("Waiting for image({}) (for {} sec)".format(data, duration))
                self.wait_for_image(data, duration=duration)
            elif mode == 'F':
                print("Click text: {}".format(data))
                self.move_to_text(data, click=1)
            elif mode == 'T':
                print("Texting")
                pyautogui.write(data)
            elif mode == 'P':
                print(data)
                pyautogui.hotkey('win', data)
                time.sleep(0.5)
            elif mode == '3DP':
                pyautogui.hotkey('alt', data)
                time.sleep(0.5)
                
            
        File.close()
        pass


if __name__ == "__main__":
    namespace = "testing"

    instron = idim_smart_lab(namespace)
    instron.execute('testing.txt')
