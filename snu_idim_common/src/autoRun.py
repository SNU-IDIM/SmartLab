#!/usr/bin/env python
# -*- coding: utf-8 -*-

import platform;   os_ = platform.system();   print('OS: {}'.format(os_))
import os, sys
import time
import cv2
import numpy as np
import pytesseract
import pyautogui
import re
# sys.path.append(r'C:\Users\IDIM-Instron\Desktop\SNU_SmartLAB\snu_idim_instron')
# from executeInstron import subject_name

if os_ == 'Windows':
    from win32api import GetSystemMetrics
    W_, H_ = GetSystemMetrics(0), GetSystemMetrics(1)
    print('Screen resolution: W{} x H{}'.format(W_, H_))
    pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
elif os_ == 'Linux':
    import subprocess
    output = subprocess.Popen('xrandr | grep "\*" | cut -d" " -f4', shell=True, stdout=subprocess.PIPE).communicate()[0]
    W_ , H_ = int(output.split('x')[0][:4]), int(output.split('x')[1][:4])
    print('Screen resolution: W{} x H{}'.format(W_, H_))
else:
    print('[ERROR] Screen resolution: W{} x H{}'.format(-1, -1))


class idimAutomation():

    def __init__(self, namespace):
        time.sleep(1.0)
        
        self.type = namespace
        if not os.path.exists(self.type):
            os.mkdir(self.type)
        

    def imageSearch(self, image_file, precision=0.8):
        try:
            img = pyautogui.screenshot()
            img_rgb = np.array(img)
            img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
            template = cv2.imread(image_file, 0)
            res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            if max_val < precision:
                return [-1, -1]
        except:
            print("[ERROR] imageSearch")
            return [-1,-1]
        return max_loc


    def wait_for_image(self, image_name, duration=5.0, precision=0.8):
        try:
            image_file = os.path.join(self.type, image_name + '.png')
            img = cv2.imread(image_file)
            height, width, channels = img.shape
            pos = self.imageSearch(image_file, precision=precision)
            t_start = time.time()
            while (time.time() - t_start < duration and pos == [-1, -1]):
                pos = self.imageSearch(image_file, precision=precision)
                print('[%.1f / %.1f] Waiting for image (%s) ' %(duration, time.time() - t_start, image_name))
            if pos == [-1, -1]:
                print('[INFO] wait_for_image - Image was not found!')
                return -1
        except:
            print('[ERROR] wait_for_image')
            return -1
        return 1
        
        
    def move_and_click(self, x, y, click=0):
        try:
            if click == 0:
                pyautogui.moveTo(x, y)
                print('[INFO] move_and_click - Image was clicked for {} time(s)'.format(click))
                return 1
            elif click == 1:
                pyautogui.click(x, y)
                print('[INFO] move_and_click - Image was clicked for {} time(s)'.format(click))
                return 1
            elif click == 2:
                pyautogui.doubleClick(x, y)
                print('[INFO] move_and_click - Image was clicked for {} time(s)'.format(click))
                return 1
            else:
                print('[ERROR] move_and_click[{}] - # of clicks should be 0 ~ 2'.format(click))
                return -1
            return 1
        except:
            print("[ERROR] move_and_click")
            return -1

    def move_to_text(self, text, click=0, precision=0.6):
        image = pyautogui.screenshot()
        image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        image = np.array(image)
        
        d = pytesseract.image_to_data(image, output_type=pytesseract.Output.DICT)#;   print(d['text'])
        n_boxes = len(d['text'])
        for i in range(n_boxes):
            if int(d['conf'][i]) > int(10 * precision) and d['text'][i] == text:
                (x, y, w, h) = (d['left'][i], d['top'][i], d['width'][i], d['height'][i])
                img = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.move_and_click(x+w/2, y+h/2, click=1)


    def move_to_icon(self, image_name, precision=0.8, click=0, sleep=0):
        try:
            image_file = os.path.join(self.type, image_name + '.png')
            img = cv2.imread(image_file)
            height, width, channels = img.shape
            pos = self.imageSearch(image_file, precision=precision)
            x = pos[0] + width / 2.0
            y = pos[1] + height / 2.0
            self.move_and_click(x, y, click=click)
            time.sleep(sleep)
            return 1
        except:
            print("[ERROR] move_to_icon")
            return -1
    
    
    def move_to_desktop(self):
        self.move_and_click(W_-2, H_-2, click=1)


    def open_program(self):
        self.move_to_icon("icon", click=1)
    

    def close_program(self):
        self.move_to_icon("exit", click=1)
        

    def get_data(self, data_number):
        image_file = os.path.join(self.type, "data_"+str(data_number)+'.png')
        # print(img_path)
        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        custom_config = "--psm 10 --oem 3 -c tessedit_char_whitelist=.0123456789"
        ocr_result = pytesseract.image_to_string(img, lang='eng', config=custom_config)
        # print(ocr_result)
        print(ocr_result[0:6])
        print(ocr_result[6:12])
        print(ocr_result[12:18])
        print(ocr_result[18:24])
    

    def changeTXT(self, cmd_file, name):
        with open(os.path.join(self.type, cmd_file), "r+") as J:
            data = J.read()
            data = re.sub('subject_name',name, data)
            J.seek(0)
            J.write(data)
            J.truncate()

    def returnTXT(self, cmd_file, name):
        with open(os.path.join(self.type, cmd_file), "r+") as J:
            data = J.read()
            data = re.sub(name,'subject_name', data)
            J.seek(0)
            J.write(data)
            J.truncate()


  
    def execute(self, cmd_file):
        with open(os.path.join(self.type, cmd_file), "r") as File:
        # File = open(os.path.join(self.type, cmd_file), "r")
        # print(File)

            while True:
                data = File.readline().split("\n")[0]
                if data == "":
                    print("[DEBUG] Command script file line end"); break
                    
                mode = data.split(',')[0]
                data = data.split(',')[1:]
                
                if mode == 'click':
                    n_click, image = int(data[0]), data[1]
                    try:
                        t_wait = float(data[2])
                    except:
                        t_wait = 10.0
                    self.wait_for_image(image, duration=t_wait)
                    print("Click icon")
                    self.move_to_icon(image, click=n_click, sleep=0.5)

                elif mode == 'wait':
                    duration = float(data[0])
                    print("Waiting for image({}) (for {} sec)".format(data[1], duration))
                    self.wait_for_image(data[1], duration=duration)

                elif mode == 'find':
                    text = data[0]
                    print("Click text: {}".format(text))
                    self.move_to_text(text, click=1)

                elif mode == 'text':
                    text = data[0]
                    print("Texting: {}".format(text))
                    pyautogui.write(text)
                    time.sleep(0.5)

                elif mode == 'program':
                    program = data[0]
                    print(program)
                    pyautogui.hotkey('winleft', program)
                    time.sleep(0.5)

                elif mode == 'tab':
                    tab = data[0]
                    pyautogui.hotkey('alt', tab)
                    time.sleep(0.5)
            
            
            # File.close()


if __name__ == "__main__":
    namespace = "src"

    test = idimAutomation('src')
    test.execute('{}.txt'.format('Instron'))
    # test.execute('start_experiment.txt')