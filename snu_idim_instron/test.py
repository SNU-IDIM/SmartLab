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

class cropIcon(): 
    def __init__(self, namespace: str):
        self.ns = namespace
        try:
            os.makedirs(self.ns)
        except FileExistsError:
            pass
        self.logFile = open(self.ns + "\\" + self.ns + ".txt", "w")
        
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.keyFlag     = False
        self.cropFlag    = False
        self.findtxtFlag = False
        self.textingFlag = False
        self.exitFlag    = False
        self.printFlag   = True
        
        self.image = pyautogui.screenshot()
        self.image = cv2.cvtColor(np.array(self.image), cv2.COLOR_RGB2BGR)
        self.image = np.array(self.image)
        self.image_original = self.image.copy()
        self.image_cropped  = self.image.copy()
        
        self.icon_name       = ""
        self.icon_click      = -1
        self.icon_texting    = ""
        self.cvCropdoneFlag  = False
        self.cvCrop_cropping = False
        self.cvCrop_x_start  = 0
        self.cvCrop_y_start  = 0
        self.cvCrop_x_end    = 0
        self.cvCrop_y_end    = 0


    def capture(self):
        self.cropFlag = True
        padding = 100
        
        center =  pyautogui.position()
        left   =  max(center[0] - padding, 0) #max(center[0]-100,  0)
        right  =  min(center[0] + padding, GetSystemMetrics(0)) #min(center[0]+100,  GetSystemMetrics(0))
        bottom =  max(center[1] - padding, 0) #min(center[1]-100,  GetSystemMetrics(1))
        top    =  min(center[1] + padding, GetSystemMetrics(1)) #max(center[1]+100,  0)
        
        pyautogui.moveTo(GetSystemMetrics(0)-2, GetSystemMetrics(1)-2)
        time.sleep(0.5)
        self.image = pyautogui.screenshot()
        self.image = cv2.cvtColor(np.array(self.image), cv2.COLOR_RGB2BGR)
        self.image = np.array(self.image)
        self.image = self.image[bottom:top, left:right]
    
    
    def on_press(self, key):
        try:
            # print('Keyboard: alphanumeric key {0} pressed'.format(key.char))
            a = 1
            if key == keyboard.KeyCode(char='/'):
                self.keyFlag = True # ; print("'/' is pressed!")
            if self.keyFlag == True:
                if key == keyboard.KeyCode(char='0'):
                    self.cropFlag = True
                    self.icon_click = 0
                elif key == keyboard.KeyCode(char='1'):
                    self.cropFlag = True
                    self.icon_click = 1
                elif key == keyboard.KeyCode(char='2'):
                    self.cropFlag = True
                    self.icon_click = 2
                elif key == keyboard.KeyCode(char='f'):
                    self.findtxtFlag = True
                elif key == keyboard.KeyCode(char='t'):
                    self.textingFlag = True
            pass
        except AttributeError:
            # print('Keyboard: special key {0} pressed'.format(key))
            pass

        
    def on_release(self, key):
        if key == keyboard.KeyCode(char='/'):
            self.keyFlag = False
        if key == keyboard.Key.esc:
            print("ESC")
            self.exitFlag = True
            return False
        
        
    def cvCallback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.cvCrop_x_start, self.cvCrop_y_start = x, y
            self.cvCrop_x_end, self.cvCrop_y_end = x, y
            self.cvCrop_cropping = True

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.cvCrop_cropping == True:
                self.cvCrop_x_end, self.cvCrop_y_end = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            self.cvCrop_x_end, self.cvCrop_y_end = x, y
            self.cvCrop_cropping = False

            refPoint = [(self.cvCrop_x_start, self.cvCrop_y_start), (self.cvCrop_x_end, self.cvCrop_y_end)]
            if len(refPoint) == 2: # when two points were found
                self.image_cropped = self.image[refPoint[0][1]:refPoint[1][1], refPoint[0][0]:refPoint[1][0]]
                cv2.imshow("Cropped", self.image_cropped)
                self.cvCropdoneFlag = True

                
    def loop(self):
        while not self.exitFlag:
            self.printManual()
            if self.cropFlag == True:
                print("Cropping...")
                print("Click: {} time(s)".format(self.icon_click))
                if self.icon_click == 0:
                    self.execute()
                elif self.icon_click == 1:
                    self.execute()
                elif self.icon_click == 2:
                    self.execute()
                self.initFlag()
            
            elif self.findtxtFlag == True:
                self.icon_click = 3
                self.icon_texting = input("Please enter text to find: ")
                self.logFile.write("3" + "," + self.icon_texting + "\n")
                self.initFlag()
            
            elif self.textingFlag == True:
                self.icon_click   = 1
                print("Click: {} time(s)".format(self.icon_click))
                self.execute()
                
                self.icon_texting = input("Please enter text to write: ")
                self.logFile.write("-1" + "," + self.icon_texting + "\n")
                self.initFlag()
                
        self.logFile.close()

        
    def initFlag(self):
        self.findtxtFlag    = False
        self.textingFlag    = False
        self.cvCropdoneFlag = False  
        self.cropFlag       = False
        self.printFlag      = False
        self.icon_click     = -1
        
        
    def printManual(self):
        if self.printFlag == True:
            print("---------------------------------------------------------------------------")
            print("< Record Mode >" + "\n")
            
            print("Mode 1: Recording icon for click action")
            print("  Step 1-1: Press '/' + 'x' on the icon on screen (x = 0 ~ 2; # of clicks)")
            print("  Step 1-2: Crop the icon in the pop-up window")
            print("  Step 1-3: Name the icon that you have cropped" + "\n")
            
            print("Mode 2: Find text on the screen and click (english & number ONLY)")
            print("  Step 2-1: Press '/' + 'f'")
            print("  Step 2-2: Type the text you want to find" + "\n")
            
            print("Mode 3: Type some text on the screen")
            print("  Step 3-1: Press '/' + 't'")
            print("  Step 3-2: Crop the typing area in the pop-up window")
            print("  Step 3-3: Name the typing area that you have cropped")
            print("  Step 3-4: Type the text you want to write" + "\n")
            
            print("If all the sequence is recorded or to escape the Record Mode, press 'ESC'")
            print("---------------------------------------------------------------------------")
        self.printFlag = False


    def execute(self):
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.cvCallback)
        
        self.capture()
        
        while not self.cvCropdoneFlag:
            i = self.image.copy()
            if not self.cvCrop_cropping:
                cv2.imshow("image", self.image)
            elif self.cvCrop_cropping:
                cv2.rectangle(i, (self.cvCrop_x_start, self.cvCrop_y_start), (self.cvCrop_x_end, self.cvCrop_y_end), (255, 0, 0), 2)
                cv2.imshow("image", i)
            cv2.waitKey(1)
        cv2.destroyAllWindows() # close all open windows

        self.icon_name = input("Please name the icon: ")
        cv2.imwrite(self.ns + "\\" + self.icon_name + ".png", self.image_cropped)
        self.logFile.write(str(self.icon_click) + "," + self.icon_name + "\n")




class idim_smart_lab():
    def __init__(self, machine_type: str):
        self.type = machine_type
        self.path = os.path.abspath(os.getcwd()) + "\\" + self.type + "\\"
        self.region_iconbar = (0, 1160, 1920, 40)
        self.region_capture = (0, 0, 0, 0)
        self.region_data    = (0, 0, 0, 0)
        self.flag_temp_pos = 1
        self.temp_pos1 = (0, 0)
        self.temp_pos2 = (0, 0)
        self.lefttop = (0, 0)
        self.rightbottom = pyautogui.size()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.flag_get_region = False   
        try:
            os.makedirs(self.type)
        except FileExistsError:
            pass
        
    def initialize(self):
        for i in range(100):
            print("Capturing Button[{}]".format(i))
            self.capture(self.type + "_button_{}".format(str(i))) # Button 1
            
        # self.capture(self.type) # icon
        # self.capture(self.type + "_data_1")   # Data region 1
        # self.capture(self.type + "_data_2")   # Data region 2
        # self.capture(self.type + "_data_3")   # Data region 3
        # self.capture(self.type + "_button_1") # Button 1
        # self.capture(self.type + "_button_2") # Button 2
        # self.capture(self.type + "_button_3") # Button 3
        # self.capture(self.type + "_button_4") # Button 4
        # self.capture(self.type + "_button_5") # Button 4
        
        

    def on_press(self, key):
        try:
            # print('Keyboard: alphanumeric key {0} pressed'.format(key.char))
            pass
        except AttributeError:
            # print('Keyboard: special key {0} pressed'.format(key))
            pass

    def on_release(self, key):
        # print('Keyboard: {0} released'.format(key))
        if key == keyboard.KeyCode(char='/'):
            if self.flag_temp_pos == 1:
                self.temp_pos1 = pyautogui.position()
                self.flag_temp_pos = 2
                print("Press '/' Button on Bottom-right")
            elif self.flag_temp_pos == 2:
                self.temp_pos2 = pyautogui.position()            
                self.flag_temp_pos = 3
        if key == keyboard.Key.esc:
            return False
    
    def capture(self, region_type: str):
        print("Capture the screen region: " + region_type)
        print("Press '/' Button on Top-left")
        while(1):
            if (self.flag_temp_pos == 3):
                break
        left   = min(self.temp_pos1[0],  self.temp_pos2[0])
        right  = max(self.temp_pos1[0],  self.temp_pos2[0])
        top    = max(self.temp_pos1[1],  self.temp_pos2[1])
        bottom = min(self.temp_pos1[1],  self.temp_pos2[1])
        width  = abs(self.temp_pos1[0] - self.temp_pos2[0])
        height = abs(self.temp_pos1[1] - self.temp_pos2[1])
        self.region_capture = (left, top, width, height)
        if region_type == "data":
            self.region_data = (left, top, width, height)
        image = pyautogui.screenshot()
        image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        image = np.array(image)
        cropped = image[bottom:top, left:right]
        cv2.imwrite(self.type + "/" + region_type + ".png", cropped)
        print("Captured completed: " + region_type + str(self.region_capture))
        self.temp_pos1 = (0, 0)
        self.temp_pos2 = (0, 0)
        self.flag_temp_pos = 1
        
        
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
        
    def move_to_text(self, text: str, click=0):
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


    def move_to_icon(self, image_name: str, click=0, sleep=0):
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
        self.move_and_click(GetSystemMetrics(0)-2, GetSystemMetrics(1)-2, click=1)
    
    def open_program(self):
        self.move_to_icon("icon", click=1)
        pass
    
    def close_program(self):
        self.move_to_icon("exit", click=1)
        pass
        
    def get_data(self, data_number: int):
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
        
        
    def execute(self):
        File = open(self.type + "\\" + self.type + ".txt", "r")
        while True:
            data = File.readline().split("\n")[0]
            if data == "":
                print("break"); break
                
            mode = int(data.split(',')[0])
            data = data.split(',')[1]
            
            if mode == 0 or mode == 1 or mode == 2:
                print("Click icon")
                self.move_to_icon(data, click=mode, sleep=1)
            if mode == 3:
                print("Click text: {}".format(data))
                self.move_to_text(data, click=1)
            elif mode == -1:
                print("Texting")
                pyautogui.write(data)
            
        File.close()
        pass
        



if __name__ == "__main__":
    namespace = "testing"
    ci = cropIcon(namespace)
    ci.loop()

    # instron = idim_smart_lab(namespace)
    # instron.execute()