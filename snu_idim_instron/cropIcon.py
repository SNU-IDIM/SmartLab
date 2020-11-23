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





if __name__ == "__main__":
    namespace = "testing"
    ci = cropIcon(namespace)
    ci.loop()
