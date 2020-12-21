#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
from time import sleep
import json
from threading import Thread

from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.support.ui import Select, WebDriverWait
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.common.by import By


'''
`   <Codes for Selenium...>

    find_element(By.XPATH, '//button[text()="Some text"]')
    find_element(By.XPATH, '//button')
    find_element(By.ID, 'loginForm')
    find_element(By.LINK_TEXT, 'Continue')
    find_element(By.PARTIAL_LINK_TEXT, 'Conti')
    find_element(By.NAME, 'username')
    find_element(By.TAG_NAME, 'h1')
    find_element(By.CLASS_NAME, 'content')
    find_element(By.CSS_SELECTOR, 'p.content')
    find_elements(By.ID, 'loginForm')
    find_elements(By.CLASS_NAME, 'content')
'''


class DeviceClass_3DP:
    def __init__(self, device_name='printer0'):
        ## Common init for all devices
        self.device_id = int(device_name.split('printer')[1])
        self.status = dict()
        self.status['device_type'] = '3D Printer'
        self.status['device_name'] = device_name
        self.status['connection'] = ''
        self.status['subject_name'] = ''
        self.status['status'] = ''
        self.status['recent_work'] = ''

        self.status['gcode_file'] = ''
        
        ## Specialized init for the device (in this case, 3D printer)
        executable_path = os.path.join('../snu_idim_3dp', 'chromedriver_81.0.4044.92')
        print(executable_path)
        self.driver = webdriver.Chrome(executable_path=executable_path)
        self.driver.get('http://0.0.0.0:500{}/?#temp'.format(self.device_id))
        try:
            self.driver.find_element(By.ID, 'login-user').send_keys('wjyun')
            self.driver.find_element(By.ID, 'login-password').send_keys('idimahn1')
            self.driver.find_element(By.ID, 'login-button').click()
        except:
            pass

        thread_1 = Thread(target=self.updateStatus)
        thread_1.start()
        # thread_1.join()


    def __del__(self):
        ## Specialized del for the device (in this case, 3D printer)
        self.driver.close()


    def updateStatus(self):
        while True:
            self.waitUntilLoaded(By.ID, 'state')
            device_status_table = self.driver.find_element(By.ID, 'state').find_elements(By.TAG_NAME, 'strong')
            self.waitUntilLoaded(By.ID, 'temperature-table')
            temperature_table   = self.driver.find_element(By.XPATH, "//*[@id='temperature-table']/tbody").find_elements(By.TAG_NAME, "tr")

            try:
                ## 'status' : '' -> 'Idle'
                self.status['status'] = 'Idle' if self.status['status'] == '' and self.status['connection'] == 'Operational'  else self.status['status']

                ## 'status' : 'Idle' -> 'Printing {subject_name}'
                if self.status['connection'].find('Printing') != -1 and self.status['status'].find('Printing') == -1:
                    self.status['recent_work'] = self.status['subject_name']
                    self.status['subject_name'] = self.status['gcode_file']
                    self.status['status'] = "Printing {}".format(self.status['subject_name'])
                    print("[DEBUG] Status: {}".format(self.status['status']))
            
                ## 'status' : 'Printing {subject_name}' -> 'Done {subject_name}'
                if self.status['status'].find('Printing') != -1 and self.status['connection'] == 'Operational':
                    self.status['status'] = 'Done {}'.format(self.status['subject_name'])

                self.status['connection']   = device_status_table[0].text
                self.status['percentage']   = device_status_table[1].text
                self.status['gcode_file']   = device_status_table[2].text
                self.status['time_total']   = device_status_table[7].text
                self.status['time_elapsed'] = device_status_table[8].text
                self.status['time_left']    = device_status_table[9].text

                for data in temperature_table:
                    if data.text.find('Tool') != -1:
                        self.status['nozzle_temperature'] = float(data.text.split('Tool')[1].split('C')[0][1:-1].encode('utf8'))
                    if data.text.find('Bed') != -1:
                        self.status['bed_temperature'] = float(data.text.split('Bed')[1].split('C')[0][1:-1].encode('utf8'))
                
                print("\n==============================================================")
                print("[DEBUG] Device type: {}".format(self.status['device_type']))
                print("[DEBUG] Device name: {}".format(self.status['device_name']))
                print("[DEBUG] Connection: {}".format(self.status['connection']))
                print("[DEBUG] Status: {}".format(self.status['status']))
                print("[DEBUG] Subject name: {}".format(self.status['subject_name']))
                print("[DEBUG] Recent work: {}".format(self.status['recent_work']))
                # self.printStatus(self.status)
                
            except:
                print("[ERROR] Status data loaded failed !!!")
        
        # return self.status
    
    def printStatus(self, status):
            print("\n[INFO] 3D Printer Status (#{})".format(self.device_id))
            print("\n  * Device:")
            print("    - Status: {}".format(status['connection']))
            print("    - File: {}".format(status['percentage']))
            print("    - Send ratio: {}".format(status['gcode_file']))
            print("    - Total time: {}".format(status['time_total']))
            print("    - Time elapsed: {}".format(status['time_elapsed']))
            print("    - Time left: {}".format(status['time_left']))

            print("\n  * Temperature:")
            print("    - Nozzle: {}".format(status['nozzle_temperature']))
            print("    - Bed: {}".format(status['bed_temperature']))


    def command(self, cmd_dict):
        # self.updateStatus()
        cmd_keys = cmd_dict.keys()
        cmd_values = cmd_dict.values()

        for i in range(len(cmd_keys)):
            if cmd_keys[i] == 'connection':
                if cmd_values[i] == True and self.status['connection'].find('Offline') != -1: # connect printer
                    self.connectDevice()
                elif cmd_values[i] == False and self.status['connection'].find('Offline') == -1: # disconnect printer
                    self.disconnectDevice()
            elif cmd_keys[i] == 'print': # start printing
                if self.status['gcode_file'] != cmd_values[i]:
                    self.selectGcodeFile(file_name=cmd_values[i])
                self.startPrinting()
            elif cmd_keys[i] == 'cancel': # cancel printing
                self.cancelPrinting()
            elif cmd_keys[i] == 'pause': # pause printing
                self.pausePrinting()


    def startPrinting(self):
        self.waitUntilLoaded(By.ID, 'job_print')
        self.driver.find_element(By.ID, 'job_print').click()
    

    def pausePrinting(self):
        self.waitUntilLoaded(By.ID, 'job_pause')
        self.driver.find_element(By.ID, 'job_pause').click()


    def cancelPrinting(self):
        self.waitUntilLoaded(By.ID, 'job_cancel')
        self.driver.find_element(By.ID, 'job_cancel').click()
        buttons = self.driver.find_elements(By.XPATH, "//*[@class='modal-footer']//*[@href='javascript:void(0)']")
        for button in buttons:
            if button.text == 'Yes, cancel the print':
                button.click();   sleep(3.0)
                break


    def connectDevice(self):
        try:
            self.waitUntilLoaded(By.ID, 'connection_ports')
            Select(self.driver.find_element(By.ID, 'connection_ports')).select_by_visible_text('/dev/ttyUSB{}'.format(self.device_id));   sleep(0.5)
            self.driver.find_element(By.ID, 'printer_connect').click()
        except:
            pass


    def disconnectDevice(self):
        try:
            self.driver.find_element(By.XPATH, "//*[@data-target='#connection']").click()
            self.waitUntilLoaded(By.ID, 'printer_connect');     sleep(1.0)
            self.driver.find_element(By.ID, 'printer_connect').click()
        except:
            pass

    
    def selectGcodeFile(self, folder_name='Smartlab', file_name=''):
        file_name = '{}.gcode'.format(file_name)
        self.waitUntilLoaded(by=By.ID, name='files') #;     sleep(1.0)
        self.driver.find_element(By.XPATH, "//*[@id='files']//*[@type='search']").send_keys(file_name)

        # self.waitUntilLoaded(by=By.CLASS_NAME, name='title clickable') #;     sleep(1.0)
        # folders = self.driver.find_element(By.XPATH, "//*[@class='gcode_files']/*[@class='scroll-wrapper']//*[@class='title clickable']").click()
        
        self.waitUntilLoaded(by=By.CLASS_NAME, name='gcode_files') #;     sleep(1.0)
        flag = False
        while flag == False:
            try:
                self.waitUntilLoaded(by=By.CLASS_NAME, name='title clickable')
                folders = self.driver.find_element(By.XPATH, "//*[@class='gcode_files']/*[@class='scroll-wrapper']//*[@class='title clickable']").click()
            except:
                if flag == False:
                    files = self.driver.find_elements(By.XPATH, "//*[@class='gcode_files']//*[@class='title clickable']")
                    for f in files:
                        if f.text == file_name:
                            print("[INFO] G-code file '{}' is selected".format(file_name))
                            f.click()
                            flag = True
                            break
                if flag == False:
                    files = self.driver.find_elements(By.XPATH, "//*[@class='gcode_files']//*[@class='title clickable text-error']")
                    for f in files:
                        if f.text == file_name:
                            print("[INFO] G-code file '{}' is selected".format(file_name))
                            f.click()
                            flag = True
                            break
                if flag == False:
                    files = self.driver.find_elements(By.XPATH, "//*[@class='gcode_files']//*[@class='title clickable text-success']")
                    for f in files:
                        if f.text == file_name:
                            print("[INFO] G-code file '{}' is selected".format(file_name))
                            f.click()
                            flag = True
                            break

        self.waitUntilLoaded(by=By.CLASS_NAME, name='search-clear');     sleep(1.0)
        self.driver.find_element(By.CLASS_NAME, 'search-clear').click()
        


    def waitUntilLoaded(self, by=By.ID, name='state', timeout=5.0):
        try:
            element_present = EC.presence_of_element_located((by, name))
            WebDriverWait(self.driver, timeout).until(element_present)
            sleep(0.1)
            return True

        except TimeoutException:
            print("[ERROR] Timed out waiting for page to load")
            return False





if __name__ == '__main__':  

    printer = DeviceClass_3DP(device_name='printer0')
    
    print("[DEBUG] 1. Connect printer")
    printer.command({'connection': True})

    print("[DEBUG] 2. Print start")
    printer.command({'print': '201122_feedrate_test'})

    print("[DEBUG] 3. Cancel printing")
    printer.command({'cancel': True})

    print("[DEBUG] 4. Disconnect printer")
    printer.command({'connection': False})

