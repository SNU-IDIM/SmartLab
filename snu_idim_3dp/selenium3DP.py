#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
from time import sleep
import json


from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.support.ui import Select, WebDriverWait
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.common.by import By



import rospy
from std_msgs.msg import String


# find_element(By.XPATH, '//button[text()="Some text"]')
# find_element(By.XPATH, '//button')
# find_element(By.ID, 'loginForm')
# find_element(By.LINK_TEXT, 'Continue')
# find_element(By.PARTIAL_LINK_TEXT, 'Conti')
# find_element(By.NAME, 'username')
# find_element(By.TAG_NAME, 'h1')
# find_element(By.CLASS_NAME, 'content')
# find_element(By.CSS_SELECTOR, 'p.content')
# find_elements(By.ID, 'loginForm')
# find_elements(By.CLASS_NAME, 'content')


class DeviceROSNode:
    def __init__(self, device_name):
        rospy.init_node(device_name)
        print("ROS initialized")
        self.device = Automate3DP(ID=0) #device_class
        self.device_status = dict()
        self.device_status_publisher = rospy.Publisher("{}/status".format(device_name), String, queue_size=1)
        rospy.Subscriber('{}/status'.format(device_name), String, self.test_cb, queue_size=1)
    


    def publishStatus(self):
        self.device_status = self.device.updateStatus()
        msg_json = json.dumps(self.device_status)
        self.device_status_publisher.publish(msg_json)
    
    

    def test_cb(self, msg):
        data = json.loads(msg.data)
        print(data['connection'])






class Automate3DP:

    def __init__(self, ID=0):
        self.device_id = str(ID)
        self.status = dict()
        self.status['device_id'] = self.device_id
        self.status['connection'] = ''
        
        ## Initialize 3DP connection
        self.driver = webdriver.Chrome(executable_path='chromedriver')
        self.driver.get('http://0.0.0.0:500{}/?#temp'.format(self.device_id))
        self.driver.find_element(By.ID, 'login-user').send_keys('wjyun')
        self.driver.find_element(By.ID, 'login-password').send_keys('idimahn1')
        self.driver.find_element(By.ID, 'login-button').click()
        self.connectDevice()

        ## Select a g-code file
        self.selectGcodeFile(folder_name='Smartlab', file_name='201122_feedrate_test.gcode')
        
        ## Start printing
        # self.startPrinting()
        # self.pausePrinting()

        ## Cancle priting
        # sleep(5)
        # self.cancelPrinting()

        ## Disconnect device
        # sleep(5)
        # self.disconnectDevice()

    

    def __del__(self):
        self.driver.close()
    


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
                button.click()
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
        self.waitUntilLoaded(by=By.CLASS_NAME, name='gcode_files');     sleep(1.0)
        folders = self.driver.find_elements(By.XPATH, "//*[@class='gcode_files']/*[@class='scroll-wrapper']//*[@class='title clickable']")
        for folder in folders:
            if folder.text == folder_name:
                folder.click()
                break
        
        self.waitUntilLoaded(by=By.CLASS_NAME, name='gcode_files');     sleep(1.0)

        flag = False
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

    

    def waitUntilLoaded(self, by=By.ID, name='state', timeout=5.0):
        try:
            element_present = EC.presence_of_element_located((by, name))
            WebDriverWait(self.driver, timeout).until(element_present)
            sleep(1.0)
        except TimeoutException:
            print("[ERROR] Timed out waiting for page to load")

    

    def updateStatus(self):
        self.waitUntilLoaded(By.ID, 'state')
        device_status_table = self.driver.find_element(By.ID, 'state').find_elements(By.TAG_NAME, 'strong')
        self.waitUntilLoaded(By.ID, 'temperature-table')
        temperature_table   = self.driver.find_element(By.XPATH, "//*[@id='temperature-table']/tbody").find_elements(By.TAG_NAME, "tr")

        try:
            self.status['connection']   = device_status_table[0].text
            self.status['percentage']   = device_status_table[1].text
            self.status['gcode_name']   = device_status_table[2].text
            self.status['time_total']   = device_status_table[7].text
            self.status['time_elapsed'] = device_status_table[8].text
            self.status['time_left']    = device_status_table[9].text

            for data in temperature_table:
                if data.text.find('Tool') != -1:
                    self.status['nozzle_temperature'] = float(data.text.split('Tool')[1].split('C')[0][1:-1].encode('utf8'))
                if data.text.find('Bed') != -1:
                    self.status['bed_temperature'] = float(data.text.split('Bed')[1].split('C')[0][1:-1].encode('utf8'))

            print("\n[INFO] 3D Printer Status (#{})".format(self.device_id))
            print("\n  * Device:")
            print("    - Status: {}".format(self.status['connection']))
            print("    - File: {}".format(self.status['percentage']))
            print("    - Send ratio: {}".format(self.status['gcode_name']))
            print("    - Total time: {}".format(self.status['time_total']))
            print("    - Time elapsed: {}".format(self.status['time_elapsed']))
            print("    - Time left: {}".format(self.status['time_left']))

            print("\n  * Temperature:")
            print("    - Nozzle: {}".format(self.status['nozzle_temperature']))
            print("    - Bed: {}".format(self.status['bed_temperature']))
        except:
            print("[ERROR] Status data loaded failed !!!")
        
        return self.status




if __name__ == '__main__':  

    # t = Automate3DP(ID=0)

    r = DeviceROSNode('printer0') 

    while True:
        sleep(1)
        # t.updateStatus()
        r.publishStatus()
