#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
from time import sleep

from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.support.ui import Select, WebDriverWait
from selenium.common.exceptions import TimeoutException
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.common.by import By


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



class auto3DP:
    def __init__(self, ID=0):
        self.device_id = str(ID)
        self.device_status = ''

        self.status_temp_nozzle = 0.0
        self.status_temp_bed = 0.0
        
        ## Initialize 3DP connection
        self.driver = webdriver.Chrome(executable_path='chromedriver')
        self.driver.get('http://0.0.0.0:500{}/?#temp'.format(self.device_id))

        self.driver.find_element(By.ID, 'login-user').send_keys('wjyun')
        self.driver.find_element(By.ID, 'login-password').send_keys('dnjswo93*')
        self.driver.find_element(By.ID, 'login-button').click()

        self.updateStatus()

        self.startPrinting()
        self.pausePrinting()
        self.cancelPrinting()

    
    def __del__(self):
        pass
        # self.driver.close()

    def startPrinting(self):
        self.driver.find_element(By.ID, 'job_print').click()
    
    def pausePrinting(self):
        self.driver.find_element(By.ID, 'job_pause').click()

    def cancelPrinting(self):
        self.driver.find_element(By.ID, 'job_cancel').click()
    


    def connectDevice(self):
        self.waitUntilLoaded(By.ID, 'connection_ports')
        Select(self.driver.find_element(By.ID, 'connection_ports')).select_by_visible_text('/dev/ttyS{}'.format(self.device_id));   sleep(0.5)
        self.driver.find_element(By.ID, 'printer_connect').click()

    
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

        self.device_status = device_status_table[0].text

        for data in temperature_table:
            if data.text.find('Tool') != -1:
                self.status_temp_nozzle = float(data.text.split('Tool')[1].split('C')[0][1:-1].encode('utf8'))
            if data.text.find('Bed') != -1:
                self.status_temp_bed = float(data.text.split('Bed')[1].split('C')[0][1:-1].encode('utf8'))

        print("[INFO] 3D Printer Status (#{})".format(self.device_id))
        print("\n  * Device:")
        print("    - Status: {}".format(self.device_status))
        print("\n  * Temperature:")
        print("    - Nozzle: {}".format(self.status_temp_nozzle))
        print("    - Bed: {}".format(self.status_temp_bed))



        

t = auto3DP(0)