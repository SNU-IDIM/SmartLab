import sqlite3
import pandas as pd
import zmq
import json
from threading import Thread
from time import sleep


class DeviceManagerClient():
    def __init__(self, ip_='localhost', port_='5555', freq_=10):
        ## SQLite
        self.conn = sqlite3.connect("device_status.db", isolation_level=None)
        self.cur = self.conn.cursor()

        ## ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{ip_}:{port_}")

        ## Some variables
        self.device_dict_list = list()
        self.freq = float(freq_)

        ## Threading nodes
        self.thread_1 = Thread(target=self.connectZMQServer())
        self.thread_1.start()
    

    def __del__(self):
        self.thread_1.join()
        pass


    def saveDeviceStatus2DB(self, device_list_dict):
        try:
            for device_id in device_list_dict.keys():
                device_dict = device_list_dict[device_id]
                # device_dict['id'] = 2
                device_type = device_dict['device_type']
                print(device_id, device_type)
                pd.DataFrame.from_dict([device_dict]).to_sql(device_type, self.conn, if_exists='append', index=False)
        except:
            print("[ERROR]")


    def connectZMQServer(self):
        i = 0
        while True:
            i += 1
            request = dict()
            request['status'] = True
            print(f"Sending request ... {i}")
            self.socket.send_string(json.dumps(request))
            response = self.socket.recv()
            response = json.loads(response)
            self.saveDeviceStatus2DB(response)
            sleep(1.0/self.freq)


if __name__ == "__main__":
    # client = DeviceManagerClient(ip_="192.168.43.62", freq_=10)
    client = DeviceManagerClient(freq_=10)

