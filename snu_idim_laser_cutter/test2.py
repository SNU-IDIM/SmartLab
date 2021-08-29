import serial
import serial.tools.list_ports


def getSerialPort(desc='Arduino'):
    ports = list(serial.tools.list_ports.comports())
    port_ = ''

    for p in ports:
        print(p)
        if p.description.find(desc) != -1:
            port_ = p.device
            
    port_ = port_ if port_ != '' else None
    return port_

print(getSerialPort())