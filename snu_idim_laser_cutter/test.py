import zmq
import random
import sys
import time
import json

port = "5556"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect ("tcp://localhost:%s" % 9448)

while True:
    command = {'focus': 1}
    print('good')
    socket.send_string(json.dumps(command))
    message = json.loads(socket.recv())
    print(message)
    time.sleep(1)