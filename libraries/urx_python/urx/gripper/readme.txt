https://robotics.stackexchange.com/questions/15782/onrobot-rg2-gripper-on-ur3-with-tcp-communication


    import socket
    import time
    HOST = "10.0.2.15"    # The remote host
    PORT = 30002              # The same port as used by the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    print("Starting Program")
    f=open("/home/ur/Desktop/TestScript.script", "r")
    s.send (f.read() + "/n")
    data = s.recv(1024)
    s.close()
    print ("Closed connection and received data")﻿
