#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 17 14:05:33 2020

@author: steven
"""
import socket
import sys
import pickle
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import time
import threading
#import cv2

HOST='192.168.86.44'
PORT=54321
s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
try:
    s.bind((HOST,PORT))
except:
    print("IP/Port already occupied...  Closing existing connection")

s.listen(10)

conn, addr = s.accept()
conn.settimeout(2)

bytes_rec = 4096
target_bytes = 921764
shared_list = []

def receiver():
    while True:    
        data = b''
        received = 0
        while True:
            try:
                i = conn.recv(min(target_bytes - received, bytes_rec))
            except:
                print("Waiting to receive next package.....")
                break
                
            data += i
            received += len(i)
            if len(data) == 921764: break
        shared_list.append(data)

def player():
    cntr = 1
    while True:
        if len(shared_list)==0:
            time.sleep(.1)
        else:
            myD= shared_list.pop()
            while len(shared_list)>0: shared_list.pop()
            if len(myD) == 921764:
                cntr += 1
                print(cntr)
                frame=pickle.loads(myD)
                rgb = np.fliplr(frame.reshape(-1,3)).reshape(frame.shape)
                img = Image.fromarray(rgb, 'RGB')
                plt.imshow(img)
                plt.pause(0.05)

t1 = threading.Thread(target=receiver)
t1.start()
t2 = threading.Thread(target=player)
t2.start()
