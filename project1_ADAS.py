# some heavy credit to these, and others, and viewers like you:
# https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb
# https://github.com/datitran/object_detector_app/blob/master/object_detection_app.py

import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import sys
import pickle
sys.path.append('/home/pi/Desktop/tf/models/research/object_detection')
#os.chdir('/home/pi/Desktop/tf/models/research/')
from utils import label_map_util
from utils import visualization_utils as vis_util

from socket import *
import time
import struct

model = 'ssdlite_mobilenet_v2_coco_2018_05_09'
model_dir = '/home/pi/Desktop/tf/models/research/object_detection/'+model+'/frozen_inference_graph.pb'
labels_dir = '/home/pi/Desktop/tf/models/research/object_detection/data/mscoco_label_map.pbtxt'

label_map = label_map_util.load_labelmap(labels_dir)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=250, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(model_dir, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)


image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

frame_rate_calc = 10
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX

w = 640
h = 480
camera = PiCamera()
camera.resolution = (w,h)
camera.framerate = 2.5
rawCapture = PiRGBArray(camera, size=(w,h))
rawCapture.truncate(0)


msg = "ON"

myAddr = ('192.168.86.247', 54321)
#myAddr2 = ('192.168.86.247', 60891)
gatewayAddr = ('192.168.86.248', 8888)
headAddr = ('192.168.86.44',54321)

streamSoc = socket(AF_INET, SOCK_STREAM)
unConnected = True
while unConnected:
    try:
        streamSoc.connect(headAddr)
        print("Connected to Head Unit!")
        unConnected = False
    except:
        print("Waiting to Connect to Head Unit...")
        time.sleep(3)
        print("Attempting Again...")
        
dist = 10

soc = socket(AF_INET, SOCK_DGRAM)
soc.bind(myAddr)
soc.settimeout(.1)

import threading

#define global threads
to_head = []
distances = []
distances_to_frame = []
obj_detections = []
obj_detections_for_can=[]
official_msg = ["OFF"]
    
def distance_monitor():
    while True:
    #need to empty buffer, then capture latest message
        g = 4096
        while True:
            try:
                rec_data, addr = soc.recvfrom(g)
            except:
                if g == 1: break
                g = g / 2
        #try 5 times to pull new udp distance packet
        g = 1
        while True:
            try:
                rec_data, addr = soc.recvfrom(4)
                dist = struct.unpack('f', rec_data)[0]
                distances.append(dist)
                #print("Received Distance!  Latest = ", distances[-1])
                break
            except:
                g+=1
                time.sleep(.025)
                if g == 5:
                    #print("DISTANCE IS DELAYED...")
                    break
        time.sleep(.025)
                
def can_communicator():
    #check for close humans....
    while True:
        if len(obj_detections_for_can)==0 or len(distances)==0:
            time.sleep(.1)
        else:
            msg = "OFF"
            [scores, classes, num] = obj_detections_for_can.pop()
            while len(obj_detections_for_can)>0: obj_detections_for_can.pop()
            mDist = distances.pop()
            while len(distances)>0: distances.pop()
            
            distances_to_frame.append(mDist)
            while len(distances)>0: distances.pop()
            #print('lastest distance:',mDist)
            if mDist < 150:
                for p in range(int(num[0])):
                    if scores[0][p] > .4 and classes[0][p] == 1:
                        msg = "ON"
            official_msg.append(msg)
            soc.sendto( msg.encode(), gatewayAddr)
            time.sleep(.1)

def frame_drawing():
    while True:
        if len(obj_detections)==0 or len(distances_to_frame)==0:
            time.sleep(.1)
        else:
            [frame, boxes, classes, scores] = obj_detections.pop()
            while len(obj_detections)>0: obj_detections.pop()
            fdDist = distances_to_frame.pop()
            while len(distances_to_frame)>0: distances_to_frame.pop()
            vis_util.visualize_boxes_and_labels_on_image_array(
                frame,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                line_thickness=8,
                min_score_thresh=0.40)
            if official_msg[-1] == "ON":
                cv2.putText(frame,"!!! LED BRAKE-LIGHT ON !!!",(30,50),font,1,(15,50,255),2,cv2.LINE_AA)
                cv2.putText(frame,"Distance:  "+str(round(fdDist,0))+"cm!",(30,70),font,1,(15,50,255),2,cv2.LINE_AA)
            else:
                cv2.putText(frame,"LED BRAKE-LIGHT OFF",(30,50),font,1,(255,50,15),2,cv2.LINE_AA)
                cv2.putText(frame,"Distance:  "+str(round(fdDist,0))+"cm!",(30,70),font,1,(255,50,15),2,cv2.LINE_AA)
            to_head.append(pickle.dumps(frame))
    

def object_detector():
    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        frame = np.copy(frame1.array)
        frame.setflags(write=1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_expanded = np.expand_dims(frame_rgb, axis=0)
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})
        obj_detections.append([frame, boxes, classes, scores])
        obj_detections_for_can.append([scores, classes, num])
        rawCapture.truncate(0)
        
        

def headUnit_communicator():
    while True:
        if len(to_head) == 0:
            time.sleep(.1)
        else:
            d = to_head.pop()
            while len(to_head)>0: to_head.pop()
            streamSoc.sendall(d)
            print('off to head!')
            
t1 = threading.Thread(target=distance_monitor)
t1.start()
t2 = threading.Thread(target=can_communicator)
t2.start()
t3 = threading.Thread(target=object_detector)
t3.start()
t4 = threading.Thread(target=frame_drawing)
t4.start()
t5 = threading.Thread(target=headUnit_communicator)
t5.start()
            
if cv2.waitKey(1) == ord('q'):
    rawCapture.truncate(0)
    camera.close()
    cv2.destroyAllWindows()





