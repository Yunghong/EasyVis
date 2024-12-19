# this code help save images from online pose estimation
# We save images with larger errors
 
from ultralytics import YOLO
from roboflow import Roboflow
from PIL import Image
import cv2
import os
import json
import numpy as np
import os
import glob

def main():
    #camera setting    
    addr1 = 0
    addr2 = 6
    addr3 = 10
    addr4 = 12
    addr5 = 16

    cap1 = cv2.VideoCapture(addr1)
    cap2 = cv2.VideoCapture(addr2)
    cap3 = cv2.VideoCapture(addr3)
    cap4 = cv2.VideoCapture(addr4)
    cap5 = cv2.VideoCapture(addr5)
    
    frame_width = int(cap1.get(3))
    frame_height = int(cap1.get(4))
    size = (frame_width, frame_height)
    
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("frame", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    
    batch='meat'
    
    if not os.path.exists('onlineSave/'+batch):
        os.makedirs('onlineSave/'+batch)
        
        os.makedirs('onlineSave/'+batch+'/src')
        os.makedirs('onlineSave/'+batch+'/painted')
        os.makedirs('onlineSave/'+batch+'/videosFrames')
        
    writer1 = cv2.VideoWriter('onlineSave/'+batch+'/v1.mp4',cv2.VideoWriter_fourcc(*'MJPG'),30, size) 
    writer2 = cv2.VideoWriter('onlineSave/'+batch+'/v2.mp4',cv2.VideoWriter_fourcc(*'MJPG'),30, size) 
    writer3 = cv2.VideoWriter('onlineSave/'+batch+'/v3.mp4',cv2.VideoWriter_fourcc(*'MJPG'),30, size) 
    writer4 = cv2.VideoWriter('onlineSave/'+batch+'/v4.mp4',cv2.VideoWriter_fourcc(*'MJPG'),30, size) 
    writer5 = cv2.VideoWriter('onlineSave/'+batch+'/v5.mp4',cv2.VideoWriter_fourcc(*'MJPG'),30, size) 
        
    cnt=0
    
    #model setting
    model = YOLO('../runs/pose/MarkerFree/weights/last.pt') 
    
    img_info=[]
    anno=[]
    
    imgId=0
    annoId=0
    
    toggle=True
    textCnt1=0
    textCnt2=0
    textCnt3=0
    textCnt4=0
    textCnt5=0

    while(True):
        # Capture frame-by-frame
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        ret3, frame3 = cap3.read()
        ret4, frame4 = cap4.read()
        ret5, frame5 = cap5.read()

        frame=[frame1,frame2,frame3,frame4,frame5]
 
        #results = model.track(frame,conf=0.8, persist=True, tracker="bytetrack.yaml")
        results = model(frame,conf=0.4,)
        annotated_frame=[]
        for i in range(len(results)):
            annotated_frame.append(results[i].plot())
            
        pressKey=cv2.waitKey(1)    
            
        if pressKey & 0xFF == ord('1') and toggle:
            cv2.imwrite('onlineSave/'+batch+'/src/'+batch+'v1'+str(cnt).zfill(7)+'.jpg',frame1)
            cv2.imwrite('onlineSave/'+batch+'/painted/'+batch+'v1'+str(cnt).zfill(7)+'.jpg',annotated_frame[0])
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v1'+str(cnt).zfill(7)+'.jpg',frame1)
            textCnt1=30
            #cnt+=1
            toggle=False
            continue
        elif pressKey & 0xFF == ord('1') and not toggle:
            continue
        else:
            toggle=True
           
        if(textCnt1>0): 
            annotated_frame[0] = cv2.putText(annotated_frame[0], 'Save', (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 0) , 2, cv2.LINE_AA) 
            textCnt1-=1
            
        if pressKey & 0xFF == ord('2') and toggle:
            cv2.imwrite('onlineSave/'+batch+'/src/'+batch+'v2'+str(cnt).zfill(7)+'.jpg',frame2)
            cv2.imwrite('onlineSave/'+batch+'/painted/'+batch+'v2'+str(cnt).zfill(7)+'.jpg',annotated_frame[1])
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v2'+str(cnt).zfill(7)+'.jpg',frame2)
            textCnt2=30
            #cnt+=1
            toggle=False
            continue
        elif pressKey & 0xFF == ord('2') and not toggle:
            continue
        else:
            toggle=True
           
        if(textCnt2>0): 
            annotated_frame[1] = cv2.putText(annotated_frame[1], 'Save', (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 0) , 2, cv2.LINE_AA) 
            textCnt2-=1
            
        if pressKey & 0xFF == ord('3') and toggle:
            cv2.imwrite('onlineSave/'+batch+'/src/'+batch+'v3'+str(cnt).zfill(7)+'.jpg',frame3)
            cv2.imwrite('onlineSave/'+batch+'/painted/'+batch+'v3'+str(cnt).zfill(7)+'.jpg',annotated_frame[2])
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v3'+str(cnt).zfill(7)+'.jpg',frame3)
            textCnt3=30
            #cnt+=1
            toggle=False
            continue
        elif pressKey & 0xFF == ord('3') and not toggle:
            continue
        else:
            toggle=True
           
        if(textCnt3>0): 
            annotated_frame[2] = cv2.putText(annotated_frame[2], 'Save', (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 0) , 2, cv2.LINE_AA) 
            textCnt3-=1   
         
        if pressKey & 0xFF == ord('4') and toggle:
            cv2.imwrite('onlineSave/'+batch+'/src/'+batch+'v4'+str(cnt).zfill(7)+'.jpg',frame4)
            cv2.imwrite('onlineSave/'+batch+'/painted/'+batch+'v4'+str(cnt).zfill(7)+'.jpg',annotated_frame[3])
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v4'+str(cnt).zfill(7)+'.jpg',frame4)
            textCnt4=30
            #cnt+=1
            toggle=False
            continue
        elif pressKey & 0xFF == ord('4') and not toggle:
            continue
        else:
            toggle=True
           
        if(textCnt4>0): 
            annotated_frame[3] = cv2.putText(annotated_frame[3], 'Save', (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 0) , 2, cv2.LINE_AA) 
            textCnt4-=1
            
        if pressKey & 0xFF == ord('5') and toggle:
            cv2.imwrite('onlineSave/'+batch+'/src/'+batch+'v5'+str(cnt).zfill(7)+'.jpg',frame5)
            cv2.imwrite('onlineSave/'+batch+'/painted/'+batch+'v5'+str(cnt).zfill(7)+'.jpg',annotated_frame[4])
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v5'+str(cnt).zfill(7)+'.jpg',frame5)
            textCnt5=30
            #cnt+=1
            toggle=False
            continue
        elif pressKey & 0xFF == ord('5') and not toggle:
            continue
        else:
            toggle=True
           
        if(textCnt5>0): 
            annotated_frame[4] = cv2.putText(annotated_frame[4], 'Save', (50, 50), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 0) , 2, cv2.LINE_AA) 
            textCnt5-=1
            
        writer1.write(frame1)
        writer2.write(frame2)
        writer3.write(frame3)
        writer4.write(frame4)
        writer5.write(frame5)
        
        if(cnt%30==0):
            frame1=results[0].plot()
            frame2=results[1].plot()
            frame3=results[2].plot()
            frame4=results[3].plot()
            frame5=results[4].plot()
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v1'+str(cnt).zfill(7)+'.jpg',frame1)
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v2'+str(cnt).zfill(7)+'.jpg',frame2)
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v3'+str(cnt).zfill(7)+'.jpg',frame3)
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v4'+str(cnt).zfill(7)+'.jpg',frame4)
            cv2.imwrite('onlineSave/'+batch+'/videosFrames/'+batch+'v5'+str(cnt).zfill(7)+'.jpg',frame5)
        
        cnt+=1
        #annotated_frame = list(map(lambda result: result.plot(), frame))
        annotated_frameRow1=cv2.hconcat([annotated_frame[0],annotated_frame[1],annotated_frame[2]])
        annotated_frameRow2=cv2.hconcat([annotated_frame[3],annotated_frame[4],np.zeros(frame1.shape, np.uint8)])
        
        annotated_frameWhole=cv2.vconcat([annotated_frameRow1,annotated_frameRow2])
        
        cv2.imshow('frame',annotated_frameWhole)
        
        # Display the resulting frame
        '''
        cv2.imshow('frame1',annotated_frame[0])
        cv2.imshow('frame2',annotated_frame[1])
        cv2.imshow('frame3',annotated_frame[2])
        cv2.imshow('frame4',annotated_frame[3])
        cv2.imshow('frame5',annotated_frame[4])
        '''
       
        
        if pressKey & 0xFF == ord('q') and toggle:
            break
            toggle=False
        elif pressKey & 0xFF == ord('q') and not toggle:
            continue
        else:
            toggle=True

    # When everything done, release the capture
    cap1.release()
    cap2.release()
    cap3.release()
    cap4.release()
    cap5.release()
    
    writer1.release()
    writer2.release()
    writer3.release()
    writer4.release()
    writer5.release()
    cv2.destroyAllWindows()

  
if __name__ == "__main__":
    main()

