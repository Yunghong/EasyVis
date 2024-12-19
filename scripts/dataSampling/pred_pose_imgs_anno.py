# this file transfer and save the yolov8-pose prediction results into the json format

from ultralytics import YOLO
from roboflow import Roboflow
from PIL import Image
import cv2
import os
import json
import numpy as np
import shutil
import glob



def main():
    files = glob.glob('dstRaw/*')
    for f in files:
        os.remove(f)
    
    #model = YOLO('model/b11super.pt')  # load a custom model
    model = YOLO('../runs/pose/weights/last.pt') 
   
    
    img_info=[]
    anno=[]
    
    imgId=0
    annoId=0

    #for path in os.listdir('onlineSave/src/'):
    for path in os.listdir('src/'):
    #for path in os.listdir('dst/'):
        print(path)
        
        #frame=cv2.imread('onlineSave/src/'+path)
        #shutil.copy('./src/'+path[3:], 'dstRaw/'+path[3:])
        
        frame=cv2.imread('src/'+path)
        #shutil.copy('src/'+path, 'dstRaw/'+path)
        #shutil.copy('src/'+path, 'dstRaw/val'+path)


        
        results = model(frame)
        annotated_frame = results[0].plot()
        
        

        
        for r in results:
            boxes = r.boxes
            kps = r.keypoints
            for box,kp in zip(boxes,kps):
            
                
                b = box.xyxy[0].detach().cpu().numpy().astype(float)  # get box coordinates in (left, top, right, bottom) format
                bb=[b[0],b[1],b[2]-b[0],b[3]-b[1]]
                #bb=[(b[0]+b[2])/2,(b[1]+b[3])/2,b[2]-b[0],b[3]-b[1]]
                area=(b[2]-b[0])*(b[3]-b[1])
                c = box.cls.detach().cpu().numpy()
                #kp=np.round(kp.xy.detach().cpu().numpy().astype(float),2)
                kp=np.round(kp.data.detach().cpu().numpy().astype(float),2)
                #kpStat=np.rint(kp.data)
                

                if(c[0]==1):
                    stat0=0
                    if(int(kp[0,0,2])):
                        stat0=2
                        
                    stat1=0
                    if(int(kp[0,1,2])):
                        stat1=2
                        
                    stat2=0
                    if(int(kp[0,2,2])):
                        stat2=2
                        
                    stat3=0
                    if(int(kp[0,3,2])):
                        stat3=2
                    
                    tmpAnno={"id":annoId,
                            "image_id":imgId,
                            "category_id":2,
                            "bbox":bb,
                            "area":area,
                            "segmentation":[],
                            "iscrowd":0,
                            "keypoints":[kp[0,0,0],kp[0,0,1],stat0,kp[0,1,0],kp[0,1,1],stat1,
                                         kp[0,2,0],kp[0,2,1],stat2,kp[0,3,0],kp[0,3,1],stat3]
                            #"keypoints":[kp[0,0,0],kp[0,0,1],2,kp[0,1,0],kp[0,1,1],2,
                            #             kp[0,2,0],kp[0,2,1],2,kp[0,3,0],kp[0,3,1],2]
                            }
                    anno.append(tmpAnno)
                    annoId+=1
                    
                else:
                    stat0=0
                    if(int(kp[0,0,2])):
                        stat0=2
                        
                    tmpAnno={"id":annoId,
                            "image_id":imgId,
                            "category_id":1,
                            "bbox":bb,
                            "area":area,
                            "segmentation":[],
                            "iscrowd":0,
                            "keypoints":[kp[0,0,0],kp[0,0,1],stat0]
                            #"keypoints":[kp[0,0,0],kp[0,0,1],2]
                            }
                    anno.append(tmpAnno)
                    annoId+=1                    
        
        tmp_img_info={ "id":imgId,
                       "file_name":path,
                       #"file_name":'val'+path,
                       "height":480,
                       "width":640}

        img_info.append(tmp_img_info)
        imgId+=1
        
   
    data = {
        "categories":[{"id":0,
                       "name":"Grasper",
                       "supercategory":"none"},
                      
                      {"id":1,
                       "name":"bean",
                       "supercategory":"Grasper",
                       "keypoints":["bean"],
                       "skeleton":[]},
                       
                      {"id":2,
                       "name":"grasper",
                       "supercategory":"Grasper",
                       "keypoints":["locator","joint","right","left"],
                       "skeleton":[[1,2],[2,3],[2,4]]}],

        "images":img_info,

        "annotations":anno
    }
    
    with open('output.json', 'w') as f:
        json.dump(data, f)
  
if __name__ == "__main__":
    main()

