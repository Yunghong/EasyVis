# code to train the model
from ultralytics import YOLO
from roboflow import Roboflow

def main():

  
  # Create a new YOLO model from scratch
  #model = YOLO('yolov8m-pose.pt')
  
  #resume
  model = YOLO('path_to_model.pt')
  freeze=[f"model.{x}." for x in range(0)]
  
  for k,v in model.named_parameters():
    v.requires_grad=True
    if any(x in k for x in freeze):
      print(f"freezing {k}")
      v.requires_grad=False
      
    

  results = model.train(data='path+to+data.yaml', epochs=1000, batch=40, patience=500, verbose=False)


if __name__ == "__main__":
    main()

