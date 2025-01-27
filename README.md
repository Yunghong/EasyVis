# EasyVis
## Description
Our real-time 3D rendering system is targeted for laparoscopic surgery. Currently, our work is in the early stages, focusing on a laparoscopic surgery box trainer environment. The development process follows a coarse-to-fine approach: first, we establish the framework, then we fill in and refine the details.

A demo of our work. We demonstrate real-time 3D reconstruction and rendering for a laparoscopic surgery box trainer, specifically focusing on the bean drop task. The shown version is targeted for an experiment that will be conducted in Spring 2025. We plan to sample data from the operations performed by medical students at UM-Madison. 

![EasyVis Demo GIF](https://github.com/Yunghong/EasyVis/blob/main/easyvis%20demo.gif)

Demo of real-time 3D visualization when operating the LS tool.

![EasyVis Demo GIF](https://github.com/Yunghong/EasyVis/blob/main/novelView.gif)

Demo of 3D visualization over meat. The distance between the surgical tool and the tissue can be directly observed through EasyVis, although this distance is invisible from the camera view. (a) captured camera views. (b) rendered novel view.

![Meat Visualization](https://github.com/Yunghong/EasyVis/blob/main/3DVisualizationOverMeat.png)

The workflow of our work.

![Project Logo](https://github.com/Yunghong/EasyVis/blob/main/easyvisPipelineV2.png)

## ST-Pose Dataset
[ST-Pose](https://uwmadison.box.com/s/49t1hc1ctdpe1a70ssqj0xw46azofbq4)

The dataset is in YOLOv5 format.

## Publications
* [EasyVis2: A Real-Time Multi-view 3D Visualization for Laparoscopic Surgery Training Enhanced by a Deep Neural Network YOLOv8-Pose](https://arxiv.org/abs/2412.16742)
* EasyVis: A Real-Time 3D Visualization Software System for Laparoscopic Surgery Box Trainer (Under review in the journal Updates In Surgery, minor revision)

## Acknowledgments
This work was supported by the National Institute of Biomedical Imaging and Bioengineering (NIBIB) of the U.S. National Institutes of Health (NIH) under award number R01EB019460.

## Contributors
- Yung-Hong Sun
- Gefei Shen
- Jiangang Chen
- Jayer Fernandes
- Jianwei Ke
- Hongrui Jiang
- Yu Hen Hu
