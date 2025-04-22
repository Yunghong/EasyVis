# EasyVis
## Description
Our real-time 3D rendering system is targeted for laparoscopic surgery. Currently, our work is in the early stages, focusing on a laparoscopic surgery box trainer environment. The development process follows a coarse-to-fine approach: first, we establish the framework, then we fill in and refine the details.

A demo of our work. We demonstrate real-time 3D reconstruction and rendering for a laparoscopic surgery box trainer, specifically focusing on the bean drop task. The shown version is targeted for an experiment that will be conducted in Spring 2025. We plan to sample data from the operations performed by medical students at UM-Madison. The process achieves a **real-time** performance of 12.6 ms per frame, from reading 5 cameras to completing 3D rendering.

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
* Yung-Hong Sun, J. Chen, T. Lin, H. Jiang, Y.H. Hu, Efficient Undistinguishable Dense 3D Objects Association Based on Multi-Camera System (In preparation).
* Yung-Hong Sun, G. Shen, J. Chen, J. Fernandes, H. Jiang, and Y.H. Hu. EasyVis2: A Real Time Multi-view 3D Visualization System for Laparoscopic Surgery Training Enhanced by a Deep Neural Network YOLOv8-Pose. arXiv preprint arXiv:2412.16742 (2024). 
* Yung-Hong Sun, J. Ke, J. Fernandes, J. Chen, H. Jiang, Y.H. Hu, EasyVis: An Interactive 3D Visualization Software System for Laparoscopic Box Trainer. Updates in Surgery, 2025. DOI: 10.1007/s13304-025-02153-w.

## Presentation & Poster
* Yung-Hong Sun, J. Ke, J. Fernandes, J. Chen, H. Jiang, Y.H. Hu, EasyVis: A Real-time 3D Rendering System for Laparoscopic Surgery Training Box. Oral presentation, BMES 2024 Annual Meeting. Baltimore, MD, United States, Oct 2024.

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
