# Motion planning for Franka Emika Robot 

Goal: Data generation for training CNN models and testing the robustness of the models using the Panda robot involves not only generating training data but also evaluating the robustness of the models.

1. Assembly of the measurement setup
2. Implementing software
3. Postprocessing

## Measurement setup

![Measurement](https://github.com/Shaoxiang2021/PandaRobot-motion-planning/assets/88537773/f50d6534-fd88-4a73-b78c-f59e2c959470)

## Description

By utilizing Panda robots, numerous image data from various poses can be generated. Moreover, the alteration of brightness and sensor settings is also easily facilitated through this constructed measurement setup. The training data is collected under optimal conditions, with the camera positioned vertically to the target plane, optimal lighting, and camera settings. The data for testing the model's robustness is achieved by changing the camera's position, adjusting lighting conditions, and modifying camera settings. The robustness of most testing models is typically evaluated using artificially generated data through methods like data augmentation. However, this project obtains data directly from real images to test the model's robustness.

To know more, please refer to the main.pdf paper.

## Software Structure

Project  
|  
|------README.md  
|------requirements.txt  
|------main.pdf
|  
|_____ image  
|        
|_____ src  
|  
|------camera_roboter.py  
|------data_manager.py  
|------label_manager.py  
|------ueye_camera.py  
|------main_split_images.ipynb  
|------main_take_images.ipynb  
|  
  
camera_roboter.py: defines a class to control the robot, containing functions for motion planning and controlling the camera to capture images.  
ueye_camera.py: defines a class to control the camera, containing functions for video streaming and capturing images.  
label_manager.py: defines a class to manage image names, containing functions to rename images when necessary.  
data_manager.py: defines a class to split images and assign correct names, containing functions to split images into 3 sub-images with new names.  
  
main_take_images.ipynb: main code to generate images through robot motion planning.  
main_split_images.ipynb: main code to split images.  

## Requirements

Linux Ubuntu 20.04 LTS mit Echtzeit-Kernel der Version 5.9.1  
ROS-Framwork Noetic Ninjemys  
Python Version 3.7  
Pyueye Version 4.96.952  
OpenCV Version 4.8.0.76  
