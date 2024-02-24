# Motion planning for Franka Emika Robot 

Goal: Data generation for training CNN models and testing the robustness of the models using the Panda robot involves not only generating training data but also evaluating the robustness of the models.

1. Assembly of the measurement setup
2. Implementing software
3. Postprocessing

## Aufbau des Messstandes

![Measurement](https://github.com/Shaoxiang2021/PandaRobot-motion-planning/assets/88537773/f50d6534-fd88-4a73-b78c-f59e2c959470)

## Description

By utilizing Panda robots, numerous image data from various poses can be generated. Moreover, the alteration of brightness and sensor settings is also easily facilitated through this constructed measurement setup. The training data is collected under optimal conditions, with the camera positioned vertically to the target plane, optimal lighting, and camera settings. The data for testing the model's robustness is achieved by changing the camera's position, adjusting lighting conditions, and modifying camera settings. The robustness of most testing models is typically evaluated using artificially generated data through methods like data augmentation. However, this project obtains data directly from real images to test the model's robustness.


