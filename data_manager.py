import os
import time
from typing import Any

class DataManager():
    def __init__(self):

        self.directory = "images"

        self.id = "None"
        self.speed = "None"
        self.rotate_speed = "None"
        self.date = "None"
        self.force = "None"
        
        self.brightness = "0"
        self.rotated_angle_z = "0"
        self.rotated_angle_x = "0"
        self.displacement_in_z = '0'
        self.zoom_blur = '0'
        self.camera_noise = '0'

        self.generate_main_folder()
    
    def generate_main_folder(self):
        os.makedirs(self.directory, exist_ok=True)

    def ask_brightness(self):
        self.brightness = self.input_brightness()

    def set_brightness(self, brightness):
        self.brightness = str(brightness)

    def set_rotate_angle_x(self, angle_x):
        self.rotated_angle_x = str(round(angle_x, 2))
        return angle_x
    
    def set_rotate_angle_z(self, angle_z):
        self.rotated_angle_z = str(round(angle_z, 2))
        return angle_z

    def set_displacement_in_z(self, z):
        self.displacement_in_z = str(round(z, 3))
        return z
    
    def set_zoom_blur(self, blur):
        self.zoom_blur = str(blur)
    
    def set_camera_noise(self, noise):
        self.camera_noise = str(noise)
    
    def set_basic_info(self, dict_info):
        self.id = dict_info["id"]
        self.speed = dict_info["speed"]
        self.rotate_speed = dict_info["rotate_speed"]
        self.date = dict_info["date"]
        self.force = dict_info["force"]
    
    def print_and_write_into_log(self, text):
        text_line = time.strftime("%d.%m.%Y %R:%S") + ": " + text
        print(text)
        os.makedirs(self.directory, exist_ok=True)
        file_path = os.path.join(self.directory, "log.txt")
        file = open(file_path, 'a')
        file.write(text_line + '\n')
        file.close()

    def generate_label(self, x):
        label = '_'.join([time.strftime("%d.%m.%Y_%R:%S"), str(round(x, 2)), self.brightness, self.rotated_angle_x, self.rotated_angle_z, self.zoom_blur]) + ".jpg"
        return label
    
    def generate_image_path(self, x):
        folder_name = '_'.join([self.id, self.speed, self.rotate_speed, self.force])
        folder_path = os.path.join(self.directory, folder_name)
        os.makedirs(folder_path, exist_ok=True)
        image_label = self.generate_label(x,)
        image_path = os.path.join(folder_path, image_label)
        return image_path
    
    @staticmethod
    def input_check():
        while(True):
            check = input("Sind alle Info richtig? (J/N)")
            if check == 'J' or check == 'N':
                return check
            else:
                print("Falsche Eingabe, bitte versuch nochmal!")

    @staticmethod
    def input_brightness():
        while(True):
            brightness = input("Bitte die Heilligkeit eingeben: ")
            if brightness.isdigit():
                return brightness
            else:
                print("Falsche Eingabe, bitte versuch nochmal!")
