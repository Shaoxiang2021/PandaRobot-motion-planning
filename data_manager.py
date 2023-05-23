import os

class DataManager():
    def __init__(self):

        self.directory = "images"

        self.id = None
        self.speed = None
        self.rotate_speed = None
        self.index = None

        self.brightness = "0"
        self.rotated_angle_z = "0"
        self.rotated_angle_x = "0"

    def ask_example_info(self):
        while(True):
            id = input("Bitte geben Sie die Probe-ID: ")
            speed = input("Bitte Vorschubgeschwindigkeit eingeben:" )
            rotate_speed = input("Bitte Drehzahl eingeben: ")
            date = input("Bitte Datum eingeben: ")
            index = input("Bitte Index von Probe eingeben: ")
            print("Überprüfen bitte die Info ...")
            print("")
            print("Probe-ID = {}".format(id))
            print("Vorschubgeschwindigkeit = {}".format(speed))
            print("Drehzahl = {}".format(rotate_speed))
            print("Datum lautet: {}".format(date))
            print("")

            check = self.input_check()

            if check == "J":
                self.id = id
                self.speed = speed
                self.rotate_speed = rotate_speed
                self.index = index
                break
            elif check == "N":
                print("")
                print("Bitte geb richtige Info ...")
                print("")
    
    def print_and_write_into_log(self, text):
        print(text)
        os.makedirs(self.directory, exist_ok=True)
        file_path = os.path.join(self.directory, "log.txt")
        file = open(file_path, 'a')
        file.write(text + '\n')
        file.close()

    def generate_label(self, x):
        label = '_'.join([self.index, str(round(x, 2)), self.brightness, self.rotated_angle_x, self.rotated_angle_z])
        return label
    
    def generate_image_path(self):
        pass
    
    def set_rotate_angle_x(self, angle_x):
        self.rotated_angle_x = str(round(angle_x, 2))
        return angle_x
    
    def set_rotate_angle_z(self, angle_z):
        self.rotated_angle_z = str(round(angle_z, 2))
        return angle_z
    
    @staticmethod
    def input_check():
        while(True):
            check = input("Sind alle Info richtig? (J/N)")
            if check == 'J' or check == 'N':
                return check
            else:
                print("Falsche Eingabe, bitte versuch nochmal!")
