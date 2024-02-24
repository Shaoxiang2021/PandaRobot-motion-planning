import cv2
import os

class LabelManager():
    def __init__(self):
        self.output_path = "workspace"
        self.szenario_index_dic = {"30_0_0_0": 1, "30_0_0.03_0": 2, "30_0_0.06_0": 3, "30_0.13_0_0": 4, "50_0_0_0": 5, "50_0_0.03_0": 6, "50_0_0.06_0": 7, "50_0.13_0_0": 8, "50_0_0_1": 9, "70_0_0_0": 10,
                               "70_0_0.03_0": 11, "70_0_0.06_0": 12, "70_0.13_0_0": 13, "100_0_0_0": 14, "100_0_0.03_0": 15, "100_0_0.06_0": 16, "100_0.13_0_0": 17}
        
        os.makedirs(self.output_path, exist_ok=True)

    def split_image(self, image_path, postion_index, szenario_index):
        image = cv2.imread(image_path)
        height, width, _ = image.shape
        split_height = int(height / 3)
        images = [image[:split_height, :, :], image[split_height:2*split_height, :, :], image[2*split_height:, :, :]]
        for i, image in enumerate(images):
            postion_index_start = (postion_index-1)*3 + 1
            image_name = "_".join([self.folder_name, str(szenario_index), str(postion_index_start+i)]) + ".jpg"
            output_image_path = os.path.join(self.save_path, image_name)
            cv2.imwrite(output_image_path, image) 

    def read_images_path(self, folder_path):
        self.image_paths = list()
        for root, dirs, files in os.walk(folder_path):
            for file in files:
                if file.endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    image_path = os.path.join(root, file)
                    self.image_paths.append(image_path)

    @staticmethod
    def return_position_index(postion):
        if postion == 0.43:
            return 1
        elif postion == 0.44 or postion == 0.45:
            return 2
        elif postion == 0.46:
            return 3
        elif postion == 0.48:
            return 4
        elif postion == 0.49 or postion == 0.50:
            return 5
        elif postion == 0.51:
            return 6
        else:
            print("error in create index for position!")
            return 0

    def generate_image_name(self, image_path):
        image_name = image_path.split('\\')[-1].split('_')
        _, _, _, _, x_position, brightness, angle_x, angle_z, blur, _ = image_name
        szenario = "_".join([brightness, angle_x, angle_z, blur])
        postion_index = self.return_position_index(float(x_position))
        szenario_index = self.szenario_index_dic[szenario]
        return szenario_index, postion_index
    
    def split_images(self, folder_path):
        self.folder_name = folder_path.split('\\')[-1]
        self.save_path = os.path.join(self.output_path, self.folder_name)
        os.makedirs(self.save_path, exist_ok=True)

        self.read_images_path(folder_path)

        for i, image_path in enumerate(self.image_paths):
            szenario_index, postion_index = self.generate_image_name(image_path)
            self.split_image(image_path, postion_index, szenario_index)

    @staticmethod
    def find_subfolder_path(folder_path):
        subfolder_paths = list()
        for root, dirs, files in os.walk(folder_path):
            for dir in dirs:
                subfolder_path = os.path.join(root, dir)
                subfolder_paths.append(subfolder_path)
        return subfolder_paths
    