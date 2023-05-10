import cv2
import numpy as np
from pyueye import ueye

class UeyeCamera():
    def __init__(self):
        # init parameter
        self.width = 752
        self.height = 480
        self.bitspixel = 24 # for colormode = IS_CM_BGR8_PACKED
        self.num = 0
        # get data from camera and display
        self.lineinc = self.width * int((self.bitspixel + 7) / 8)

        # init camera
        self.hcam = ueye.HIDS(0)
        ret = ueye.is_InitCamera(self.hcam, None)
        print(f"initCamera returns {ret}")

        # set color mode
        ret = ueye.is_SetColorMode(self.hcam, ueye.IS_CM_BGR8_PACKED)
        print(f"SetColorMode IS_CM_BGR8_PACKED returns {ret}")

        # set region of interest
        self.rect_aoi = ueye.IS_RECT()
        self.rect_aoi.s32X = ueye.int(0)
        self.rect_aoi.s32Y = ueye.int(0)
        self.rect_aoi.s32Width = ueye.int(self.width)
        self.rect_aoi.s32Height = ueye.int(self.height)
        ueye.is_AOI(self.hcam, ueye.IS_AOI_IMAGE_SET_AOI, self.rect_aoi, ueye.sizeof(self.rect_aoi))
        print(f"AOI IS_AOI_IMAGE_SET_AOI returns {ret}")

        # allocate memory
        self.mem_ptr = ueye.c_mem_p()
        self.mem_id = ueye.int()
        ret = ueye.is_AllocImageMem(self.hcam, self.width, self.height, self.bitspixel, self.mem_ptr, self.mem_id)
        print(f"AllocImageMem returns {ret}")

        # set active memory region
        ret = ueye.is_SetImageMem(self.hcam, self.mem_ptr, self.mem_id)
        print(f"SetImageMem returns {ret}")

        # continuous capture to memory
        ret = ueye.is_CaptureVideo(self.hcam, ueye.IS_DONT_WAIT)
        print(f"CaptureVideo returns {ret}")
    
    def save_image(self):
        img = ueye.get_data(self.mem_ptr, self.width, self.height, self.bitspixel, self.lineinc, copy=True)
        img = np.reshape(img, (self.height, self.width, 3))
        self.num = self.num + 1
        filename = "Bild{}.jpg".format(self.num)
        cv2.imwrite(filename, img)
        print("Bild gespeichert als " + filename)
