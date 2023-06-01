import cv2
import numpy as np
from pyueye import ueye
from data_manager import DataManager

class UeyeCamera():
    def __init__(self, exposure_ms=10):
        
        self.DataManager_Camera = DataManager()

        # init parameter
        self.width = 752
        self.height = 480
        self.bitspixel = 24 # for colormode = IS_CM_BGR8_PACKED
        self.exposure_ms = ueye.double(exposure_ms)

        # get data from camera and display
        self.lineinc = self.width * int((self.bitspixel + 7) / 8)

        # init camera
        self.hcam = ueye.HIDS(0)
        ret = ueye.is_InitCamera(self.hcam, None)
        self.DataManager_Camera.print_and_write_into_log(f"initCamera returns {ret}")

        # set color mode
        ret = ueye.is_SetColorMode(self.hcam, ueye.IS_CM_BGR8_PACKED)
        self.DataManager_Camera.print_and_write_into_log(f"SetColorMode IS_CM_BGR8_PACKED returns {ret}")

        # Belichtungszeit
        ret =ueye.is_Exposure(self.hcam, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, self.exposure_ms, ueye.sizeof(self.exposure_ms))
        self.DataManager_Camera.print_and_write_into_log(f"setExposure returns {ret}")

        # set region of interest
        self.rect_aoi = ueye.IS_RECT()
        self.rect_aoi.s32X = ueye.int(0)
        self.rect_aoi.s32Y = ueye.int(0)
        self.rect_aoi.s32Width = ueye.int(self.width)
        self.rect_aoi.s32Height = ueye.int(self.height)
        ueye.is_AOI(self.hcam, ueye.IS_AOI_IMAGE_SET_AOI, self.rect_aoi, ueye.sizeof(self.rect_aoi))
        self.DataManager_Camera.print_and_write_into_log(f"AOI IS_AOI_IMAGE_SET_AOI returns {ret}")

        # allocate memory
        self.mem_ptr = ueye.c_mem_p()
        self.mem_id = ueye.int()
        ret = ueye.is_AllocImageMem(self.hcam, self.width, self.height, self.bitspixel, self.mem_ptr, self.mem_id)
        self.DataManager_Camera.print_and_write_into_log(f"AllocImageMem returns {ret}")

        # set active memory region
        ret = ueye.is_SetImageMem(self.hcam, self.mem_ptr, self.mem_id)
        self.DataManager_Camera.print_and_write_into_log(f"SetImageMem returns {ret}")

        # continuous capture to memory
        ret = ueye.is_CaptureVideo(self.hcam, ueye.IS_DONT_WAIT)
        self.DataManager_Camera.print_and_write_into_log(f"CaptureVideo returns {ret}")

    def close(self):
        ret = ueye.is_StopLiveVideo(self.hcam, ueye.IS_FORCE_VIDEO_STOP)
        self.DataManager_Camera.print_and_write_into_log(f"StopLiveVideo returns {ret}")
        ret = ueye.is_ExitCamera(self.hcam)
        self.DataManager_Camera.print_and_write_into_log(f"ExitCamera returns {ret}")
    
    def save_image(self, filename):
        img = ueye.get_data(self.mem_ptr, self.width, self.height, self.bitspixel, self.lineinc, copy=True)
        img = np.reshape(img, (self.height, self.width, 3))
        cv2.imwrite(filename, img)
        self.DataManager_Camera.print_and_write_into_log("image is saved als name: " + filename)

    def show_image(self):
        img = ueye.get_data(self.mem_ptr, self.width, self.height, self.bitspixel, self.lineinc, copy=True)
        img = np.reshape(img, (self.height, self.width, 3))
        cv2.imshow('uEye Python Example (q to exit)', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def get_video_stream(self):
        while True:
            img = ueye.get_data(self.mem_ptr, self.width, self.height, self.bitspixel, self.lineinc, copy=True)
            img = np.reshape(img, (self.height, self.width, 3))
            cv2.imshow('uEye Python Example (q to exit)', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                # cleanup
                # self.close()
                break
