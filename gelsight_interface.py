import openvino as ov
import numpy as np
import cv2
from imutils.video import WebcamVideoStream
import os
import matplotlib.pyplot as plt
from smart_gripper_pkg.scripts import PKG_DIR
import time

class Gelsigth:
    def __init__(self, device="CPU", calib_model="midnet_v2", img_resolution=(255, 255)):
        """
        device: type string, device where inference is going to be processed ("CPU", "GPU").
        calib_model: type string, OpenVINO model used for calibration.
        img_resolution: type tuple (int, int), resolution of output image.
        """
        self.resolution = img_resolution
        self.__core = ov.Core()
        self.__ov_model = self.__core.read_model(model=calib_model+"_static"+".xml")
        self.__ov_compiled_model = self.__core.compile_model(self.__ov_model, device, config={"PERFORMANCE_HINT": "LATENCY"})
        
        #self.cap = cv2.VideoCapture(self.__find_cameras()[0])
        ##self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 255)
        ##self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 255)
        ##self.cap.set(cv2.CAP_PROP_FPS, 5)
        #self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap =  WebcamVideoStream(self.__find_cameras()[0]).start()

    def __del__(self):
        print("Resources released")
        self.cap.stop()
        del self.__ov_compiled_model


    def __find_cameras(self):
        # checks the first 10 indexes.
        index = 0
        arr = []
        if os.name == 'nt':
            cameras = find_cameras_windows()
            return cameras
        i = 10
        while i >= 0:
            print(i)
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:       
                command = 'v4l2-ctl -d ' + str(index) + ' --info'
                is_arducam = os.popen(command).read()
                if is_arducam.find('Arducam') != -1 or is_arducam.find('Mini') != -1:
                    arr.append(index)
                cap.release()
            index += 1
            i -= 1
        return arr

    def get_image(self):
        #for _ in range(5):
        #ret, img = self.cap.read()
        img = self.cap.read()
        #if not(ret):
        #    return None
        img = cv2.resize(img, self.resolution)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            #time.sleep(0.1)
        return img

    def get_forces(self):
        img = self.get_image()
        #plt.imshow(img)
        #plt.show()
        if not isinstance(img, np.ndarray):
            print("No img recieved. Check camera connection!")
            return None
        else:
            # Changing channel order to: (3, 255, 255)
            input_img = np.moveaxis(img, -1, 0)
            input_img = input_img[None, :, :, :]
            output = self.__ov_compiled_model(input_img)[0]
            fx, fy, fz = np.squeeze(output) 
            return np.abs(fx), np.abs(fy), np.abs(fz)


def main():
    sensor =  Gelsigth()
    fps = []
    try:
        while True:
            start = time.time()
            fx, fy, fz = sensor.get_forces()
            mag = np.sqrt(fx**2 + fy**2)
            end = time.time()
            hz = 1/(end-start)
            fps.append(hz)
            print(f"Magnitude: {np.round(mag,3)} \t Fz: {np.round(fz,3)} \t Hz: {np.round(hz,3)}")
    except KeyboardInterrupt:
        print("Releasing resources")
        print("Average Hz: {} \t Samples: {}".format(sum(fps)/len(fps), len(fps)) )
        del sensor


if __name__ == "__main__":
    main()

