import openvino as ov
import numpy as np
import cv2
import os
from imutils.video import WebcamVideoStream
import matplotlib.pyplot as plt
from smart_gripper_pkg.scripts import PKG_DIR
import time

class Gelsight:
    def __init__(self, device="CPU", calib_model="midnet_s2_v3", img_resolution=(255, 255)):
        """
        device: type string, device where inference is going to be processed ("CPU", "GPU").
        calib_model: type string, OpenVINO model used for calibration.
        img_resolution: type tuple (int, int), resolution of output image.
        """
        # ************* Force Model Params *************
        self.resolution = img_resolution
        self.__core = ov.Core()
        self.__ov_model = self.__core.read_model(model=calib_model+"_static"+".xml")
        self.__ov_compiled_model = self.__core.compile_model(self.__ov_model, device, config={"PERFORMANCE_HINT": "LATENCY"})
        self.cap =  WebcamVideoStream(self.__find_cameras()[0]).start()
        self.FX_B, self.FY_B, self.FZ_B = self.__warmup()
        # ************* Force Model Params *************

        # ************* Slippage Model Params *************
        self.__torch_slippage_model = torch.jit.load("slippage_jit_model_v0.pt")
        # ************* Slippage Model Params *************

    def detect_slippage(self, input_sequence):
        """
        input_sequence: type torch.tensor with shape (1, 100, 4) == (batch_size, timesteps, features)
        returns: index of highest probable class (0 or 1) == (static or slipped)
        """
        output = self.__torch_slippage_model(input_sequence)
        ret, predicted_label = torch.max(output.data, 1)
        return predicted_label.detach().numpy()


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

    def __remove_bias(self, fx, fy, fz):
        fx = fx-self.FX_B
        fy = fy-self.FY_B
        fz = fz-self.FZ_B
        
        if fx < 0.0:
            fx = 0.0

        if fy < 0.0:
            fy = 0.0

        if fz < 0.0:
            fz = 0.0
            
        return fx, fy, fz

    def __warmup(self):
        print("Warming up Force Sensor...")
        history_fx = []
        history_fy = []
        history_fz = []
        time_accum = 0
        while True:
            start = time.time()
            fx, fy, fz = self.get_forces(remove_bias=False)            
            end = time.time()
            time_accum += end-start
            # Im using it to calculate an average of the bias
            history_fx.append(round(fx,3))
            history_fy.append(round(fy,3))
            history_fz.append(round(fz,3))
            if time_accum >= 5:
                break
        fx_b = np.mean(history_fx)
        fy_b = np.mean(history_fy)
        fz_b = np.mean(history_fz)
        return fx_b, fy_b, fz_b


    def __to_numpy(self, tensor):
        return tensor.detach().cpu().numpy()[None, :, :] if tensor.requires_grad else tensor.cpu().numpy()[None,:,:]


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

    def get_forces(self, remove_bias=True):
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
            fx, fy, fz = np.abs(fx), np.abs(fy), np.abs(fz)
            if remove_bias:
                fx, fy, fz = self.__remove_bias(fx, fy, fz)
            return fx, fy, fz


def main():
    sensor =  Gelsigth()
    fps = []
    try:
        while True:
            start = time.time()
            fx, fy, fz = sensor.get_forces()
            mag = np.sqrt(fx**2 + fy**2 + fz**2)
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

