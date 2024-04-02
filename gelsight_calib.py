import os
import sys
import csv
import cv2
import time
import glob
import NetFT
import pickle
import serial
import subprocess
import numpy as np
from datetime import datetime
import pandas as pd
import usb.core
import usb.backend.libusb1

SER = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)
RATIO = 3300.0  # 1mm=800
MAX_POS = 3300.0
MIN_POS = 0.0
position = []


class ATI:
    def __init__(self, ip='192.168.1.1', mean_x=0, mean_y=0, mean_z=0):
        self.__ft_sensor = NetFT.Sensor(ip)
        self.mean_x = mean_x
        self.mean_y = mean_y
        self.mean_z = mean_z

    def update_means(self, mean_x, mean_y, mean_z):
        self.mean_x = mean_x
        self.mean_y = mean_y
        self.mean_z = mean_z

    def getMeasurement(self):
        return self.__ft_sensor.getMeasurement()

    def tare(self):
        self.__ft_sensor.tare()

    def zero(self):
        self.__ft_sensor.zero()

    def get_small_measurement(self):
        ft_result = self.__ft_sensor.getMeasurement()
        ft_result[0] = round(ft_result[0]/1000000, 5)
        ft_result[1] = round(ft_result[1]/1000000, 5)
        ft_result[2] = round(ft_result[2]/1000000, 5)
        ft_result[3] = round(ft_result[3]/1000, 5)
        ft_result[4] = round(ft_result[4]/1000, 5)
        ft_result[5] = round(ft_result[5]/1000, 5)
        return ft_result[:3]

    #def save_image(self, img,addr, arg, num):
    #    address = addr +'/'+ str(num)+'_'+str(arg)+'.jpg'
    #    cv2.imwrite(address,img)

    def read_FT(self):
        fx = []
        fy = []
        fz = []
        # Read 100 samples from ATI sensor
        for i in range(100):   
            m_x, m_y, m_z = self.get_small_measurement()
            fx.append(m_x-self.mean_x)
            fy.append(m_y-self.mean_y)
            fz.append(m_z-self.mean_z)

        ati_arr = []
        df = pd.DataFrame({"fx": fx, "fy":fy, "fz":fz})
        for force in df.columns:
            counts, bins = np.histogram(df[force], bins=20)
            series_bin = pd.Series(bins)
            mean_bins = series_bin.rolling(2).mean().dropna()
            weights = counts/len(df[force])
            weighted_mean = (weights * mean_bins).sum()
            ati_arr.append(weighted_mean)

        self.tare()
        self.zero()
    
        # RETURN array with 3 weighted means of each force xyz
        return ati_arr

    """
    def save_FT(self, addr, arg, num):    
        address = addr +'/'+ str(num)+'_'+str(arg)+'.csv'
        for i in range(100):
            ati_arr = []
            with open(address, 'a') as csvfile:
                writer = csv.writer(csvfile)
                m_x, m_y, m_z = self.get_small_measurement()
                ati_arr.append(m_x-self.mean_x)
                ati_arr.append(m_y-self.mean_y)
                ati_arr.append(m_z-self.mean_z)
                writer.writerow(ati_arr)
        self.tare()
        self.zero()
    """

    """
    def read_FT(self):
        ati_arr = []
        # address = addr +'/'+ str(num)+'_'+str(arg)+'.csv'
        for i in range(100):
            m_x, m_y, m_z = self.get_small_measurement()
            ati_arr.append(m_x-self.mean_x)
            ati_arr.append(m_y-self.mean_y)
            ati_arr.append(m_z-self.mean_z)

        
        
        self.tare()
        self.zero()
        return np.mean(ati_arr[0::3])
    """

def rec():
    line = (SER.readline()).decode()
    if len(line) > 0:
        # print(line)
        return line

def init():
    SER.close()
    SER.open()
    if not SER.isOpen():
        print("Open port fail")
    else:
        SER.write("?R\r".encode())
        rec()

def pos():
    global position
    SER.write("?X\r".encode())
    data = (rec().split("+"))
    
    xpos = float(data[1]) / RATIO

    SER.write("?Y\r".encode())
    data = (rec().split("+"))
    ypos = float(data[1]) / RATIO

    SER.write("?Z\r".encode())
    data = (rec().split("+"))
    # print(data[1])
    zpos = float(data[1]) / RATIO

    position = [xpos, ypos, zpos]
    #print(f"Inside POS function: {position}")
    # return position

def check(axis, dis):
    if axis == "X":
        final_pos = position[0] + dis
        # print(final_pos)
        if final_pos > MAX_POS or final_pos < MIN_POS:
            return False
        else:
            return True
    if axis == "Y":
        final_pos = position[1] + dis
        if final_pos > MAX_POS or final_pos < MIN_POS:
            return False
        else:
            return True
    if axis == "Z":
        final_pos = position[2] + dis
        if final_pos > MAX_POS or final_pos < MIN_POS:
            return False
        else:
            return True

def move(axis, dis):
    if check(axis, dis):
        step = int(abs(dis) * RATIO)
        # print(dis)
        if dis >= 0:
            SER.write((axis + "+" + str(step) + "\r").encode())
            rec()
            pos()
        elif dis < 0:
            SER.write((axis + "-" + str(step) + "\r").encode())
            rec()
            pos()
    else:
        print("Exceed Travel move limit!")

def go2pos(axis, location):
    pos()
    if location >= MAX_POS or location < MIN_POS:
        print("Exceed Travel limit!")
    else:
        if axis == "X":
            dis = ((location - position[0]))
            move(axis, dis)
        elif axis == "Y":
            dis = ((location - position[1]))
            move(axis, dis)
        elif axis == "Z":
            dis = ((location - position[2]))
            move(axis, dis)

def find_cameras():
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

def find_cameras_windows():
    # checks the first 10 indexes.
    index = 0
    arr = []
    idVendor = 0xC45
    idProduct = 0x636D
    import usb.core
    import usb.backend.libusb1
    backend = usb.backend.libusb1.get_backend(
        find_library=lambda x: "libusb_win/libusb-1.0.dll"
    )
    dev = usb.core.find(backend=backend, find_all=True)
    # loop through devices, printing vendor and product ids in decimal and hex
    for cfg in dev:
        #print('Decimal VendorID=' + hex(cfg.idVendor) + ' & ProductID=' + hex(cfg.idProduct) + '\n')
        if cfg.idVendor == idVendor and cfg.idProduct == idProduct:
            arr.append(index)
        index += 1
    return arr

def resize_crop_mini(img, imgw, imgh):
    # resize, crop and resize back
    img = cv2.resize(img, (895, 672))  # size suggested by janos to maintain aspect ratio
    border_size_x, border_size_y = int(img.shape[0] * (1 / 7)), int(np.floor(img.shape[1] * (1 / 7)))  # remove 1/7th of border from each size
    img = img[border_size_x:img.shape[0] - border_size_x, border_size_y:img.shape[1] - border_size_y]
    img = img[:, :-1]  # remove last column to get a popular image resolution
    img = cv2.resize(img, (imgw, imgh))  # final resize for 3d
    return img


def save_image(cap, dataset_path, test_type, counter):
    imgw = 320
    imgh = 240
    #time.sleep(0.1)
    for _ in range(5):
        ret, frame = cap.read()
        if not(ret):
            return 0
        #frame = resize_crop_mini(frame, imgw, imgh)
        frame = cv2.resize(frame, (255, 255))
        time.sleep(0.1)
    cv2.imwrite(dataset_path + "/images/" + test_type + f"/{test_type}_img_{counter}.jpg", frame)
    #print("Image saved")

def main():
    ati = ATI('192.168.1.1')
    #print("DEBUG")

    cameras = find_cameras()
    print("DEBUG: ", len(cameras))
    cap = cv2.VideoCapture(cameras[0])
    WHILE_COND = cap.isOpened()
    print("AFTER DEMO")
    # set the format into MJPG in the FourCC format
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

    #time.sleep(5)

    ati_arr_x = []
    ati_arr_y = []
    ati_arr_z = []

    # Initial position NO CONTACT with sensor
    #print("DEBUG 2")
    init()
    pos()

    # HOME LOCATION
    go2pos("Z", 0)
    go2pos("X", 0)
    go2pos("Y", 0)

    #print("DEBUG 3")
    for i in range(1000):
        m_x, m_y, m_z = ati.get_small_measurement()
        ati_arr_x.append(m_x)
        ati_arr_y.append(m_y)
        ati_arr_z.append(m_z)
    #print("DEBUG 4")
    mean_x = np.mean(ati_arr_x)
    mean_y = np.mean(ati_arr_y)
    mean_z = np.mean(ati_arr_z)

    print(f"X mean: {mean_x}")
    print(f"Y mean: {mean_y}")
    print(f"Z mean: {mean_z}")

    ati.update_means(mean_x, mean_y, mean_z)

    print(f"Position: {position}")
    current_X = 0.0
    current_Y = 0.0
    current_Z = 0.0

    start_time = time.time()
    dz = 0.1
    dx = 0.2
    dy = 0.2

    
    ## Get the current time
    #current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    ## Define the CSV file name using the current time
    #folder_name = "/home/corcasta/Documents/ati/"+f"{current_time}"
    #folder_path = os.path.join(os.getcwd(), folder_name)
    ### Create the folder
    #os.makedirs(folder_path)


        
    test_type = "romboid_inv"
    dataset_path = "/home/corcasta/Documents/ati/dataset"
    csv_name = f"{test_type}_data.csv"

    #ati.save_FT(folder_name,'data',i)

    # ***** SQUARE PARAMS *********
    #max_x = 20.5              # in mm
    #max_y = 34.15 #22.15            # in mm
    #min_z = 1.4             # in mm
    #max_z = min_z + 0.9 #0.9     # in mm
    #step_size = 4         # in mm
    # ***** SQUARE PARAMS *********
    
    # ***** SPHERE PARAMS *********
    #max_x = 13              # in mm
    #max_y = 19.15            # in mm
    #min_z = 1.7             # in mm
    #max_z = min_z + 0.9     # in mm
    #step_size = 3         # in mm
    # ***** SPHERE PARAMS *********

    # ***** ROMBOID PARAMS *********
    max_x = 20.5              # in mm
    max_y = 30.15 #22.15            # in mm
    min_z = 1.8             # in mm
    max_z = min_z + 1 #0.9     # in mm
    step_size = 4         # in mm
    # ***** ROMBOID PARAMS *********

    ati.tare()
    ati.zero()
        
    x_locations = np.round(np.arange(0, max_x, step_size), 1)
    y_locations = np.round(np.arange(0, max_y, step_size), 1)
    z_locations = np.round(np.arange(min_z, max_z, 0.1), 1)
    y_shear_locs = np.round(np.arange(0.1, 0.4, 0.1), 1)
    print(f"x_locations: {x_locations}")
    print(f"y_locations: {y_locations}")
    print(f"z_locations: {z_locations}")
    print(f"y_shear_locs: {y_shear_locs}")


    """
    #****************************************************************************************************************
    #                       This is just to collect sample when the sensor is not being touched
    fx_list = []
    fy_list = []
    fz_list = []
    counter += 1
    while counter < 2000:
        fx, fy, fz = ati.read_FT()
        fx_list.append(round(fx, 3))
        fy_list.append(round(fy, 3))
        fz_list.append(round(fz, 3))
        print(f"Counter: {counter} \t --------> \t Fx: {fx} \t Fy: {fy} \t Fz: {fz}")
        save_image(cap, dataset_path, test_type, counter)
        
        if counter % 10 == 0:
            temp_df = pd.DataFrame({"fx": fx_list, "fy": fy_list, "fz": fz_list, "x": np.zeros(len(fx_list)), "y": np.zeros(len(fx_list)), "z": np.zeros(len(fx_list)), "x_shear": np.zeros(len(fx_list)), "y_shear": np.zeros(len(fx_list))})
            main_df = pd.concat([main_df, temp_df], ignore_index=True)
            main_df.to_csv(dataset_path + "/labels/" + test_type + "/" + csv_name)
            fx_list = []
            fy_list = []e
            fz_list = []

        counter += 1
    #****************************************************************************************************************
    """
    
    
    
    #****************************************************************************************************************
    main_df = pd.DataFrame(columns=["img_name", "fx", "fy", "fz", "x", "y", "z", "x_shear", "y_shear"])
    instance_counter = 0
    x_counter = 1
    for x in x_locations:
        print(f"******************** Next X loc: {x_counter} | {len(x_locations)} ********************")
        # This loop is for x-loc
        go2pos("Z", 0)
        go2pos("X", x)
        print("Loop X: ", x)

        x_counter += 1
        y_counter = 1 
        for y in y_locations:
            print(f"******************** Next Y loc: {y_counter} | {len(y_locations)} ********************")
            # This loop is for y-loc
            go2pos("Z", 0)
            go2pos("Y", y)
            print("Loop Y: ", y)

            # Clean lists every time Z entering loop
            #fx_list = []
            #fy_list = []
            #fz_list = []
            #x_list = []
            #y_list = []
            #z_list = []
            #x_shear_list = []
            #y_shear_list = []
            #names_list = []

            y_counter += 1
            z_counter = 1
            for z in z_locations:
                print(f"******************** Next Z loc: {z_counter} | {len(z_locations)} ********************")
                # This loop is for depth pressure
                go2pos("Z", z)
                print("Loop Z: ", z)
                
                # Appending values
                #fx_list.append(round(fx, 3))
                #fy_list.append(round(fy, 3))
                #fz_list.append(round(fz, 3))
                #x_list.append(x)
                #y_list.append(y)
                #z_list.append(z)
                #x_shear_list.append(0)
                #y_shear_list.append(0)
                #names_list.append(test_type + "_img_" + str(instance_counter) + ".jpg")

                # Reading image and saving it in dataset
                save_image(cap, dataset_path, test_type, instance_counter)
                fx, fy, fz = ati.read_FT()
                
                print(f"Counter: {instance_counter} \t --------> \t x: {x} \t y: {y} \t z: {z} \t --------> \t fx: {round(fx, 3)} \t fy: {round(fy, 3)} \t fz: {round(fz, 3)}")
                temp_df = pd.DataFrame({"img_name": [test_type + "_img_" + str(instance_counter) + ".jpg"], 
                                        "fx": [round(fx, 3)], 
                                        "fy": [round(fy, 3)], 
                                        "fz": [round(fz, 3)], 
                                        "x": [x], 
                                        "y": [y], 
                                        "z": [z], 
                                        "x_shear": [0], 
                                        "y_shear": [0]})
                main_df = pd.concat([main_df, temp_df], ignore_index=True)
                main_df.to_csv(dataset_path + "/labels/" + test_type + "/" + csv_name, index=False)
                instance_counter += 1

                for y_shear in np.round(np.arange(0.2, 1.1, 0.3), 1):
                    # This loop is for generating shear force in y-axis
                    y_total = round(y+y_shear, 1)
                    go2pos("Y", y_total)
                    print("Loop Y-shear: ", y_shear)
                    
                    # Ahow to know all possible combinationppending values
                    #fx_list.append(round(fx, 3))
                    #fy_list.append(round(fy, 3))
                    #fz_list.append(round(fz, 3))
                    #x_list.append(x)
                    #y_list.append(y)
                    #z_list.append(z)
                    #names_list.append(test_type + "_img_" + str(instance_counter) + ".jpg")
                    #x_shear_list.append(0)
                    #y_shear_list.append(y_shear)

                    # Reading image and saving it in dataset
                    save_image(cap, dataset_path, test_type, instance_counter)
                    fx, fy, fz = ati.read_FT()  
                    
                    print(f"Counter: {instance_counter} \t --------> \t x: {x} \t y_total: {y_total} \t z: {z} \t --------> \t fx: {round(fx, 3)} \t fy: {round(fy, 3)} \t fz: {round(fz, 3)}")
                    temp_df = pd.DataFrame({"img_name": [test_type + "_img_" + str(instance_counter) + ".jpg"], 
                                        "fx": [round(fx, 3)], 
                                        "fy": [round(fy, 3)], 
                                        "fz": [round(fz, 3)], 
                                        "x": [x], 
                                        "y": [y], 
                                        "z": [z], 
                                        "x_shear": [0], 
                                        "y_shear": [y_shear]})
                    main_df = pd.concat([main_df, temp_df], ignore_index=True)
                    main_df.to_csv(dataset_path + "/labels/" + test_type + "/" + csv_name, index=False)
                    instance_counter += 1

                go2pos("Z", 0)  # Go home in z-axis
                go2pos("Y", y)  # Go to current global y-loc
                go2pos("Z", z)  # Go to current z-loc

                for x_shear in np.round(np.arange(0.2, 1.1, 0.3), 1):
                    # This loop is for generating shear force in y-axis
                    x_total = round(x+x_shear, 1)
                    go2pos("X", x_total)
                    print("Loop X-shear: ", x_shear)
                    
                    # Appending values
                    #fx_list.append(round(fx, 3))
                    #fy_list.append(round(fy, 3))
                    #fz_list.append(round(fz, 3))
                    #x_list.append(x)
                    #y_list.append(y)
                    #z_list.append(z)
                    #names_list.append(test_type + "_img_" + str(instance_counter) + ".jpg")
                    #x_shear_list.append(x_shear)
                    #y_shear_list.append(0)

                    # Reading image and saving it in dataset
                    save_image(cap, dataset_path, test_type, instance_counter)
                    fx, fy, fz = ati.read_FT()
                    print(f"Counter: {instance_counter} \t --------> \t x_total: {x_total} \t y: {y} \t z: {z} \t --------> \t fx: {round(fx, 3)} \t fy: {round(fy, 3)} \t fz: {round(fz, 3)}")
                    temp_df = pd.DataFrame({"img_name": [test_type + "_img_" + str(instance_counter) + ".jpg"], 
                                        "fx": [round(fx, 3)], 
                                        "fy": [round(fy, 3)], 
                                        "fz": [round(fz, 3)], 
                                        "x": [x], 
                                        "y": [y], 
                                        "z": [z], 
                                        "x_shear": [x_shear], 
                                        "y_shear": [0]})
                    main_df = pd.concat([main_df, temp_df], ignore_index=True)
                    main_df.to_csv(dataset_path + "/labels/" + test_type + "/" + csv_name, index=False)
                    instance_counter += 1

                go2pos("Z", 0)  # Go home in z-axis
                go2pos("X", x)  # Go to current global y-loc
                z_counter += 1
            #temp_df = pd.DataFrame({"img_name": names_list, "fx": fx_list, "fy": fy_list, "fz": fz_list, "x": x_list, "y": y_list, "z": z_list, "x_shear": x_shear_list, "y_shear": y_shear_list})
            #main_df = pd.concat([main_df, temp_df], ignore_index=True)
            #main_df.to_csv(dataset_path + "/labels/" + test_type + "/" + csv_name, index=False)
            
            
    #****************************************************************************************************************
    
   
    """
    i = 0
    while True:
        # ati.tare()
        # ati.zero()
        user_input = input("press d to down, u to up: ")
        try:
            if user_input.lower() == 'a':
                move("X", 20.5)
                current_Z += dz
                print(ati.read_FT())
                print("DDDD")
            
            if user_input.lower() == 'b':
                move("Y", 30.15  )
                current_Z += dz
                print(ati.read_FT())

            if user_input.lower() == 'd':
                move("Z",dz)
                current_Z += dz
                print(current_Z)
                #ati.save_FT(folder_name,'data',i)
                print("DDDD")

            elif user_input.lower() == 'u':
                move("Z",-dz)
                current_Z -= dz
                print(current_Z)
                print("DDDD")
            
            elif user_input.lower() == 's':
                save_FT(folder_name,'data',i)
                ret,frame = cap.read()
                save_image(frame,folder_name,'data',i)
                i+=1

            elif user_input.lower() == 'q':
                break

        except KeyboardInterrupt:
            print('end')
    """

    go2pos("Z", 0)
    go2pos("X", 0)
    go2pos("Y", 0)
    SER.close()
    

if __name__ == "__main__":
    print("Starting")
    main()