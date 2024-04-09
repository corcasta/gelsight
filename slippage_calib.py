from ..lsight_interface import Gelsigth
import pandas as pd
import numpy as np
import time 

def main():
    sensor =  Gelsigth()

    history_fx = []
    history_fy = []
    history_fz = []
    history_time = []

    #try:
    time_accum = 0
    while True:
        start = time.time()
        fx, fy, fz = sensor.get_forces()            
        end = time.time()

        time_accum += end-start

        history_fx.append(fx)
        history_fy.append(fy)
        history_fz.append(fz)
        history_time.append(time_accum)

        print("Time: {:.3f} \t Fx: {:.3f} \t Fy: {:.3f} \t Fz: {:.3f}".format(time_accum, fx, fy, fz))
        
        if time_accum >= 30:
            break
    #except KeyboardInterrupt:
    print("********* Creating Dataframe... *********")
    df = pd.DataFrame({
        "time": history_time,
        "fx": history_fx,
        "fy": history_fy,
        "fz": history_fz
    })
    
    csv_name = "test_slippage_data3.csv"
    df["mag"] = np.sqrt(df["fx"]**2 + df["fy"]**2 + df["fz"]**2)
    df["fx_avg10"] = df["fx"].rolling(10).mean()
    df["fy_avg10"] = df["fy"].rolling(10).mean()
    df["fz_avg10"] = df["fz"].rolling(10).mean()
    df["mag_avg10"] = df["mag"].rolling(10).mean()

    df["fx_avg20"] = df["fx"].rolling(20).mean()
    df["fy_avg20"] = df["fy"].rolling(20).mean()
    df["fz_avg20"] = df["fz"].rolling(20).mean()
    df["mag_avg20"] = df["mag"].rolling(20).mean()

    df["fx_avg30"] = df["fx"].rolling(30).mean()
    df["fy_avg30"] = df["fy"].rolling(30).mean()
    df["fz_avg30"] = df["fz"].rolling(30).mean()
    df["mag_avg30"] = df["mag"].rolling(30).mean()

    df["fx_avg50"] = df["fx"].rolling(50).mean()
    df["fy_avg50"] = df["fy"].rolling(50).mean()
    df["fz_avg50"] = df["fz"].rolling(50).mean()
    df["mag_avg50"] = df["mag"].rolling(50).mean()
    df.to_csv(f"dataset_slippage/{csv_name}", index=False)
    print("Done :)")

    print("Releasing resources")
    print("Average Hz: {} \t Samples: {}".format(sum(fps)/len(fps), len(fps)) )
    del sensor


if __name__ == "__main__":
    main()