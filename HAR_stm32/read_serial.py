from datetime import datetime
import time
import serial
import time
from tensorflow import keras
import numpy as np

ser=serial.Serial("COM4",115200,timeout=1)

time.sleep(0.5)

while True:
    line=ser.readline()

    line=line.decode("utf")
    #print(line)       

    if line not in [None,"\n"]:
        time.sleep(1)
        print(line)          #time.sleep(10)
        
        #line.replace(','," ")
        list1=line.split()
        if (len(list1)>10):
            
            Ax=list1[1]
            Ay=list1[3]
            Az=list1[5]
            Gx=list1[7]
            Gy=list1[9]
            Gz=list1[11]
            Ax=Ax.replace("," ," ")
            Ay=Ay.replace("," ," ")
            Az=Az.replace("," ," ")
            Gx=Gx.replace("," ," ")
            Gy=Gy.replace("," ," ")
            Gz=Gz.replace(","," ")
            Ax=float(int(Ax)/1000000)
            Ay=float(int(Ay)/1000000)
            Az=float(int(Az)/1000000)
            Gx=float(int(Gx)/1000000)
            Gy=float(int(Gy)/1000000)
            Gz=float(int(Gz)/1000000)
            print(Ax,Ay,Az,Gx,Gy,Gz)
            if(Ax):
                if (type(Ax)==float):
                    new_model = keras.models.load_model('D:\\Downloads\\archive (8)\\model.h5')
                    y=np.array([[0,Ax,Ay,Az,Gx,Gy,Gz]])
                    y_pred=new_model.predict(y)
                    categories=["walking","running"]
                    print(categories[(np.argmax(y_pred))])

