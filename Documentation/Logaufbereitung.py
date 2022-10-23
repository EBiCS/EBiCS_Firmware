import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import filedialog

root = tk.Tk()
root.withdraw()

file_path = filedialog.askopenfilename() 
na=np.genfromtxt(file_path,delimiter=',')
na=na.astype(int)

x1=na[:,0:1]
x2=na[:,1:2]
x3=na[:,2:3]
y=na[:,3:4]
z=na[:,4:5]
#fig, axs = plt.subplots(3,sharex='all')

fig = plt.figure()

gs = fig.add_gridspec(3, hspace=0.1)
axs = gs.subplots(sharex=True)
plt.subplots_adjust(left=0.1, right=0.8, top=0.9, bottom=0.1)
fig.set_figwidth(10)
fig.suptitle('EBiCS log plotter')

axs[0].plot(x1, label="i")
axs[0].plot(x2, label="Byte i-7")
axs[0].plot(x3, label="Checksum Fault")
axs[0].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)
axs[1].plot(y,color='g', label="torque-signal")
axs[1].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)
axs[2].plot(z,color='r', label="wheeltime")
axs[2].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)

plt.show()
