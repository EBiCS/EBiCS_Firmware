import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import filedialog
import csv
import re
clean_vars = []    
special_characters = "!@#$%^&*()_-+=<>,./?;:'\"[]{}\\|`~"
translation_table = str.maketrans("", "", special_characters)

root = tk.Tk()
root.withdraw()

file_path = filedialog.askopenfilename()
#falsche Spaltenanzahl rausfiltern
with open(file_path, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    
    for row in reader:
        # clean up your file here and append it to the list
        if len(row)==7:
            OK=1
            #Zeilen mit korupten Daten rausschmeißen
            for column in row:
                try:
                    myvar= int(column)
                    
                except:
                    OK=0
                    print("Bäh")
                #clean_vars.append([char for char in column if char.isalnum()])
            if OK:
                clean_vars.append([char for char in row if char])
    
#na=np.genfromtxt(file_path,delimiter=',',invalid_raise=False)

na = np.asarray(clean_vars)

na=na.astype(int)
#print (na)
x1=na[:,2:3]
x2=na[:,4:5]
x3=na[:,5:6]
y=na[:,4:5]
z=na[:,3:4]
z1=na[:,3:4]
#fig, axs = plt.subplots(3,sharex='all')

fig = plt.figure()

gs = fig.add_gridspec(3, hspace=0.1)
axs = gs.subplots(sharex=True)
plt.subplots_adjust(left=0.1, right=0.8, top=0.9, bottom=0.1)
fig.set_figwidth(10)
fig.suptitle('EBiCS log plotter')

axs[0].plot(x1, label="i_q ist")
axs[0].plot(x2, label="i_q soll")
axs[0].plot(x3, label="Pedal Position")
#axs[2].plot(z1, label="i_q")
axs[0].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)
axs[1].plot(y,color='g', label="Wheeltime")
axs[1].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)
axs[2].plot(z,color='r', label="Torque")
#axs[2].plot(z1,color='b', label="i_q")
axs[2].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)

plt.show()
