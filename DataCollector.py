#Create a CSV file that I can load into another program

import os
import csv

data_file = open("data.csv", 'w', newline='') #open the CSV I want to write to
writer = csv.writer(data_file) #going to write to it now

for dirpath, dirnames, filenames in os.walk("vehicles"):
    for filename in filenames:
        fullPath = os.path.join(dirpath, filename)
        writer.writerow([fullPath, 1])

for dirpath, dirnames, filenames in os.walk("non-vehicles"):
    for filename in filenames:
        fullPath = os.path.join(dirpath, filename)
        writer.writerow([fullPath, 0])