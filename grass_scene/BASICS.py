with open ('D:/sirotalab/fabian_data/110_Day/BPositions_20200429-152831/metadata_20200429-152831.txt' ,"r") as infile:
    basic1 = infile.readlines()
import pandas as pd
basic = pd.read_csv('D:/sirotalab/fabian_data/110_Day/BPositions_20200429-152831/metadata_20200429-152831.txt')
for i in basic1:
    data = (i.split(' : '))
print (data)
