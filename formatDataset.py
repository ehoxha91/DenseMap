import argparse
import sys
import os
import numpy
import cv2

def read_file_list(filename):

    file = open(filename)
    data = file.read()
    lines = data.split("\n") 
    return lines

def savenew(lines):
    directory = os.getcwd()+"/"
    print(directory)
    for i in range(len(lines)):
        line = lines[i].split(' ')
        rgb = directory+str(line[1])
        newrgb = directory+"/rgb/"+str('r'+str(i)+'.png')
        depth = directory+str(line[3])
        newdepth = directory+"/depth/"+str('d'+str(i)+'.png')
        print(i)
        # print(rgb)
        # print(depth)
        os.rename(rgb, newrgb)
        os.rename(depth, newdepth)

    print("Renaming done!")

if __name__ == '__main__':
    association = read_file_list("associations.txt")
    savenew(association)
