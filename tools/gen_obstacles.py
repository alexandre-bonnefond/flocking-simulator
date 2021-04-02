#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 29 11:57:42 2020

@author: alexandre
"""
import time
from pathlib import Path
import numpy as np 
import matplotlib.pyplot as plt
from os import path
from os import mkdir



def gen_obstacles(distrib = "uniform", num_obst = 30, radius_obst = 500, std = 100):
    
    if distrib == "uniform":
        coordX = np.random.uniform(ArenaCenterX - radius, ArenaCenterX + radius, num_obst)
        coordY = np.random.uniform(ArenaCenterY - radius, ArenaCenterY + radius, num_obst)
        
     
    elif distrib == "normal":
        coordX = np.random.normal(ArenaCenterX, radius/2, num_obst)
        coordY = np.random.normal(ArenaCenterY, radius/2, num_obst)
    
    coords = np.vstack((coordX, coordY)).T
    size_obst = np.random.normal(radius_obst, std, num_obst)
    if check_intersection(coords, size_obst) == True:
        coords, size_obst = gen_obstacles(distrib, num_obst, radius_obst, std)
        
    
    return coords, size_obst
    
    
def check_intersection(centers, sizes):
    assert(centers.shape == (len(sizes), 2))
    intersect = False
    for i in range(len(sizes)):
        point_i = get_edges(centers[i, :], sizes[i])
        for j in range(i+1, len(sizes)):
            point_j = get_edges(centers[j, :], sizes[j])
            if not(point_i[0, 0] > point_j[1, 0] or point_j[0, 0] > point_i[1, 0]):
                if not(point_i[1, 1] > point_j[2, 1] or point_j[1, 1] > point_i[2, 1]):
                    print("intersecting obstacles")
                    intersect = True
                    return intersect
            
def get_edges(center, size):
    points_list = np.zeros((4, 2))  #only square shapes 
    
    points_list[0, 0] = center[0] - size   # bottom left point
    points_list[0, 1] = center[1] - size
    
    points_list[1, 0] = center[0] + size   # bottom right point
    points_list[1, 1] = center[1] - size

    points_list[2, 0] = center[0] + size   # top right point
    points_list[2, 1] = center[1] + size

    points_list[3, 0] = center[0] - size   # top left point
    points_list[3, 1] = center[1] + size   
    
    return points_list     


if __name__ == '__main__':
    
    strings = time.strftime("%m,%d,%H,%M")
    t = strings.split(',')
    
    data_folder = Path("../parameters/")
    file_to_open = data_folder / "flockingparams.dat"
    with open(file_to_open) as f:
        lines = f.readlines()
        for line in lines:
            if "ArenaRadius" in line:
                line = line.split('=')
                radius = float(line[1])
            elif "ArenaCenterX" in line:
                line = line.split('=')
                ArenaCenterX = float(line[1])
            elif "ArenaCenterY" in line:
                line = line.split('=')
                ArenaCenterY = float(line[1])
            elif "ArenaShape" in line:
                line = line.split('=')
                ArenaShape = float(line[1])
    f.close()
    radius -= 200 #avoid obstacles on the edges of the arena
    distrib = input("Which distribution of the obstacles on the map (uniform or normal) ? ")
    numb_obst = input("How many obstacles ? ")
    numb_obst = int(numb_obst)
    radius_of_obst = input("What is the radius of the obstacle (in cm) ? ")
    radius_of_obst = float(radius_of_obst)
    std = input("What is the standard deviation of the radius of the obstacles ? ")
    std = float(std)
    filename = "obstacles_" + distrib + "_" + str(numb_obst) + "_" + t[0] + "_" + t[1] + "_"\
                                + t[2] + "_" + t[3] + ".default"    
    
    if not path.isdir("../obstacles"):
        mkdir("../obstacles")
    newpath = Path("../obstacles")     
    file_to_create = newpath / filename                       
    f= open(file_to_create, "w+")
    f.write("[init]\n\n")
    f.write("angle=0\n\n")
    f.write("[obstacles]\n\n\n")
    
    coords, size_obst = gen_obstacles(distrib, numb_obst, radius_of_obst, std)
    plt.figure()
    for i in range(numb_obst):
        points = get_edges(coords[i, :], size_obst[i])
        plt.plot(points[:, 0], points[:, 1])
        f.write("obst" + str(i) + ".point=" + str(points[0, 0]) + " " + str(points[0, 1]) + "\n")
        f.write("obst" + str(i) + ".point=" + str(points[1, 0]) + " " + str(points[1, 1]) + "\n")
        f.write("obst" + str(i) + ".point=" + str(points[2, 0]) + " " + str(points[2, 1]) + "\n")
        f.write("obst" + str(i) + ".point=" + str(points[3, 0]) + " " + str(points[3, 1]) + "\n\n\n\n")
    plt.show()
    f.close()










