# Define functions to import and posttreat the experimental data

import numpy as np
import csv


phi_L = [0,120,240]
Zcollision = [80,80,79.4]

def getExpData(filename):
    pressure = []
    force = []
    zcoord = []
    with open(filename, newline='') as csvfile:
        filereader = csv.reader(csvfile, delimiter=';', quotechar='|')
        flag = 0
        p0, f0, z0 = 0, 0, 0
        for row in filereader:
            pressure.append(np.round(float(row[0]), 3))
            force.append(np.round(float(row[1]), 3))
            zcoord.append(np.round(float(row[3]), 2))

    return pressure,force,zcoord

def postProcessData(forceI,Z0,pressure,force,zcoord):

    p0 = pressure[0]
    z0 = zcoord[0]
    f0 = force[0]
    pressurePP = [x-p0 for x in pressure]
    zcoordPP = [x - z0 for x in zcoord]
    pressureI = np.interp(forceI,force,pressure)
    zcoordI = np.interp(forceI,force,zcoord)
    pressurePP = pressureI
    zcoordPP = zcoordI

    return pressurePP,zcoordPP
