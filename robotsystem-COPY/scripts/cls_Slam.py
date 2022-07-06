#!/usr/bin/python3
# -*- coding: utf-8 -*-


from robotsystem import msg
import rospy
import numpy as np
import json
import matplotlib
import matplotlib.cm as cm
import matplotlib.pyplot as plt

import os
from ScanMatcher_OGBased import ScanMatcher
from OccupancyGrid import OccupancyGrid1
import math
import copy
from nav_msgs.msg import Odometry, MapMetaData
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray

from tf2_msgs.msg import TFMessage

from geometry_msgs.msg import Twist, Pose, TransformStamped, Transform, Quaternion, Vector3
from scout_msgs.msg import ScoutStatus

from collections import OrderedDict

matplotlib.use('Qt5Agg')


diretory = os.path.dirname(os.path.abspath(__file__))
os.chdir(diretory)



class slamdata:

    def __init__(self):
        
        self.initMapXLength, self.initMapYLength, self.unitGridSize, self.lidarFOV, self.lidarMaxRange = 50, 50, 0.02, np.pi, 10  # in Meters
        self.scanMatchSearchRadius, self.scanMatchSearchHalfRad, self.scanSigmaInNumGrid, self.wallThickness, self.moveRSigma,self. maxMoveDeviation, self.turnSigma, \
        self.missMatchProbAtCoarse, self.coarseFactor = 1.4, 0.25, 2, 5 * self.unitGridSize, 0.1, 0.25, 0.3, 0.15, 5
        
        # sensorData = self.readJson("./JSON/particleInit.json")
    
        # #print(sensorData)
        # numSamplesPerRev = len(sensorData[list(sensorData)[0]]['range'])  # Get how many points per revolution
        # initXY = sensorData[sorted(sensorData.keys())[0]]
        # numParticles = 10
        # ogParameters = [self.initMapXLength, self.initMapYLength, initXY, self.unitGridSize, self.lidarFOV, self.lidarMaxRange, numSamplesPerRev, self.wallThickness]
        # smParameters = [self.scanMatchSearchRadius, self.scanMatchSearchHalfRad, self.scanSigmaInNumGrid, self.moveRSigma, self.maxMoveDeviation, self.turnSigma, \
        #     self.missMatchProbAtCoarse, self.coarseFactor]
        #self.pf = ParticleFilter(numParticles, ogParameters, smParameters)

    def processSensorData(self, pf, sensorData, plotTrajectory = True):
        # gtData = readJson("../DataSet/PreprocessedData/intel_corrected_log") #########   For Debug Only  #############
        count = 0
        #plt.figure(figsize=(19.20, 19.20))
        for key in sorted(sensorData.keys()):
            count += 1
            print(count)
            pf.updateParticles(sensorData[key], count)
            if pf.weightUnbalanced():
                pf.resample()
                print("resample")

            #plt.figure(figsize=(19.20, 19.20))
            maxWeight = -1
            for particle in pf.particles:
                if maxWeight < particle.weight:
                    maxWeight = particle.weight
                    bestParticle = particle
                    plt.plot(particle.xTrajectory, particle.yTrajectory)

            xRange, yRange = [-13, 20], [-25, 7]
            ogMap = bestParticle.og.occupancyGridVisited / bestParticle.og.occupancyGridTotal
            xIdx, yIdx = bestParticle.og.convertRealXYToMapIdx(xRange, yRange)
            ogMap = ogMap[yIdx[0]: yIdx[1], xIdx[0]: xIdx[1]]
            ogMap = np.flipud(1 - ogMap)
            
            print(ogMap)
            plt.imsave('./Output/' + str(count).zfill(3) + '.png',ogMap)
            #plt.imshow(ogMap, cmap='gray', extent=[xRange[0], xRange[1], yRange[0], yRange[1]])
            #plt.savefig('./Output/' + str(count).zfill(3) + '.png')
            plt.close()

            #if count == 100:
            #     break
        maxWeight = 0
        for particle in pf.particles:
            particle.plotParticle()
            if maxWeight < particle.weight:
                maxWeight = particle.weight
                bestParticle = particle
        bestParticle.plotParticle()
        plt.close()
       
        

    #제이슨 파일 리드함수
    def readJson(self,jsonFile):

        print(jsonFile)

        with open(jsonFile, 'r') as f:
            input = json.load(f)
            return input['map']
        
    
   

class ParticleFilter:
    def __init__(self, numParticles, ogParameters, smParameters):
        self.numParticles = numParticles
        self.particles = []
        self.initParticles(ogParameters, smParameters)
        self.step = 0
        self.prevMatchedReading = None
        self.prevRawReading = None
        self.particlesTrajectory = []

    def initParticles(self, ogParameters, smParameters):
        for i in range(self.numParticles):
            p = Particle(ogParameters, smParameters)
            self.particles.append(p)

    def updateParticles(self, reading, count):
        for i in range(self.numParticles):
            self.particles[i].update(reading, count)


    def weightUnbalanced(self):
        self.normalizeWeights()
        variance = 0
        for i in range(self.numParticles):
            variance += (self.particles[i].weight - 1 / self.numParticles) ** 2
            #variance += self.particles[i].weight**2
        print(variance)
        if variance > ((self.numParticles - 1) / self.numParticles)**2 + (self.numParticles - 1.000000000000001) * (1 / self.numParticles)**2:
        #if variance > 2 / self.numParticles:
            return True
        else:
            return False

    def normalizeWeights(self):
        weightSum = 0
        for i in range(self.numParticles):
            weightSum += self.particles[i].weight
        for i in range(self.numParticles):
            self.particles[i].weight = self.particles[i].weight / weightSum

    def resample(self):
        # for particle in self.particles:
        #     particle.plotParticle()
        #     print(particle.weight)
        weights = np.zeros(self.numParticles)
        tempParticles = []
        for i in range(self.numParticles):
            weights[i] = self.particles[i].weight
            tempParticles.append(copy.deepcopy(self.particles[i]))
        resampledParticlesIdx = np.random.choice(np.arange(self.numParticles), self.numParticles, p=weights)
        for i in range(self.numParticles):
            self.particles[i] = copy.deepcopy(tempParticles[resampledParticlesIdx[i]])
            self.particles[i].weight = 1 / self.numParticles

class Particle:
    def __init__(self, ogParameters, smParameters):
        initMapXLength, initMapYLength, initXY, unitGridSize, lidarFOV, lidarMaxRange, numSamplesPerRev, wallThickness = ogParameters
        scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid,  moveRSigma, maxMoveDeviation,\
        turnSigma, missMatchProbAtCoarse, coarseFactor  = smParameters
        og = OccupancyGrid1(initMapXLength, initMapYLength, initXY, unitGridSize, lidarFOV, numSamplesPerRev, lidarMaxRange, wallThickness)
        sm = ScanMatcher(og, scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid, moveRSigma, maxMoveDeviation, turnSigma, missMatchProbAtCoarse, coarseFactor)
        self.og = og
        self.sm = sm
        self.xTrajectory = []
        self.yTrajectory = []
        self.weight = 1

    def updateEstimatedPose(self, currentRawReading):
        estimatedTheta = self.prevMatchedReading['theta'] + currentRawReading['theta'] - self.prevRawReading['theta']
        estimatedReading = {'x': self.prevMatchedReading['x'], 'y': self.prevMatchedReading['y'], 'theta': estimatedTheta,
                            'range': currentRawReading['range']}
        dx, dy = currentRawReading['x'] - self.prevRawReading['x'], currentRawReading['y'] - self.prevRawReading['y']
        estMovingDist = math.sqrt(dx ** 2 + dy ** 2)
        rawX, rawY, prevRawX, prevRawY = currentRawReading['x'], currentRawReading['y'], self.prevRawReading['x'], \
                                         self.prevRawReading['y']
        rawXMove, rawYMove = rawX - prevRawX, rawY - prevRawY
        rawMove = math.sqrt((rawX - prevRawX) ** 2 + (rawY - prevRawY) ** 2)

        if rawMove > 0.3:
            if self.prevRawMovingTheta != None:
                if rawYMove > 0:
                    rawMovingTheta = math.acos(rawXMove / rawMove)  # between -pi and +pi
                else:
                    rawMovingTheta = -math.acos(rawXMove / rawMove)
                rawTurnTheta = rawMovingTheta - self.prevRawMovingTheta
                estMovingTheta = self.prevMatchedMovingTheta + rawTurnTheta
            else:
                if rawYMove > 0:
                    rawMovingTheta = math.acos(rawXMove / rawMove)  # between -pi and +pi
                else:
                    rawMovingTheta = -math.acos(rawXMove / rawMove)
                estMovingTheta = None
        else:
            rawMovingTheta = None
            estMovingTheta = None

        return estimatedReading, estMovingDist, estMovingTheta, rawMovingTheta

    def getMovingTheta(self, matchedReading):
        x, y, theta, range = matchedReading['x'], matchedReading['y'], matchedReading['theta'], matchedReading['range']
        prevX, prevY = self.xTrajectory[-1], self.yTrajectory[-1]
        xMove, yMove = x - prevX, y - prevY
        move = math.sqrt(xMove ** 2 + yMove ** 2)
        if move != 0:
            if yMove > 0:
                movingTheta = math.acos(xMove / move)
            else:
                movingTheta = -math.acos(xMove / move)
        else:
            movingTheta = None
        return movingTheta

    def update(self, reading, count):
        if count == 1:
            self.prevRawMovingTheta, self.prevMatchedMovingTheta = None, None
            matchedReading, confidence = reading, 1
        else:
            currentRawReading = reading
            estimatedReading, estMovingDist, estMovingTheta, rawMovingTheta = self.updateEstimatedPose(currentRawReading)
            matchedReading, confidence = self.sm.matchScan(estimatedReading, estMovingDist, estMovingTheta, count, matchMax=False)
            self.prevRawMovingTheta = rawMovingTheta
            self.prevMatchedMovingTheta = self.getMovingTheta(matchedReading)
        self.updateTrajectory(matchedReading)
        self.og.updateOccupancyGrid(matchedReading)
        self.prevMatchedReading, self.prevRawReading = matchedReading, reading
        self.weight *= confidence

    def updateTrajectory(self, matchedReading):
        x, y = matchedReading['x'], matchedReading['y']
        self.xTrajectory.append(x)
        self.yTrajectory.append(y)

    def plotParticle(self):
        #plt.figure(figsize=(19.20, 19.20))
        plt.scatter(self.xTrajectory[0], self.yTrajectory[0], color='r', s=500)
        colors = iter(cm.rainbow(np.linspace(1, 0, len(self.xTrajectory) + 1)))
        for i in range(len(self.xTrajectory)):
            plt.scatter(self.xTrajectory[i], self.yTrajectory[i], color=next(colors), s=35)
        plt.scatter(self.xTrajectory[-1], self.yTrajectory[-1], color=next(colors), s=500)
        plt.plot(self.xTrajectory, self.yTrajectory)
        self.og.plotOccupancyGrid([-13, 20], [-25, 7], plotThreshold=False)

    




        


    