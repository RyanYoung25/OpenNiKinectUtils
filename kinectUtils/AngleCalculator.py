#! /usr/bin/env python

import numpy as np
import math 
import json
import sys

'''
Calculate the theta between two vectors a, and b. 
'''
def calculateTheta(a, b):
    dotProd = np.dot(a, b)        #a dot b
    aNorm = np.sqrt(np.dot(a, a))  # ||a||
    bNorm = np.sqrt(np.dot(b, b))  # ||b||
    #theta = arccos(a dot b / (||a||*||b||))
    return math.acos(dotProd/(aNorm*bNorm))


'''
Takes a list of xyz positions for two points, A and
B and generates the vector from A to B
'''
def generateVector(Apos, Bpos):
    #B minus A
    xComp = Bpos[0] - Apos[0]
    yComp = Bpos[1] - Apos[1]
    zComp = Bpos[2] - Apos[2]
    return np.array([xComp, yComp, zComp])


'''
Return a numpy vector of the coordinates for
the joint struture passed to to
'''
def getPointFromJoint(joint):
    posList = []
    posList.append(joint['pos']['x'])
    posList.append(joint['pos']['y'])
    posList.append(joint['pos']['z'])
    return np.array(posList)


'''
Math for calculating the Shoulder Yaw from
Matthew Wiese Star Project ITERO inspired
'''
def calculateYaw(EW, ES, joint):
        #finding z line cross shoulder elbow vector:
        avec1 = np.array([joint[0], joint[1], 2])
        bvec1 = ES
        zcross = np.cross(avec1, bvec1)
        EO = np.cross(EW, ES)
        EB = joint
        #finding cross of zcrossR and bvec
        zCross2 = np.cross(bvec1, zcross)
        #Normalize the elbow orientation and component add the elbow bend
        step1EO = EO / np.linalg.norm(EO)
        EO = step1EO + EB
        #Normalize the zCross2 vector to get the direction and component
        # add the elbow bend
        step1zCross2 = zCross2 / np.linalg.norm(zCross2)
        zCross2 = step1zCross2 + EB

        #Make the final vectors to calculate the angle between
        elbowToOrentation = generateVector(EB, EO)
        elbowToZ = generateVector(EB, zCross2)

        theta = calculateTheta(elbowToOrentation, elbowToZ) - 1.57
        return theta


def reduceRadians(angle):
    #For negative numbers use negative 2pi
    scaler = 2 * math.pi
    if(angle < 0.0):
        scaler = -1 * scaler

    #Mod division to get the remaining amount of rotation. Should normalize
    #things a little bit. Hopefully
    newAngle = (angle % scaler)
    return newAngle


def getPitch(vector):
    #acos(sqrt(1/(1+(vecta[0]/vecta[2]))**2))
    zComps = vector[0]/vector[2]
    zComps = zComps**2
    zComps = 1 + zComps
    beta = math.sqrt(1/zComps)
    beta = math.acos(beta)
    return beta


def getRoll(vector):
    #asin(vect[1])
    alpha = math.asin(vector[1])
    return alpha


def generateAngles(jsonString):
    #Make the jsonString into a dictionary to get the data
    dict = json.loads(jsonString)
    '''
    Order for joint angles: [REB, LEB, RSY, LSY, RSR, LSR, RSP, LSP]
    To calculate each angle, take the angle between the vectors formed
    by the specified points*:

    REB- RE-RW, RE-RS
    LEB- LE-LW, LE-LS
    RSY*-
    LSY*-
    RSR- RS-NK, RS-RE
    LSR- LS-NK, LS-LE
    RSP**- cross(RSR), NK-HD
    LSP**- cross(LSR), NK-HD

    *The shoulder yaws require a bit more complexity. First we must
    make a constant vector in the z direction and take the cross product of the two vectors
    specified. Then we need to cross that vector with the constant vector and get the angle between
    that vector and the vector from the cross product of the orginal vectors

    JOINTS = [
       0 'head',
       1 'neck',
       2 'torso',
       3 'left_shoulder',
       4 'left_elbow',
       5 'left_hand',
       6 'left_hip',
       7 'left_knee',
       8 'left_foot',
       9 'right_shoulder',
       10 'right_elbow',
       11 'right_hand',
       12 'right_hip',
       13 'right_knee',
       14 'right_foot'
        ]
    '''
    #Get the joint list
    jointList = dict["Joints"]

    #Create coordianate vectors for each joint point
    #Indices from above
    RH = getPointFromJoint(jointList[12])
    RW = getPointFromJoint(jointList[11])
    RE = getPointFromJoint(jointList[10])
    RS = getPointFromJoint(jointList[9])
    NK = getPointFromJoint(jointList[1])
    HD = getPointFromJoint(jointList[0])
    LH = getPointFromJoint(jointList[6])
    LS = getPointFromJoint(jointList[3])
    LE = getPointFromJoint(jointList[4])
    LW = getPointFromJoint(jointList[5])

    #Calculate Rich's angles
    #RS, RE
    vectorA = generateVector(RE, RS)
    vectorB = generateVector(LE, LS)


    #Generate the vectors that we will need to calculate the angles
    REBA = generateVector(RE, RW)
    REBB = generateVector(RE, RS)
    LEBA = generateVector(LE, LW)
    LEBB = generateVector(LE, LS)

    RSYA = generateVector(RE, RW)
    RSYB = generateVector(RE, RS)
    LSYA = generateVector(LE, LW)
    LSYB = generateVector(LE, LS)

    RSRA = generateVector(RS, RH)
    RSRB = generateVector(RS, RE)
    LSRA = generateVector(LS, LH)
    LSRB = generateVector(LS, LE)

    RSPA = np.cross(RSRA, RSRB)
    RSPB = generateVector(NK, HD)
    LSPA = np.cross(LSRA, LSRB)
    LSPB = generateVector(NK, HD)

    #Generate the angles
    REB = calculateTheta(REBA, REBB)
    LEB = calculateTheta(LEBA, LEBB)
    RSY = calculateYaw(RSYA, RSYB, RE)
    LSY = calculateYaw(LSYA, LSYB, LE)
    RSR = -1 * calculateTheta(RSRA, RSRB)  #getRoll(vectorA)
    LSR = calculateTheta(LSRA, LSRB)  #getRoll(vectorB)
    RSP = calculateTheta(RSPA, RSPB) #getPitch(vectorA)
    LSP = calculateTheta(LSPA, LSPB) #getPitch(vectorB)

    #Reduce the angles! Might not be needed after all. I thought there was a bug
    # but it was somewhere else.
    #REB = reduceRadians(REB)
    #LEB = reduceRadians(LEB)
    #RSY = reduceRadians(RSY)
    #LSY = reduceRadians(LSY)
    #RSR = reduceRadians(RSR)
    #LSR = reduceRadians(LSR)
    #RSP = reduceRadians(RSP)
    #LSP = reduceRadians(LSP)


    angles = np.array([REB, LEB, RSY, LSY, RSR, LSR, RSP, LSP])
    print len(angles)
    return angles


if __name__ == '__main__':
    #If ran alone try to convert a file containing json strings
    # of the appropriate data to a file of angle arrays
    filename = "Positions.log"
    output = "JointAngles.txt"

    if len(sys.argv) == 2:
        filename = sys.argv[1]
    elif len(sys.argv) == 3:
        output = sys.argv[2]
    elif len(sys.argv) > 3:
        print "Usage: ./AngleCalculator.py <Input File> <Output File>"

    #Open the input file
    fIn = open(filename, 'r')
    #Open the output file
    fOut = open(output, 'w')
    #Iterate through line by line of the input
    #creating a row vector for each line in the output
    for line in fIn:
        vector = generateAngles(line)
        fOut.write(str(vector) + "\n")
