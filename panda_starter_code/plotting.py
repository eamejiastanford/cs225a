import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Plot Panda Arm Parameters')

    parser.add_argument('--position', '-p', help='Plot the EE trajectory', action='store_true')
    parser.add_argument('--joints', '-q', help='Plot the joint trajectory', action='store_true')

    args = parser.parse_args()
    return args

def parse_ee_position(filename):
    x = []
    y = []
    z = []
    t = []
    with open(filename, 'r') as infile:
        for line in infile:
            line = line.rstrip()
            pos = list(filter(None, line.split(' ')))
            x.append(float(pos[0]))
            y.append(float(pos[1]))
            z.append(float(pos[2]))
            t.append(float(pos[3]))

    return x, y, z, t

def parse_phi(filename):
    x = []
    y = []
    z = []
    with open(filename, 'r') as infile:
        for line in infile:
            line = line.rstrip()
            joints = list(filter(None, line.split(' ')))
            x.append(float(joints[0]))
            y.append(float(joints[1]))
            z.append(float(joints[2]))

    return x, y, z

def parse_joint_trajectories(filename):
    joint = [[] for i in range(7)]
    with open(filename, 'r') as infile:
        for line in infile:
            line = line.rstrip()
            line = list(filter(None, line.split(' ')))
            joint[0].append(float(line[0]))
            joint[1].append(float(line[1]))
            joint[2].append(float(line[2]))
            joint[3].append(float(line[3]))
            joint[4].append(float(line[4]))
            joint[5].append(float(line[5]))
            joint[6].append(float(line[6]))

    return joint

if __name__ == "__main__":
    args = parse_args()

    if (args.position):
        x, y, z, t = parse_ee_position("../bin/panda_starter_code/trajectory.txt")
        xDes, yDes, zDes, tDes = parse_ee_position("../bin/panda_starter_code/des_trajectory.txt")

        print("Plotting EE position...")

        f = plt.figure(1)
        ax = plt.axes(projection='3d')

        plt.title("EE Trajectories")
        plt.ylabel("y")
        plt.xlabel("x")
        ax.plot3D(x, y, z)
        ax.plot3D(xDes, yDes, zDes)
        legend = ["Actial", "Desired"]
        plt.legend(legend, loc=1)
        plt.show()

    if (args.joints):
        x, y, z, t = parse_joint_trajectories("joints.txt")

        print("Plotting joint trajectories...")
        f = plt.figure(2)

        plt.title("Joint Angle Trajectories for Controller {}".format(problemNum))
        plt.ylabel("rad")
        plt.xlabel("time")
        plt.plot(t, q[0])
        plt.plot(t, q[1])
        plt.plot(t, q[2])
        plt.plot(t, q[3])
        plt.plot(t, q[4])
        plt.plot(t, q[5])
        plt.plot(t, q[6])
        legend = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        plt.legend(legend, loc=1)

        plt.show()

    '''
    elif (problemNum == 2):
        jointFileName = "{}joints.txt".format(problemNum)

        x, y, z, t = parse_trajectories(fileName)
        q = parse_joint_trajectories(jointFileName)

        desiredPos = {"x":[], "y":[], "z":[]}
        desiredQ = {"4upper":[], "4lower":[], "6upper":[], "6lower":[]}
        for i in t:
            #desiredPos["x"].append(-0.1)
            #desiredPos["y"].append(0.15)
            #desiredPos["z"].append(0.2)
            desiredPos["x"].append(-0.65)
            desiredPos["y"].append(-0.45)
            desiredPos["z"].append(0.7)
            desiredQ["4lower"].append(np.radians(-170))
            desiredQ["4upper"].append(np.radians(-30))
            desiredQ["6lower"].append(np.radians(0))
            desiredQ["6upper"].append(np.radians(210))

        print("Plotting...")

        f = plt.figure(1)

        plt.title("Operational Point Trajectories for Controller {}".format(problemNum))
        plt.ylabel("pos")
        plt.xlabel("time")
        plt.plot(t, x)
        plt.plot(t, y)
        plt.plot(t, z)
        plt.plot(t, desiredPos["x"])
        plt.plot(t, desiredPos["y"])
        plt.plot(t, desiredPos["z"])
        legend = ["X", "Y", "Z", "Desired X", "Desired Y", "Desired Z"]
        plt.legend(legend, loc=1)

        f = plt.figure(2)

        plt.title("Joint Angle Trajectories for Controller {}".format(problemNum))
        plt.ylabel("rad")
        plt.xlabel("time")
        plt.plot(t, q[3])
        plt.plot(t, q[5])
        plt.plot(t, desiredQ["4upper"])
        plt.plot(t, desiredQ["4lower"])
        plt.plot(t, desiredQ["6upper"])
        plt.plot(t, desiredQ["6lower"])
        legend = ["Joint 4", "Joint 6", "Joint 4 Upper", "Joint 4 Lower", "Joint 6 Upper", "Joint 6 Lower"]
        plt.legend(legend, loc=1)

        plt.show()

    elif (problemNum == 3):
        phiFileName = "{}phi.txt".format(problemNum)

        x, y, z, t = parse_trajectories(fileName)
        d1, d2, d3 = parse_phi(phiFileName)

        desiredPos = {"x":[], "y":[], "z":[]}
        desiredQ = {"4upper":[], "4lower":[], "6upper":[], "6lower":[]}
        for i in t:
            desiredPos["x"].append(0.6)
            desiredPos["y"].append(0.3)
            desiredPos["z"].append(0.5)

        print("Plotting...")

        f = plt.figure(1)

        plt.title("Operational Point Trajectories for Controller {}".format(problemNum))
        plt.ylabel("pos")
        plt.xlabel("time")
        plt.plot(t, x)
        plt.plot(t, y)
        plt.plot(t, z)
        plt.plot(t, desiredPos["x"])
        plt.plot(t, desiredPos["y"])
        plt.plot(t, desiredPos["z"])
        legend = ["X", "Y", "Z", "Desired X", "Desired Y", "Desired Z"]
        plt.legend(legend, loc=1)

        f = plt.figure(2)

        plt.title("DPhi")
        plt.ylabel("dphi")
        plt.xlabel("time")
        plt.plot(t, d1)
        plt.plot(t, d2)
        plt.plot(t, d3)
        legend = ["D1", "D2", "D3"]
        plt.legend(legend, loc=1)


        plt.show()

    else:

        x, y, z, t = parse_trajectories(fileName)
        phiFileName = "{}phi.txt".format(problemNum)
        vx, vy, vz = parse_phi(phiFileName)

        desiredPos = {"x":[], "y":[], "z":[]}
        maxV = {"pos":[], "neg":[]}
        for i in t:
            desiredPos["x"].append(0.6)
            desiredPos["y"].append(0.3)
            desiredPos["z"].append(0.4)
            maxV["pos"].append(0.1)
            maxV["neg"].append(-0.1)

        print("Plotting...")

        f = plt.figure(1)

        plt.title("Operational Point Trajectories for Controller {}".format(problemNum))
        plt.ylabel("pos")
        plt.xlabel("time")
        plt.plot(t, x)
        plt.plot(t, y)
        plt.plot(t, z)
        plt.plot(t, desiredPos["x"])
        plt.plot(t, desiredPos["y"])
        plt.plot(t, desiredPos["z"])
        legend = ["X", "Y", "Z", "Desired X", "Desired Y", "Desired Z"]
        plt.legend(legend, loc=1)

        f = plt.figure(2)

        plt.title("Velocity Trajectories for Controller {}".format(problemNum))
        plt.ylabel("velocity")
        plt.xlabel("time")
        plt.plot(t, vx)
        plt.plot(t, vy)
        plt.plot(t, vz)
        plt.plot(t, maxV["pos"])
        plt.plot(t, maxV["neg"])
        legend = ["V_X", "V_Y", "V_Z", "V Max +", "V Max -"]
        plt.legend(legend, loc=1)

        plt.show()
    '''


