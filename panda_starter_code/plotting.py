import numpy as np
import math
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
    parser.add_argument('--velocity', '-v', help='Plot the EE velocity', action='store_true')
    parser.add_argument('--torques', '-t', help='Plot the joint torques', action='store_true')
    parser.add_argument('--joint_velocity', '-j', help='Plot the joint velocities', action='store_true')

    args = parser.parse_args()

    if noArgs:
        parser.print_help()
        sys.exit()

    return args

def parse_ee_position_velocity(filename):
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
    joint = [[] for i in range(8)]
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
            joint[7].append(float(line[7]))

    return joint

if __name__ == "__main__":

    # Ensures that the user has selected something to plot
    noArgs = False
    if len(sys.argv) == 1:
        noArgs = True

    args = parse_args()


    if (args.position):
        x, y, z, t = parse_ee_position_velocity("../bin/panda_starter_code/trajectory.txt")
        xDes, yDes, zDes, t = parse_ee_position_velocity("../bin/panda_starter_code/des_trajectory.txt")

        print("Plotting EE position...")

        f = plt.figure(1)
        ax = plt.axes(projection='3d')

        plt.title("EE Trajectories")
        plt.ylabel("y")
        plt.xlabel("x")
        ax.plot3D(x, y, z)
        ax.plot3D(xDes, yDes, zDes)
        legend = ["Actual", "Desired"]
        plt.legend(legend, loc=1)

        '''
        f = plt.figure(2)
        plt.title("EE trajectory")
        plt.ylabel("pos")
        plt.xlabel("time")
        plt.plot(x)
        plt.plot(y)
        plt.plot(z)
        legend = ["X", "Y", "Z"]
        plt.legend(legend, loc=1)
        '''

    if (args.joints):
        q = parse_joint_trajectories("../bin/panda_starter_code/joints.txt")

        print("Plotting joint trajectories...")

        q_bounds = {'0u':[], '0l':[], 
                '1u':[], '1l':[], 
                '2u':[], '2l':[], 
                '3u':[], '3l':[], 
                '4u':[], '4l':[], 
                '5u':[], '5l':[], 
                '6u':[], '6l':[]
                }
        for i in q[0]:
            q_bounds['0u'].append(2.7)
            q_bounds['0l'].append(-2.7)
            q_bounds['1u'].append(1.6)
            q_bounds['1l'].append(-1.6)
            q_bounds['2u'].append(2.7)
            q_bounds['2l'].append(-2.7)
            q_bounds['3u'].append(-0.2)
            q_bounds['3l'].append(-3.0)
            q_bounds['4u'].append(2.7)
            q_bounds['4l'].append(-2.7)
            q_bounds['5u'].append(3.6)
            q_bounds['5l'].append(0.2)
            q_bounds['6u'].append(2.7)
            q_bounds['6l'].append(-2.7)


        f = plt.figure(2)
        plt.title("Joints 1 & 2 Angle Trajectories")
        plt.ylabel("rad")
        plt.xlabel("time")
        plt.plot(q[0])
        plt.plot(q[1])
        plt.plot(q_bounds['0u'])
        plt.plot(q_bounds['0l'])
        plt.plot(q_bounds['1u'])
        plt.plot(q_bounds['1l'])
        legend = ["Joint 1", "Joint 2", "Joint 1 Upper", "Joint 1 Lower", "Joint 2 Upper", "Joint 2 Lower"]
        plt.legend(legend, loc=1)

        g = plt.figure(3)
        plt.title("Joints 3 & 4 Angle Trajectories")
        plt.ylabel("rad")
        plt.xlabel("time")
        plt.plot(q[2])
        plt.plot(q[3])
        plt.plot(q_bounds['2u'])
        plt.plot(q_bounds['2l'])
        plt.plot(q_bounds['3u'])
        plt.plot(q_bounds['3l'])
        legend = ["Joint 3", "Joint 4", "Joint 3 Upper", "Joint 3 Lower", "Joint 4 Upper", "Joint 4 Lower"]
        plt.legend(legend, loc=1)

        h = plt.figure(4)
        plt.title("Joints 5-7 Angle Trajectories")
        plt.ylabel("rad")
        plt.xlabel("time")
        plt.plot(q[4])
        plt.plot(q[5])
        plt.plot(q[6])
        plt.plot(q_bounds['4u'])
        plt.plot(q_bounds['4l'])
        plt.plot(q_bounds['5u'])
        plt.plot(q_bounds['5l'])
        plt.plot(q_bounds['6u'])
        plt.plot(q_bounds['6l'])
        legend = ["Joint 5", "Joint 6", "Joint 7", "Joint 5 Upper", "Joint 5 Lower", "Joint 6 Upper", "Joint 6 Lower", "Joint 7 Upper", "Joint 7 Lower"]
        plt.legend(legend, loc=1)

    if (args.velocity):
        vx, vy, vz, t = parse_ee_position_velocity("../bin/panda_starter_code/velocity.txt")

        v = []
        for i in range(len(vx)):
            v.append(math.sqrt(vx[i]**2 + vy[i]**2 + vz[i]**2))

        print("Plotting EE velocity...")

        f = plt.figure(5)

        plt.title("EE Velocity")
        plt.ylabel("m/s")
        plt.xlabel("time")
        plt.plot(t, vx)
        plt.plot(t, vy)
        plt.plot(t, vz)
        plt.plot(t, v)
        legend = ["Vx", "Vy", "Vz", "Total V"]
        plt.legend(legend, loc=1)

    if (args.torques):
        tau = parse_joint_trajectories("../bin/panda_starter_code/torques.txt")

        print("Plotting joint torques...")

        f = plt.figure(6)
        plt.title("Joint Torques")
        plt.ylabel("rad")
        plt.xlabel("time")
        plt.plot(tau[0])
        plt.plot(tau[1])
        plt.plot(tau[2])
        plt.plot(tau[3])
        plt.plot(tau[4])
        plt.plot(tau[5])
        plt.plot(tau[6])
        legend = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6", "Joint 7"]
        plt.legend(legend, loc=1)

    if (args.joint_velocity):
        q_v = parse_joint_trajectories("../bin/panda_starter_code/joint_velocity.txt")

        print("Plotting joint velocities...")

        f = plt.figure(7)
        plt.title("Joint Velocities")
        plt.ylabel("rad/s")
        plt.xlabel("time")
        plt.plot(q_v[7], q_v[0])
        plt.plot(q_v[7], q_v[1])
        plt.plot(q_v[7], q_v[2])
        plt.plot(q_v[7], q_v[3])
        plt.plot(q_v[7], q_v[4])
        plt.plot(q_v[7], q_v[5])
        plt.plot(q_v[7], q_v[6])
        legend = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6", "Joint 7"]
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


