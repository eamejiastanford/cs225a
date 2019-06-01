import numpy as np
import numpy.linalg
from numpy.linalg import eig, inv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def fitPolynomial(time, q, deg):
    z = np.polyfit(time, q, deg)
    return z


if __name__ == '__main__':
    n = 7
    # matrix = np.array([[1.96717, 0.62873, -1.11768, -1.96305, -0.00694139, 1.81579, -0.54889],
    #                     [1.89353, 0.628836, -1.12445, -2.1382, -0.178045, 1.81398, -0.548895],
    #                     [1.81511, 0.628146, -1.17783, -2.33654, -0.4661, 1.81367, -0.548895],
    #                     [1.67071, 0.416672, -1.22405, -2.52939, -0.891082, 1.72784, -0.548891],
    #                     [1.30877, 0.366178, -1.13545, -2.57107, -1.06904, 1.79045, -0.548891],
    #                     [0.965591, 0.368119, -1.0121, -2.57036, -1.26115, 1.78971, -0.548891],
    #                     [0.69847, 0.368119, -0.988169, -2.58332, -1.71343, 1.78976, -0.548891],
    #                     [0.392499, 0.368119, -0.988361, -2.43441, -1.94586, 1.92835, -0.548891]])

    matrix = np.array([[1.91575,0.925337,-1.22434,-1.9919,0.547301,2.22555,2.11608],
                        [1.40925,0.925337,-1.22434,-1.9919,0.547301,2.22555,2.11608],
                        [1.18678,0.925337,-1.22476,-1.9924,0.547546,2.22555,2.11608],
                        [0.811433,0.925727,-1.22476,-1.9924,0.547546,2.22555,2.11608],
                        [0.497473,0.930687,-1.22476,-1.99408,0.547546,2.22555,2.11608],
                        [0.125612,0.959772,-1.22468,-1.99446,0.547548,2.22555,2.11608],
                        [0.0785506,0.959772,-1.22468,-1.99446,0.54548,2.22555,2.11608]])

    np.reshape(matrix, (n,7))             
    time = np.linspace(0, 2, n)

    # Get the coefficients
    for i in range (2):
        ai = fitPolynomial(time, matrix[:,i], 1)
        print(ai)
    # Initialize the figure and define the two sets of timesteps
    # fig = plt.figure()

    # Solve for the x and y positions over the first segment
    # j1 = a1[0] + a1[1]*time + a1[2]*time + a1[3]*time
    # j2 = a2[0] + a2[1]*time + a2[2]*time + a2[3]*time
    # j3 = a3[0] + a3[1]*time + a3[2]*time + a3[3]*time
    # j4 = a4[0] + a4[1]*time + a4[2]*time + a4[3]*time
    # j5 = a5[0] + a5[1]*time + a5[2]*time + a5[3]*time
    # j6 = a6[0] + a6[1]*time + a6[2]*time + a6[3]*time
    # j7 = a7[0] + a7[1]*time + a7[2]*time + a7[3]*time
    # j8 = a8[0] + a8[1]*time + a8[2]*time + a8[3]*time
    # Solve for the x and y positions over the second segment

    # Plot the trajectory
    # f = plt.figure(1)
    # ax = plt.axes(projection='3d')

    # plt.title("EE Trajectories")
    # ax.plot3D(ux, uy, uz)
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.show()


    #new points:
   # 1.34851,0.456864,-1.35736,-2.24334,0.826839,2.54507,-1.21573
