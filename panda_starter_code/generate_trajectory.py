import numpy as np
import matplotlib.pyplot as plt
from sympy import Matrix, linsolve
from mpl_toolkits.mplot3d import Axes3D

def calculateCoefficients(startPos, interPos, endPos, startVel, endVel, tmid, tfinal):
    # Coefficient Matrix
    A = Matrix([
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, tmid, 0, 0, tmid**2, 0, 0, tmid**3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, tmid, 0, 0, tmid**2, 0, 0, tmid**3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, tmid, 0, 0, tmid**2, 0, 0, tmid**3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 2*tmid, 0, 0, 3*tmid**2, 0, 0, 0, 0, 0, -1, 0, 0, -2*tmid, 0, 0, -3*tmid**2, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 2*tmid, 0, 0, 3*tmid**2, 0, 0, 0, 0, 0, -1, 0, 0, -2*tmid, 0, 0, -3*tmid**2, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 2*tmid, 0, 0, 3*tmid**2, 0, 0, 0, 0, 0, -1, 0, 0, -2*tmid, 0, 0, -3*tmid**2],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, tmid, 0, 0, tmid**2, 0, 0, tmid**3, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, tmid, 0, 0, tmid**2, 0, 0, tmid**3, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, tmid, 0, 0, tmid**2, 0, 0, tmid**3],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, tfinal, 0, 0, tfinal**2, 0, 0, tfinal**3, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, tfinal, 0, 0, tfinal**2, 0, 0, tfinal**3, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, tfinal, 0, 0, tfinal**2, 0, 0, tfinal**3],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 2*tfinal, 0, 0, 3*tfinal**2, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 2*tfinal, 0, 0, 3*tfinal**2, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 2*tfinal, 0, 0, 3*tfinal**2],
        [0, 0, 0, 1, 0, 0, 2*tmid, 0, 0, 3*tmid**2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 2*tmid, 0, 0, 3*tmid**2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 2*tmid, 0, 0, 3*tmid**2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ])
    #[0, 0, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 0, 36, 36, 0],
    #[0, 0, 0, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 36]

    b = Matrix([
        [startPos[0]],
        [startPos[1]],
        [startPos[2]],
        [startVel[0]],
        [startVel[1]],
        [startVel[2]],
        [interPos[0]],
        [interPos[1]],
        [interPos[2]],
        [0],
        [0],
        [0],
        [interPos[0]],
        [interPos[1]],
        [interPos[2]],
        [endPos[0]],
        [endPos[1]],
        [endPos[2]],
        [endVel[0]],
        [endVel[1]],
        [endVel[2]],
        [0],
        [-0.5],
        [0.1]
        ])

    return linsolve((A,b))

if __name__ == '__main__':

    # Define the initial conditions
    startPos = np.array([0.328847, 0.458458, 0.846774])
    endPos = np.array([0.4328,-0.2, 0.15])

    tmid = 1.5
    tfinal = 0.1

    # Define the intermediate point
    interPos = np.array([0.4328,0.09829, 0.10])

    # Define the starting and ending velocities
    startVel = endVel = np.zeros(3)

    # Get the coefficients
    a = list(calculateCoefficients(startPos, interPos, endPos, startVel, endVel, tmid, tfinal))[0]

    print(a)

    # Initialize the figure and define the two sets of timesteps 
    fig = plt.figure()
    t1 = np.linspace(0, tmid, 50)
    t2 = np.linspace(tmid, tfinal, 50)

    # Solve for the x and y positions over the first segment
    u1x = a[0] + a[3]*t1 + a[6]*t1**2 + a[9]*t1**3
    u1y = a[1] + a[4]*t1 + a[7]*t1**2 + a[10]*t1**3
    u1z = a[2] + a[5]*t1 + a[8]*t1**2 + a[11]*t1**3

    # Solve for the x and y positions over the second segment
    u2x = a[12] + a[15]*t2 + a[18]*t2**2 + a[21]*t2**3
    u2y = a[13] + a[16]*t2 + a[19]*t2**2 + a[22]*t2**3
    u2z = a[14] + a[17]*t2 + a[20]*t2**2 + a[23]*t2**3

    # Concatenate the two segments together
    ux = np.concatenate((u1x, u2x[1:]))
    uy = np.concatenate((u1y, u2y[1:]))
    uz = np.concatenate((u1z, u2z[1:]))

    # Plot the trajectory
    f = plt.figure(1)
    ax = plt.axes(projection='3d')

    plt.title("EE Trajectories")
    #ax.plot3D(u1x, u1y, u1z)
    ax.plot3D(ux, uy, uz)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()

