import numpy as np
import matplotlib.pyplot as plt
from sympy import Matrix, linsolve

def calculateCoefficients(startPos, interPos, endPos, startVel, endVel, tmid, tfinal):
    # Coefficient Matrix
    A = Matrix([
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, tmid, 0, tmid**2, 0, tmid**3, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, tmid, 0, tmid**2, 0, tmid**3, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 2*tmid, 0, 3*tmid**2, 0, 0, 0, -1, 0, -2*tmid, 0, -3*tmid**2, 0],
        [0, 0, 0, 1, 0, 2*tmid, 0, 3*tmid**2, 0, 0, 0, -1, 0, -2*tmid, 0, -3*tmid**2],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, tmid, 0, tmid**2, 0, tmid**3, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, tmid, 0, tmid**2, 0, tmid**3],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, tfinal, 0, tfinal**2, 0, tfinal**3, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, tfinal, 0, tfinal**2, 0, tfinal**3],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2*tfinal, 0, 3*tfinal**2, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2*tfinal, 0, 3*tfinal**2],
        [0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 36, 36]
        ])

    b = Matrix([
        [startPos[0]],
        [startPos[1]],
        [startVel[0]],
        [startVel[1]],
        [interPos[0]],
        [interPos[1]],
        [0],
        [0],
        [interPos[0]],
        [interPos[1]],
        [endPos[0]],
        [endPos[1]],
        [endVel[0]],
        [endVel[1]],
        [0],
        [0]
        ])

    return linsolve((A,b))

if __name__ == '__main__':

    # Define the initial conditions
    startPos = np.array([-5, 0])
    endPos = np.array([-1, 4])

    tmid = 3
    tfinal = 6

    # Define the intermediate point
    #interPos = np.array([5, -3])
    #interPos = np.array([-3, 3])
    interPos = np.array([-6, 3])

    # Define the starting and ending velocities
    startVel = endVel = np.zeros(2)

    # Get the coefficients
    a = list(calculateCoefficients(startPos, interPos, endPos, startVel, endVel, tmid, tfinal))[0]

    # Initialize the figure and define the two sets of timesteps 
    fig = plt.figure()
    t1 = np.linspace(0, 3, 50)
    t2 = np.linspace(3, 6, 50)

    # Solve for the x and y positions over the first segment
    u1x = a[0] + a[2]*t1 + a[4]*t1**2 + a[6]*t1**3
    u1y = a[1] + a[3]*t1 + a[5]*t1**2 + a[7]*t1**3

    # Solve for the x and y positions over the second segment
    u2x = a[8] + a[10]*t2 + a[12]*t2**2 + a[14]*t2**3
    u2y = a[9] + a[11]*t2 + a[13]*t2**2 + a[15]*t2**3

    # Concatenate the two segments together
    ux = np.concatenate((u1x, u2x[1:]))
    uy = np.concatenate((u1y, u2y[1:]))

    # Plot the trajectory
    plt.plot(ux, uy)
    plt.show()

