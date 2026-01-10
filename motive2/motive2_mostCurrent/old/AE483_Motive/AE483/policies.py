
import numpy as np
import scipy
from scipy.linalg import solve_discrete_are

def lqr_outer():

    dt = 0.033;
    m = 1
    g = 9.81

    Ac = np.zeros((6,6))
    Ac[0:3,3:6] = np.eye(3)
    Bc = np.zeros((6,3))
    Bc[3:6,:] = (1/m)*np.eye(3)

    A = np.eye(6) + dt*Ac
    B = dt*Bc
    Q = np.diag([20.0, 20.0, 10.0, 5.0, 5.0, 5.0])
    R = np.diag([1.0, 1.0, 1.0])

    S = solve_discrete_are(A,B,Q,R)
    K = np.dot(np.linalg.inv(np.dot(B.T, np.dot(S,B)) + R ), np.dot(B.T, np.dot(S,A)))
    #print "K = ",K
    return K
