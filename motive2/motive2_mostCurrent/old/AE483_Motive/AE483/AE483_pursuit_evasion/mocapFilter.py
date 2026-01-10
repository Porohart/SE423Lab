import sys,os,time
import numpy as np



#################################################
#   get a position and attitude from mocap
#   calculate translational and angular velocity
#################################################
def mocapfilter(mocap_state, prev_state, prev_filter_state, dt): #{{{1
    # mocap_state, prev_state := (x,y,z,phi,theta,psi)
    # prev_filter_state = (x,y,z,vx,vy,vz,phi,theta,psi,p,q,r)

    #print mocap_state
    #print prev_state

    # status byte will represent things like whether or not we lost track
    # of the rigid body or not.
    # status of 0 ==> all is well
    #           1 ==> lost track
    status = 0

    # smooth positions
    alpha1 = 1.0 
    x = (alpha1) * mocap_state[0] + (1-alpha1)*prev_filter_state[0]
    y = (alpha1) * mocap_state[1] + (1-alpha1)*prev_filter_state[1]
    z = (alpha1) * mocap_state[2] + (1-alpha1)*prev_filter_state[2]
    phi = (alpha1) * mocap_state[3] + (1-alpha1)*prev_filter_state[3]
    theta = (alpha1) * mocap_state[4] + (1-alpha1)*prev_filter_state[4]
    psi = (alpha1) * mocap_state[5] + (1-alpha1)*prev_filter_state[5]

    # finite difference to get translational velocity
    alpha = 0.8
    #vx = (mocap_state[0] - prev_state[0]) / dt
    vx = (x - prev_filter_state[0]) / dt
    vx = (alpha) * vx + (1-alpha) * prev_filter_state[3]

    #vy = (mocap_state[1] - prev_state[1]) / dt
    vy = (y - prev_filter_state[1]) / dt
    vy = (alpha) * vy + (1-alpha) * prev_filter_state[4]

    #vz = (mocap_state[2] - prev_state[2]) / dt
    vz = (z - prev_filter_state[2]) / dt
    vz = (alpha) * vz + (1-alpha) * prev_filter_state[5]

    #phidot = (mocap_state[3] - prev_state[3]) / dt
    phidot = (phi - prev_filter_state[6]) / dt
    phidot = (alpha) * phidot + (1-alpha) * prev_filter_state[9]

    #thetadot = (mocap_state[4] - prev_state[4]) / dt
    thetadot = (theta - prev_filter_state[7]) / dt
    thetadot = (alpha) * thetadot + (1-alpha) * prev_filter_state[10]

    #psidot = (mocap_state[5] - prev_state[5]) / dt
    psidot = (psi - prev_filter_state[8]) / dt
    psidot = (alpha) * psidot + (1-alpha) * prev_filter_state[11]

    #filter_state = np.array([mocap_state[0], mocap_state[1], mocap_state[2],
                             #vx, vy, vz, 
                             #mocap_state[3], mocap_state[4], mocap_state[5],
                             #phidot, thetadot, psidot])
    filter_state = np.array([x, y, z,
                             vx, vy, vz, 
                             phi, theta, psi,
                             phidot, thetadot, psidot])

    #print "filter_state: "
    #print filter_state


    # if velocity is larger than threshold, then current state is an outlier we should reject
    #threshold = 5
    #v = np.linalg.norm([vx,vy,vz])
    #if v > threshold:
        #filter_state = prev_filter_state


    # if position is all zeros, then mocap has lost track of rigid body and we should estimate where the vehicle is
    threshold2 = 1e-6
    d = np.linalg.norm(mocap_state[0:3])
    if d < threshold2:
        status = 1
        #x_est = prev_filter_state[0] + prev_filter_state[3]*dt
        #y_est = prev_filter_state[1] + prev_filter_state[4]*dt
        #z_est = prev_filter_state[2] + prev_filter_state[5]*dt
        #vx_est = prev_filter_state[3]
        #vy_est = prev_filter_state[4]
        #vz_est = prev_filter_state[5]
        #phi_est = prev_filter_state[6] + prev_filter_state[9]*dt
        #theta_est = prev_filter_state[7] + prev_filter_state[10]*dt
        #psi_est = prev_filter_state[8] + prev_filter_state[11]*dt
        #p_est = prev_filter_state[9]
        #q_est = prev_filter_state[10]
        #r_est = prev_filter_state[11]
        #filter_state = np.array([x_est, y_est, z_est, vx_est, vy_est, vz_est,
                                 #phi_est, theta_est, psi_est, p_est, q_est, r_est])
#
    return (filter_state, status)










    #### OLD STUFF #########
        #t = np.linspace(0,(len(xprev)-1)*model.tau,len(xprev))
        #tck,tau = interpolate.splprep([xprevmat[0,:],xprevmat[1,:]],u=t,s=0.01)
        #deriv = interpolate.splev(t,tck,der=1)
        ##deriv2 = interpolate.splev(t,tck,der=2)
        ##y[3] = np.average(np.sqrt(deriv[0]**2 + deriv[1]**2))
        #y[3] = np.sqrt(deriv[0]**2. + deriv[1]**2)[-1]
        #if y[3] > 5:
            #y[3] = 5.0
        #print "y3 = ",y[3]
        ##y[3] = chop(y[3],0.0,3.0)
        ##kappa = (deriv[0] * deriv2[1] - deriv[1]*deriv2[0]) / ((deriv[0]**2 + deriv[1]**2)**(3./2))
        ##delta = np.arcsin(kappa * model.L)  # positive kappa ==> counterclockwise rotation
        ##delta[np.where(np.isnan(delta))[0]] = 0.
        ##delta = np.average(delta)
        ##y[4] = delta
        ##y[4] = mocapState[4]
        ##tck1 = interpolate.splrep(t,xprevmat[2,:],s=0.01)
        #tck1 = interpolate.splrep(t, xprevmat[2,:], s=0.01)
        #thetadot = interpolate.splev(t,tck1,der=1)
        #print "thetadot[-1] = ",thetadot[-1]
        #if y[3] > 0.05 and not np.isnan(thetadot[-1]):
            #y[4] = np.arcsin(thetadot[-1] * model.L / y[3])
        #else:
            #y[4] = 0.
        #if np.isnan(y[4]):
            #y[4] = 0.

def mocapfilter_obs(mocap_state, prev_state, prev_filter_state, dt): #{{{1
    # mocap_state, prev_state := (x,y,z,phi,theta,psi)
    # prev_filter_state = (x,y,z,vx,vy,vz,phi,theta,psi,p,q,r)

    # status byte will represent things like whether or not we lost track
    # of the rigid body or not.
    # status of 0 ==> lost track
    #           1 ==> all is well
    status = 1

    # smooth positions
    alpha1 = 1.0 
    x = (alpha1) * mocap_state[0] + (1-alpha1)*prev_filter_state[0]
    y = (alpha1) * mocap_state[1] + (1-alpha1)*prev_filter_state[1]
    z = (alpha1) * mocap_state[2] + (1-alpha1)*prev_filter_state[2]
    phi = (alpha1) * mocap_state[3] + (1-alpha1)*prev_filter_state[3]
    theta = (alpha1) * mocap_state[4] + (1-alpha1)*prev_filter_state[4]
    psi = (alpha1) * mocap_state[5] + (1-alpha1)*prev_filter_state[5]

    # finite difference to get translational velocity
    alpha = 0.8
    #vx = (mocap_state[0] - prev_state[0]) / dt
    vx = (x - prev_filter_state[0]) / dt
    vx = (alpha) * vx + (1-alpha) * prev_filter_state[3]

    #vy = (mocap_state[1] - prev_state[1]) / dt
    vy = (y - prev_filter_state[1]) / dt
    vy = (alpha) * vy + (1-alpha) * prev_filter_state[4]

    #vz = (mocap_state[2] - prev_state[2]) / dt
    vz = (z - prev_filter_state[2]) / dt
    vz = (alpha) * vz + (1-alpha) * prev_filter_state[5]

    #phidot = (mocap_state[3] - prev_state[3]) / dt
    phidot = (phi - prev_filter_state[6]) / dt
    phidot = (alpha) * phidot + (1-alpha) * prev_filter_state[9]

    #thetadot = (mocap_state[4] - prev_state[4]) / dt
    thetadot = (theta - prev_filter_state[7]) / dt
    thetadot = (alpha) * thetadot + (1-alpha) * prev_filter_state[10]

    #psidot = (mocap_state[5] - prev_state[5]) / dt
    psidot = (psi - prev_filter_state[8]) / dt
    psidot = (alpha) * psidot + (1-alpha) * prev_filter_state[11]

    #filter_state = np.array([mocap_state[0], mocap_state[1], mocap_state[2],
                             #vx, vy, vz, 
                             #mocap_state[3], mocap_state[4], mocap_state[5],
                             #phidot, thetadot, psidot])
    filter_state = np.array([x, y, z,
                             vx, vy, vz, 
                             phi, theta, psi,
                             phidot, thetadot, psidot])

    #print "filter_state: "
    #print filter_state


    # if velocity is larger than threshold, then current state is an outlier we should reject
    #threshold = 5
    #v = np.linalg.norm([vx,vy,vz])
    #if v > threshold:
        #filter_state = prev_filter_state


    # if position is all zeros, then mocap has lost track of rigid body and we should estimate where the vehicle is
    threshold2 = 1e-6
    d = np.linalg.norm(mocap_state[0:3])
    if d < threshold2:
        status = 0
        #x_est = prev_filter_state[0] + prev_filter_state[3]*dt
        #y_est = prev_filter_state[1] + prev_filter_state[4]*dt
        #z_est = prev_filter_state[2] + prev_filter_state[5]*dt
        #vx_est = prev_filter_state[3]
        #vy_est = prev_filter_state[4]
        #vz_est = prev_filter_state[5]
        #phi_est = prev_filter_state[6] + prev_filter_state[9]*dt
        #theta_est = prev_filter_state[7] + prev_filter_state[10]*dt
        #psi_est = prev_filter_state[8] + prev_filter_state[11]*dt
        #p_est = prev_filter_state[9]
        #q_est = prev_filter_state[10]
        #r_est = prev_filter_state[11]
        #filter_state = np.array([x_est, y_est, z_est, vx_est, vy_est, vz_est,
                                 #phi_est, theta_est, psi_est, p_est, q_est, r_est])
#
    return (filter_state, status)

