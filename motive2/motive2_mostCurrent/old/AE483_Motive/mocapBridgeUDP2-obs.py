##################################################################
######## Configuration Options ###################################
##################################################################

# Drone Computer IP    (ifconfig on Linux machines)
#drone_ip = "128.174.193.149"
#drone_ip = "192.168.1.71" #RobotIP
#drone_ip = "127.0.0.1"  #IP of Mech computer
#drone_ip = 192.168.0.100
#drone_ip = "192.168.1.118"

# Drone Computer UDP Port   
#drone_port = 3500
            
# OptiTrack Computer IP address
# [Start->type 'cmd' in command window, type ipconfig]
opti_ip ="192.168.0.99"
#opti_ip ="128.174.193.140"
#opti_ip ="192.168.1.99"

# Data Port Set in Optitrack Streaming Properties
opti_port = 1511

# Multicast Interface in Optitrack Streaming Properties
multicastAdd = "239.255.42.99"

udp_port = 50000
#udp_ip = "128.174.192.46"  # Our normal corner workstation
#udp_ip = "128.174.192.73"  # another workstation  MECHTR23

address_list = ["192.168.1.99",
                "192.168.1.98",
                "192.168.1.97",
                "192.168.1.96",
                "192.168.1.95",
                "192.168.1.94",
                "192.168.1.93",
                "192.168.1.92",
                "192.168.1.91"]


##################################################################
##################################################################
# DO NOT EDIT ANYTHING BENEATH THIS LINE!!!!
##################################################################
##################################################################


import sys
import socket
import struct
import threading
import time
import serial
import Queue
import fmcore
import scipy.io
from policies import lqr_outer

from utils import *

def unPack(data): #{{{1
    trackableState = []
    byteorder='@'
    PacketIn = data
    major = 2
    minor = 0
    offset = 0
    # message ID, nBytes
    messageID, nBytes = struct.unpack(byteorder+'hh',PacketIn[offset:offset+4])
    offset += 4
    #print 'messageID=',messageID,' number of bytes=',nBytes
    if (messageID == 7):
        frameNumber,nMarkerSets = struct.unpack(byteorder+'ii',PacketIn[offset:offset+8])
        offset += 8
        #print 'framenumber=',frameNumber,' Markersets=', nMarkerSets
        i=nMarkerSets
        #print "nMarkerSets = ",nMarkerSets
        while (i > 0):
            #print "i = %d" % i
            ns = PacketIn[offset:offset+255]
            szNamelen=ns.find('\0')
            #print ns, szNamelen
            szName = struct.unpack(byteorder+str(szNamelen)+'s',PacketIn[offset:offset+szNamelen])[0]
            offset += szNamelen+1 # include the C zero char
            #print 'Modelname=',szName
            # markers
            nMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Markercount=',nMarkers
            j=nMarkers
            while (j>0):
                x,y,z = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
                offset += 12
                j=j-1
            i=i-1

        #unidentified markers
        nUMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'Unidentified Markercount=',nUMarkers
        i = nUMarkers
        while (i > 0):
            ux,uy,uz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
            offset += 12
            i=i-1

        # rigid bodies
        nrigidBodies = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        nr = nrigidBodies
        #print "nrigidBodies = %d" % nrigidBodies
        offset += 4
        #print 'Rigid bodies=',nrigidBodies
        while (nr > 0):
            ID = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            rbx,rby,rbz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
            offset += 12
            rbqx,rbqy,rbqz,rbqw = struct.unpack(byteorder+'ffff',PacketIn[offset:offset+16])
            offset += 16

            trackableState.append([ID,frameNumber, rbx, rby, rbz, rbqw, rbqx, rbqy, rbqz])  # our quaternion convention is scalar part first!

            #print 'ID=',ID
            #print 'pos:',rbx,rby,rbz
            #print 'ori:',rbqx,rbqy,rbqz,rbqw
            # associated marker positions
            nRigidMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Marker count=',nRigidMarkers
            md = []
            markerID = []
            markersize = []
            for i in range(0,nRigidMarkers):
                md.extend(struct.unpack(byteorder+'fff',PacketIn[offset:offset+12]))
                offset += 12
            if major >= 2:
                for i in range(0,nRigidMarkers):
                    markerID.append(struct.unpack(byteorder+'I',PacketIn[offset:offset+4])[0])
                    offset += 4
                for i in range(0,nRigidMarkers):
                    markersize.append(struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0])
                    offset += 4
                for i in range(0,nRigidMarkers):
                    pass
                    #print 'Marker ',i+1,' ID=',markerID[i],' markerData=',md[3*i],md[3*i+1],md[3*i+2],' markersize=',markersize[i]
            else:
                for i in range(0,nRigidMarkers):
                    pass
                    #print 'Marker ',i+1,' markerData=',md[3*i],md[3*i+1],md[3*i+2]

            # marker errors
            if major >= 2:
                markerError = struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0]
                offset += 4
                #print 'Mean marker error=',markerError

            nr = nr-1 # next rigid body

        #skeletons
        if (major==2 and minor>0) or major>2:
            nSkeletons = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Skeletons=',nSkeletons
            ns = nSkeletons
            while (ns > 0):
                skeletonID = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                offset += 4
                #print 'SkeletonID=',skeletonID
                nsrigidBodies = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                nsr = nsrigidBodies
                offset += 4
                #print 'Rigid body count=',nsrigidBodies
                while (nsr > 0):
                    IDsr = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                    offset += 4
                    srbx,srby,srbz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
                    offset += 12
                    srbqx,srbqy,srbqz,srbqw = struct.unpack(byteorder+'ffff',PacketIn[offset:offset+16])
                    offset += 16
                    #print 'ID=',IDsr
                    #print 'pos:',srbx,srby,srbz
                    #print 'ori:',srbqx,srbqy,srbqz,srbqw
                    # associated marker positions
                    nsRigidMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                    offset += 4
                    #print 'Marker count=',nsRigidMarkers
                    smd = []
                    smarkerID = []
                    smarkersize = []
                    for i in range(0,nsRigidMarkers):
                        smd.extend(struct.unpack(byteorder+'fff',PacketIn[offset:offset+12]))
                        offset += 12
                    for i in range(0,nsRigidMarkers):
                        smarkerID.append(struct.unpack(byteorder+'I',PacketIn[offset:offset+4])[0])
                        offset += 4
                    for i in range(0,nsRigidMarkers):
                        smarkersize.append(struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0])
                        offset += 4
                    for i in range(0,nsRigidMarkers):
                        pass
                        #print 'Marker ',i+1,' ID=',smarkerID[i],' markerData=',smd[3*i],smd[3*i+1],smd[3*i+2],' markersize=',markersize[i]

                    # marker errors
                    smarkerError = struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0]
                    offset += 4
                    #print 'Mean marker error=',smarkerError

                    nsr = nsr-1 # next rigidbody of skeleton

                ns = ns-1 # next skeleton

        #latency
        latency = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'latency=',latency
        #end of data tag
        eod = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'end of packet'
    
    return trackableState


if __name__ == '__main__': #{{{1

    frame_counter = 1
    
    que = Queue.Queue(100)
    
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    #s.bind((opti_ip, opti_port))
    s.bind(('',opti_port))
    
    # Initialize Multicast Socket
    mreq = struct.pack('4sl',socket.inet_aton(multicastAdd), socket.INADDR_ANY)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    # initialize UDP Socket
    udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    K = lqr_outer()
    xnom_outer = np.array([0,0,-1,0,0,0,0])
    g = 9.81
    mocap_state = []
    prev_state = []
    status = []
    rbid = []
    pymsg = []


    print "begin recv'ing from TrackingTools multicast"
    t0 = time.time()
    t_prev = t0

    while (True):
        #if frame_counter > 2:
            #break

        #pymsg = ''
        t = time.time()
        data = s.recv(10240)  
        if data:
            state = unPack(data)
            if not(state):
                continue

            #### number of rigid bodies:
            nRigidBodies = len(state)
            print "Number of Rigid Bodies: ", nRigidBodies

            if nRigidBodies > 10:
                state = state[0:11]
            
            # append virtual rigid bodies to pad 10
            while nRigidBodies < 10:
                state.append([0, 0, 0, 0, 0, 0, 0, 0, 0])
                nRigidBodies = nRigidBodies + 1


            if len(mocap_state) == 0:
                for i in range(nRigidBodies):
                    mocap_state.append([])
                    status.append(0)
                    rbid.append(0)
                    pymsg.append('')

            #pymsg = ''
            for i in range(nRigidBodies):

                rbid[i],fn,x,y,z,qw,qx,qy,qz = state[i]
                quat = Quaternion()
                quat.w, quat.x, quat.y, quat.z = qw, qx, qy, qz
                angles = optiquat2euler(quat.vec()) 
                R_bo = quat2rot(quat)
                ## rotate euler angles and position from mocap frame to "world" frame
                angles0 = EulerAngles(0, -np.pi/2, np.pi/2)
                R_wo = Rbw(angles0.vec()) 
                R_ow = np.transpose(R_wo)
                R_bw = np.dot(R_bo, R_ow)
                R_wb = np.transpose(R_bw)
    
                R_b1b2 = Rbw(angles0.vec())
                R_b1w = np.dot(R_b1b2, R_bw)
    
                R_wb1 = np.transpose(R_b1w)
                angles_bw = rot2euler(R_wb1) # FIXME this notation is confusing
                x_w, y_w, z_w = np.dot(R_wo, np.array([x,y,z]))
    
                mocap_state[i] = np.array([x_w, y_w, z_w, angles_bw[0], angles_bw[1], angles_bw[2]])

                threshold2 = 1e-9
                d = np.dot(mocap_state[i][0:3], mocap_state[i][0:3])
                if d < threshold2:
                    status[i] = 0
                else:
                    status[i] = rbid[i]

                pymsg[i] = struct.pack('ddddddB', mocap_state[i][0], mocap_state[i][1], mocap_state[i][2], mocap_state[i][3], mocap_state[i][4], mocap_state[i][5],status[i])



            if frame_counter % 100 == 0:
                #print map(ord,pymsg)
                for i in range(nRigidBodies):
                    print "mocap_state id: %d  status :%d = %.2f %.2f %.2f %.2f %.2f %.2f" % (rbid[i],status[i],mocap_state[i][0], mocap_state[i][1], mocap_state[i][2], mocap_state[i][3], mocap_state[i][4], mocap_state[i][5])

            # SORT the states so that they are in ascending order
            mydata = zip(rbid, pymsg)
            mydata_sorted = sorted(mydata)
            pymsg_sorted = [d[1] for d in mydata_sorted]
            pymsg_data = ''.join(pymsg_sorted)

            if frame_counter % 100 == 0:
                print "len(pymsg_data) = ", len(pymsg_data)
                #print map(ord,pymsg[0])
                #print map(ord,pymsg_data)

            # SEND filter_state over UDP
            for address in address_list:
                #print "len(pymsg_data) = ", len(pymsg_data)
                #print map(ord,pymsg_data)
                udpsock.sendto(pymsg_data, (address, udp_port))

            frame_counter = frame_counter + 1




