from pyquaternion import Quaternion
import time
import math
import numpy as np


class Attitude():

    def __init__(self):
        pass

    def quatInv(self,quat):

        qconj = [ -quat[0], -quat[1], -quat[2], quat[3] ]
        qinv = qconj
        norm = np.sqrt( qconj[0]**2 + qconj[1]**2 + qconj[2]**2 + qconj[3]**2 )
        if norm != 0:

            for i in range(0,len(qconj)):

                qinv[i] = qinv[i] / norm

        return qinv

    def quatConj(self, quat):

        qconj = [ -quat[0], -quat[1], -quat[2], quat[3] ]

        return qconj

    def quatProd(self, quat1, quat2):

        qv1 = np.array( [ quat1[0], quat1[1], quat1[2] ] )
        qs1 = quat1[3]

        qv2 = np.array( [ quat2[0], quat2[1], quat2[2] ] )
        qs2 = quat2[3]

        qvf = qs1 * qv2 + qs2 * qv1 - np.cross( qv1, qv2 )
        qsf = qs1 * qs2 - np.dot( qv1, qv2 )

        quat = [ qvf[0], qvf[1], qvf[2], qsf ]

        return quat


    def quatToEuler(self, quat):

        norm = np.sqrt( quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2 )

        # Normalize
        if norm != 0:

            for i in range(0,len(quat)):
                quat[i] = quat[i] / norm

            A11 = quat[0]**2 - quat[1]**2 - quat[2]**2 + quat[3]**2
            A12 = 2 * quat[0] * quat[1] + 2 * quat[2] * quat[3]
            A13 = 2 * quat[0] * quat[2] - 2 * quat[1] * quat[3]
            A23 = 2 * quat[0] * quat[3] + 2 * quat[1] * quat[2]
            A33 = - quat[0]**2 - quat[1]**2 + quat[2]**2 + quat[3]**2

            phi = np.arctan2(A23, A33)
            theta = -np.arcsin(A13)
            psi = np.arctan2(A12, A11)

            euler = [phi, theta, psi]

        else:

            euler = [0, 0, 0]

        return euler

    def eulerToQuat(self, euler):

        phi = euler[0]
        theta = euler[1]
        psi = euler[2]

        qx = [ np.sin(phi/2), 0, 0, np.cos(phi/2) ]
        qy = [ 0, np.sin(theta/2), 0, np.cos(theta/2) ]
        qz = [ 0, 0, np.sin(psi/2), np.cos(psi/2) ]

        quat = self.quatProd(qx, self.quatProd(qy, qz))
        norm = np.sqrt( quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2 )

        # Normalize
        if norm != 0:

            for i in range(0,len(quat)):
                quat[i] = quat[i] / norm

        return quat


    def quatToAtt(self, quat):

        norm = np.sqrt( quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2 )

        if norm!=0:

            for i in range(0, len(quat)):
                quat[i] = quat[i] / norm

        qv = np.array([ [quat[0]], [quat[1]], [quat[2]] ])
        qv_T = qv.reshape(1, -1)
        qs = quat[3]

        rox = np.zeros((3, 3))
        rox[0, 1] = -qv[2, 0]
        rox[0, 2] = qv[1, 0]
        rox[1, 0] = qv[2, 0]
        rox[1, 2] = -qv[0, 0]
        rox[2, 0] = -qv[1, 0]
        rox[2, 1] = qv[0, 0]

        norm_qv_sq = quat[0]**2 + quat[1]**2 + quat[2]**2

        I = np.eye(3)

        A = (qs**2 - norm_qv_sq) * I + 2 * np.matmul(qv, qv_T) - 2 * qs * rox

        return A

    def attToEuler(self, A):

        phi = np.arctan2( A[1,2], A[2,2] )
        theta = - np.arcsin( A[0,2] )
        psi = np.arctan2( A[0,1], A[0,0] )

        euler = [phi, theta, psi]

        return euler

    def attToQuat(self, A):

        vector = [ A[0,0], A[1,1], A[2,2], A.trace() ]
        i = np.argmax(vector)

        if i==0:

            q = np.array([ [1 + A[0,0] - A[1,1] - A[2,2] ],\
                           [ A[0,1] + A[1,0] ],\
                           [ A[0,2] + A[2,0] ],\
                           [ A[1,2] - A[2,1] ] ])
        elif i==1:

            q = np.array([ [ A[1,0] + A[0,1] ],\
                           [ 1 + A[1,1] - A[2,2] - A[0,0] ],\
                           [ A[1,2] + A[2,1] ],\
                           [ A[2,0] - A[0,2] ] ])

        elif i==2:

            q = np.array([ [ A[2,0] + A[0,2] ],\
                           [ A[2,1] + A[1,2] ],\
                           [1 + A[2,2] - A[0,0] - A[1,1] ],\
                           [ A[0,1] - A[1,0] ] ])
        elif i==3:

            q = np.array([ [ A[1,2] - A[2,1] ],\
                           [ A[2,0] - A[0,2] ],\
                           [ A[0,1] - A[1,0] ],\
                           [ 1 + A[0,0] + A[1,1] + A[2,2] ] ])

        else:
            print("Error: check the attitude matrix")

        norm = np.sqrt( q[0, 0]**2 + q[1, 0]**2 + q[2, 0]**2 + q[3, 0]**2 )
        q1 = q[0, 0] / norm
        q2 = q[1, 0] / norm
        q3 = q[2, 0] / norm
        q4 = q[3, 0] / norm
        quat = [q1, q2, q3, q4]

        return quat


class Attitude_Hamilton():

    def quatProd(self, quat1, quat2):

        qv1 = np.array( [ quat1[1], quat1[2], quat1[3] ] )
        qs1 = quat1[0]

        qv2 = np.array( [ quat2[1], quat2[2], quat2[3] ] )
        qs2 = quat2[0]

        qvf = qs1 * qv2 + qs2 * qv1 + np.cross( qv1, qv2 )
        qsf = qs1 * qs2 - np.dot( qv1, qv2 )

        quat = [ qsf, qvf[0], qvf[1], qvf[2]]

        return quat


    def quatConj(self, quat):

        qconj = [ quat[0], -quat[1], -quat[2], -quat[3] ]

        return qconj

    def JPL2Hamilton(self, quat):

        q_ham = np.array( [ quat[3], quat[0], quat[1], quat[2] ] )

        return q_ham

    def quatInv(self, quat):

        qconj = self.quatConj(quat)
        qinv = qconj
        norm = np.sqrt( qconj[0]**2 + qconj[1]**2 + qconj[2]**2 + qconj[3]**2 )
        if norm != 0:

            for i in range(0,len(qconj)):

                qinv[i] = qinv[i] / norm

        return qinv

    def quatNormalize(self, quat):

        norm = np.sqrt( quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2 )
        if norm != 0:

            for i in range(0,len(quat)):

                quat[i] = quat[i] / norm

        return quat
