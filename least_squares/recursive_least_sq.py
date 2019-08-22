#!/usr/bin/env python
from __future__ import division
import numpy as np
from numpy.linalg import pinv
from numpy import dot

"""
We have a single constant number and 100 measurements
Estimate via recursive unweighted least squares the constant number
Graph the error and covariance as we take more measurements
"""

num_meas_total = 100
sigma = 20
rand_const = np.random.randint(0,100)

x_hat = np.array(np.random.randint(0,100)).reshape((1,1)) # estimate mean
INFINITY = 1e9 # No certainty what our number is
P = np.array(INFINITY).reshape((1,1)) # uncertainty
H = np.ones((1,1)) # measurement matrix
R = np.array( sigma * sigma).reshape((1,1)) # meas noise covariance
measurements = np.array([])

for i in range(num_meas_total):
    meas = np.random.normal(rand_const, sigma, size=(1,1)).item()
    tmp = pinv( dot( dot(H,P), H.T) + R)
    K = dot( dot( P, H.T), tmp) # gain
    correction = meas - dot(H, x_hat)
    x_hat = x_hat + dot( K, correction)
    P = dot( np.eye(1) - dot(K, H), P)
    measurements = np.append(measurements, meas)

# error = ( ( estimate-rand_const ) / rand_const )
print("Original Number: %d" % rand_const)
print("Measurements: \n"+str(measurements))
print("Estimate: %f" % x_hat.item())
# print("Error percentage: "+"{:.2%}".format(abs(error)))