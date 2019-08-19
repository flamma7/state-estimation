#!/usr/bin/env python
from __future__ import division
import numpy as np
from numpy.linalg import pinv
from numpy import dot

"""
We have a single constant number
We have one very noisy measurement source called "bad"
One more accurate measurement source called "good"
Estimate via weighted least squares the constant number
Print out the error percentage
"""

num_meas_total = 20
bad_sigma = 10
good_sigma = 2

rand_const = np.random.randint(0,100)

num_bad_meas = int(num_meas_total / 2)
# num_bad_meas = num_meas_total -1 # almost all bad measurements
num_good_meas = num_meas_total - num_bad_meas
bad_measurements = np.random.normal(rand_const, bad_sigma, size=(num_bad_meas,1))
# print(bad_measurements)
good_measurements = np.random.normal(rand_const, good_sigma, size=(num_good_meas,1))
measurements = np.concatenate(( bad_measurements, good_measurements))

H = np.ones((num_meas_total, 1))
bad_meas_covariances_vector = np.ones((num_bad_meas,1)) * ( bad_sigma ** 2)
# print( bad_meas_covariances_vector )
good_meas_covariances_vector = np.ones((num_good_meas, 1)) * ( good_sigma ** 2)
meas_covariance_vector = np.concatenate((bad_meas_covariances_vector, good_meas_covariances_vector))
# print( meas_covariance_vector)
meas_covariance = np.identity(num_meas_total) * meas_covariance_vector # broadcast
R = meas_covariance
# print( meas_covariance)

estimate = pinv( dot( dot( H.T, pinv(R)), H))
estimate = dot( estimate, H.T)
estimate = dot( estimate, pinv(R))
estimate = dot( estimate, measurements).item()
error = ( ( estimate-rand_const ) / rand_const )

print("Original Number: %d" % rand_const)
print("Measurements: \n"+str(measurements))
print("Estimate: %f" % estimate)
print("Error percentage: "+"{:.2%}".format(abs(error)))