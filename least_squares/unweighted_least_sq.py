#!/usr/bin/env python
"""
We have a single constant number, take 1000 measurements w/ gaussian noise added
Estimate via unweighted least squares the constant number
Print out the error percentage
"""

import numpy as np


num_meas = 1000
rand_const = np.random.randint(0,100)

measurements = np.random.normal(rand_const, 10, size=(num_meas,1))
ones = np.ones((num_meas, 1))
hl = np.linalg.pinv(ones)
estimate = np.dot(hl, measurements).item()
error = ( ( estimate-rand_const ) / rand_const )

print("Original Number: %d" % rand_const)
print("Estimate: %f" % estimate)
print("Error percentage: "+"{:.2%}".format(abs(error)))
