from __future__ import division
from scipy import integrate
import numpy as np

# simple example: dydx = x --> y = (1/2)*x^2

def diff_eq(x, y):
    print("calculating... " + str(y))
    return np.array([x])

x0, x1 = 0, 5
y0 = np.array([0])

r = integrate.RK45(diff_eq, x0, y0, x1)
# r = integrate.RK45(lambda x,y : np.array([x]), x0, y0, x1) # w/ lambda
while r.status == "running":
    status = r.step()
print(r.y)

# plt.show(y,x_seq)
# plt.show()