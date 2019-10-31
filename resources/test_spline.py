#! /usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import sys

global knot_vector
global p
global n_points

def basis(i,n,x):
  global knot_vector
  global p
  global n_points
  
  # left end
  if n == p and i == 0 and x < knot_vector[0]:
    return 1
    
  # right end
  if n == p and i == n_points-1 and x >= knot_vector[-1]:
    return 1
    
    
  if n == 0:
    if knot_vector[i] <= x < knot_vector[i+1]:
      return 1
    return 0
  else:
    # compute first factor
    nominator1 = (x - knot_vector[i])
    denominator1 = (knot_vector[i+n] - knot_vector[i])
    
    factor1 = 0     # here, 0/0 =: 0
    if abs(denominator1) > 1e-12:
      factor1 = nominator1 / denominator1
    
    # compute second factor
    nominator2 = (knot_vector[i+n+1] - x)
    denominator2 = (knot_vector[i+n+1] - knot_vector[i+1])
    
    factor2 = 0
    if abs(denominator2) > 1e-12:
      factor2 = nominator2 / denominator2
      
    return  factor1 * basis(i,n-1,x) + factor2 * basis(i+1,n-1,x)


points = [[0,0],[1,2],[3,1],[4,-1],[6,2],[8,2],[9,3],[10,4],[10,6]]
p = 3
target_multiplicity = 2

n_points = len(points)
n_basis_functions = n_points
k = n_basis_functions + p + 1  # length of knot vector
max_knot = (int)(np.ceil((k - 2*(p+1)) / float(target_multiplicity) + 1))
n_basis_functions_with_max_knot = (max_knot-1)*target_multiplicity + p + 1
print("n_basis_functions_with_max_knot: {}".format(n_basis_functions_with_max_knot))

n_missing_functions = n_basis_functions_with_max_knot - n_points
last_multiplicity = target_multiplicity - n_missing_functions

print("n_points: {}".format(n_points))
print("length of knot_vector: {}".format(k))
print("max_knot: {}".format(max_knot))
print("last_multiplicity: {}".format(last_multiplicity))

max_knot = float(max_knot)
knot_vector = \
  [0 for i in range(p+1)] \
  + [i/float(max_knot) for i in range(1,int(max_knot)-1) for j in range(target_multiplicity)] \
  + [(max_knot-1)/float(max_knot) for j in range(last_multiplicity)] \
  + [max_knot/float(max_knot) for i in range(p+1)];

k = len(knot_vector)
n_basis_functions = k - p - 1

print("p: {}".format(p))
print("target_multiplicity: {}".format(target_multiplicity))
print("knot_vector: {}".format(knot_vector))
print("n_basis_functions: {}".format(n_basis_functions))

# plot basis functions
x_list = np.linspace(-0.2,1.2,1000)
for i in range(0,n_basis_functions):
  plt.plot(x_list, [basis(i,p,x) for x in x_list], label="b_{},{}".format(i,p))
  
plt.legend()
plt.xlabel("x")
plt.ylabel("y")
plt.grid(which='major')
#plt.show()

# create spline curve
xvalues = []
yvalues = []
for x in x_list:
  
  point = [0,0]
  for i in range(0,n_basis_functions):
    point[0] += points[i][0] * basis(i,p,x)
    point[1] += points[i][1] * basis(i,p,x)

  xvalues.append(point[0])
  yvalues.append(point[1])
  
plt.figure(2)
plt.plot(xvalues, yvalues)
plt.plot([p[0] for p in points], [p[1] for p in points], "ro")
plt.show()
