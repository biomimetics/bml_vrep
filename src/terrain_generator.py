from digifab import *
import random
from numpy.linalg import inv, det
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from noise import pnoise2,snoise2
import os

random.seed(42)

def normal_pdf(x,u=(0,0),sigma=numpy.eye(2)):
  x = numpy.array(x).reshape(-1,2)
  u = numpy.array(u)
  si = inv(sigma)
  denom = (2*numpy.pi*det(sigma))**0.5
  return numpy.exp([-(d).T.dot(si.dot(d))/2 for d in x-u])/denom

def random_surface(n_o=10, max_h=1000, n_x=100, n_y=100, x=5000, y=5000):
  surface = numpy.zeros((n_x,n_y))
  x = numpy.array([[i,j] for i in range(n_x) for j in range(n_y)])
  X,Y = numpy.meshgrid(n_x,n_y)
  
  for n in range(n_o):
    u = numpy.random.rand(2) * numpy.array([n_x,n_y])
    sigma = numpy.random.rand(2,2)
    sigma = sigma.T.dot(sigma)
    sigma *= (50 + 50*numpy.random.rand())
    h = (numpy.random.rand()*max_h)/n_o
    Z = h*normal_pdf(x,u,sigma).reshape(n_x,-1)
    surface += Z
  
  return surface
 
def simplex_surface(max_h=100.0, n_x=100, n_y=100, freq=0.5):
  surface = numpy.zeros((n_x,n_y))
  for i in range(n_x):
    for j in range(n_y):
      surface[i,j] = max_h*snoise2(i/(freq*n_x),j/(freq*n_y))
  return surface

def add_walls(surface, wall_height=1000.0):
  w,h = surface.shape
  surface[0:w,0] = wall_height
  surface[0,0:h] = wall_height
  surface[w-1,0:h] = wall_height
  surface[0:w,h-1] = wall_height

def save_surface(surface, filename, x=5000.0, y=5000.0):
  n_x, n_y = surface.shape
  x_scale = x/n_x
  y_scale = y/n_y
  dat_filename = os.getcwd() + '/surface.txt'
  f = open(dat_filename,'w')
  for line in surface:
    for v in line:
      f.write('%f ' % v)
    f.write('\n')
  f.close()
  s = PolyMesh(
    generator=solid.scale([x_scale,y_scale,1])(
      solid.surface(dat_filename, center=True)
    )
  )
  s.save(filename)

def plot_surface(Z):
  n_x, n_y = Z.shape
  X,Y = numpy.meshgrid(range(n_x),range(n_y))
  fig = plt.figure()
  ax = fig.add_subplot(111,projection='3d')
  ax.plot_wireframe(X,Y,Z)
  plt.axis('equal')
  plt.show()

