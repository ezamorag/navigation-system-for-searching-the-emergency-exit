# coding: utf-8
# Functions to create images of planning results 
import numpy as np
import matplotlib.pyplot as plt
import re
import yaml

P_ocp = 0.95
P_free = 0.05

def CreateMapPathExpansion(ogmap,fclosed=None,path=None,init=None,goal=None,sp=None):
    ### Create a image of map, path and explored cells ###
    height = ogmap.shape[0]
    width = ogmap.shape[1]
    image = np.zeros((height,width,3)) 
    ogmap = 1 - ogmap 
    image[:,:,0] = ogmap 
    image[:,:,1] = ogmap
    image[:,:,2] = ogmap

    if fclosed != None: 
       for i in range(height):
          for j in range(width):
             if fclosed[i][j] == 1:
                image[i,j,:] = [0.75,0.75,0.0]   # Closed cells - light grey

    if path != None:
	image[path[:,0],path[:,1],:] = [1,0,0]
    if init != None:
	image[init[0],init[1],:] = [0,1,0]
    if goal != None:
	image[goal[0],goal[1],:] = [0,0,1]
    if sp != None:
	image[sp[0],sp[1],:] = [1,0,1]
    return image

def CreateMapPathObservation(ogmap,Obsmap=None,fObstacles=None,path=None,init=None,goal=None,sp=None):
    ### Create a image of map, path and explored cells ###
    height = ogmap.shape[0]
    width = ogmap.shape[1]
    image = np.zeros((height,width,3)) 
    tmp = 1 - ogmap 
    image[:,:,0] = tmp 
    image[:,:,1] = tmp
    image[:,:,2] = tmp

    if Obsmap != None: 
       for i in range(height):
          for j in range(width):
             if Obsmap[i][j] == 1:
                if ogmap[i,j] < P_free:
                   image[i,j,:] = [0,0.7,1]   
                elif ogmap[i,j] > P_ocp: 
                   image[i,j,:] = [0,0.5,0.1] 
                else:
                   image[i,j,:] = [0,0,0.0] 

    if fObstacles != None: 
       for i in range(height):
          for j in range(width):
             if fObstacles[i][j] == 1:
                image[i,j,:] = [0.75,0.75,0.0]   

    if path != None:
	image[path[:,0],path[:,1],:] = [1,0,0]
    if init != None:
	image[init[0],init[1],:] = [0,1,0]
    if goal != None:
	image[goal[0],goal[1],:] = [0,0,1]
    if sp != None:
	image[sp[0],sp[1],:] = [1,0,1]
    return image

def TakePicture(nMovie,outfolder,ogmap,Obsmap,fObstacles,path,s0,sn,sp): 
   MaxDigits = 5  # Maximum number of digits to name images
   imaplot = plt.imshow(CreateMapPathObservation(ogmap,Obsmap,fObstacles,path,s0,sn,sp),origin='upper')
   nZeros = MaxDigits
   n = nMovie
   while not n == 0:
      n = n/10
      nZeros -= 1
   num = str(nMovie)
   for i in range(nZeros):
      num = '0' + num

   plt.savefig(outfolder + '/image' + num + '.png')
   nMovie += 1

   return nMovie


def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

def pgmTogrid(image, docyaml):
    
    occupied_thresh = docyaml['occupied_thresh']
    free_thresh = docyaml['free_thresh']
    negate = docyaml['negate']

    new = np.zeros(image.shape)
    if negate == 0:
       for i in range(image.shape[0]):
          for j in range(image.shape[1]):
             new[i,j] = 1.0 - image[i,j]/255.0
             if new[i,j] < occupied_thresh and new[i,j] > free_thresh:
                new[i,j] = 0.5
    elif negate == 1: 
       for i in range(image.shape[0]):
          for j in range(image.shape[1]):
             new[i,j] = image[i,j]/255.0
             if new[i,j] < occupied_thresh and new[i,j] > free_thresh:
                new[i,j] = 0.5
    return new

