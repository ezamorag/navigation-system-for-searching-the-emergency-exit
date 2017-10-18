# coding: utf-8

# Object description: A* and BasicTheta* planners for 2D grid 
#   Class variables
#      .map  contains the map
#      .height,.width size of the map
#      .s0 start state
#      .sn goal state
#   Class methods
#      .getParameters()     To see some intarnal variables
#      .NumberExpansions()  To count how many states have been expanded
#      .Numberobservedstates()   To count how many states have been observed
#      .distancepath()      To compute the displacement of a path

#      .InitializeObservedMap() To initialize the map that keep track of observed states
#      .MakeUnobservedMap_FSA() To make unobserved map freespace
#      .map_update()        To update the map accoding to real environment

#      .Initialize()          To initialize search
#      .ComputeShorestPath()  To compute shorest path


# Search w/freespace assumption (codeline ~ 238)

from math import *
import numpy as np

class Astarsearch: 
   global mB 
   #        M1     M2    M3      M4     M5     M6     M7    M8
   mB = [ [0,1],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1],[1,0],[1,1]]  
   action = [[-1,0 ], [ 0,-1], [ 1, 0], [ 0, 1], [-1,-1], [-1, 1], [ 1,-1], [ 1, 1]] 
   P_ocp = 0.95
   P_free = 0.05
   ObstacleSize = 3    # Number of cells to grow around the obstacles size
   Waypoints_Dist = 10

   def __init__(self,ogmap,s0,sn):

      self.map = ogmap.copy()
      self.height = ogmap.shape[0] 
      self.width = ogmap.shape[1] 
      if ogmap[s0[0],s0[1]] < self.P_free:
         self.s0 = s0
         self.sn = sn
      else: 
         print 'Error: Start state is not on a free space'
   
   def getParameters(self):
      self.fclosed = fclosed
  
   def NumberExpansions(self):
      N = 0
      for i in range(len(fclosed)):
         for j in range(len(fclosed[0])):
            if fclosed[i][j]==1:
               N += 1
      return N

   def Numberobservedstates(self):
      N = 0
      for i in range(self.height):
         for j in range(self.width):
            if self.map[i,j] < self.P_free or self.map[i,j] > self.P_ocp:
               N += 1
      return N

   def distancepath(self,path):
      ds = 0
      for step in range(len(path)-1):
         dx = path[step][0]-path[step+1][0]
         dy = path[step][1]-path[step+1][1]
         ds += sqrt(dx**2 + dy**2)
      return ds
   
   def InitializeObservedMap(self):
      global Obsmap
      Obsmap = np.zeros((self.height,self.width))
      Obsmap[self.s0[0],self.s0[1]] = 1

   def MakeUnobservedMap_FSA(self):
      for i in range(self.height):
         for j in range(self.width):
            if Obsmap[i,j] == 0:
               self.map[i,j] = 0.5

   def map_update(self,realenv):
      # A simple map update for agent that sense only the 8-neighborhood
      for n in range(len(self.action)):
         i = self.s0[0] + self.action[n][0]
         j = self.s0[1] + self.action[n][1]
         if i >= 0 and i < self.height and j >=0 and j < self.width:
            if 'Obsmap' in globals():
               Obsmap[i,j] = 1  
            if realenv[i,j] > self.P_free: #Necesario para mapas reales con incetidumbre 0.5
               self.map[i,j] = 1
            if realenv[i,j] < self.P_free:
               self.map[i,j] = 0

   def __c(self,s1,s2):
      dx = s1[0]-s2[0]
      dy = s1[1]-s2[1]
      cost = sqrt(dx**2 + dy**2)
      return cost

   def __h(self,s):
      dx = abs(self.sn[0]-s[0])
      dy = abs(self.sn[1]-s[1])
      #h_value = sqrt(dx**2 + dy**2)            # Euclidean norm
      #h_value = max(dx,dy)                     # Infinite Norm
      h_value = sqrt(2)*min(dx,dy) + abs(dx-dy) # Octile distances
      #h_value = dx + dy                        # Manhattan distance
      #h_value = 0
      return h_value

   def __lineofsight(self,s1,s2): #Consume 88% del tiempo de ejecución
      if self.map[s1[0],s1[1]] > self.P_ocp or self.map[s2[0],s2[1]] > self.P_ocp:
         return False 

      ## Check for line of sight using Bresenham's alg.
      dx, dy = s2[1]-s1[1], -(s2[0]-s1[0])  #Rotation of coordinates
      X = (dx>=0)
      Y = (dy>=0)
      Z = ((abs(dx)-abs(dy)) >= 0)
      F = [X and Z, Y and not Z, not X and Z, not Y and not Z]
      G = [X and Y, not X and Y, not X and not Y, X and not Y]    
      m1 = mB[2*F.index(True)]
      m2 = mB[2*G.index(True)+1]
      if Z: 
         da,db = abs(dx),abs(dy)
      else:
         da,db = abs(dy),abs(dx)
      nmax = max(abs(dx),abs(dy))
      D = 2*db-da
      i,j = s1
      for steps in range(nmax):
         if D >= 0:
            D = D+2*db-2*da
            i,j = i+m2[0],j+m2[1]
         else:
            D = D+2*db
            i,j = i+m1[0],j+m1[1]
         if self.map[i,j] > self.P_ocp:           
            return False 

      return True

   def __InitializeVertex(self,s):
      global g, parent
      g[s[0]][s[1]] = float("inf")  
      parent[s[0]][s[1]] = None     

   def Initialize(self):
      global openlist, fopen, fclosed, g, parent
   
      openlist = []
      fopen = [[0 for row in range(self.width)] for col in range(self.height)]
      fclosed = [[0 for row in range(self.width)] for col in range(self.height)]
      g = [[[] for row in range(self.width)] for col in range(self.height)]
      parent = [[[] for row in range(self.width)] for col in range(self.height)]

      self.__InitializeVertex(self.s0)
      self.__InitializeVertex(self.sn)
      g[self.s0[0]][self.s0[1]] = 0 
      parent[self.s0[0]][self.s0[1]] = self.s0   

      openlist.append([0+self.__h(self.s0),self.s0])  # First Node [f_value, vertex]
      fopen[self.s0[0]][self.s0[1]] = 1

   def __ComputeCostBasicThetastar(self,s1,s2):
      global g, parent
      g1 = g[s1[0]][s1[1]]
      g2 = g[s2[0]][s2[1]]
      parent1 = parent[s1[0]][s1[1]]
      if self.__lineofsight(parent1,s2):
         gParent1 = g[parent1[0]][parent1[1]]
         if gParent1 + self.__c(parent1,s2) < g2:
            parent[s2[0]][s2[1]] = parent1
            g[s2[0]][s2[1]] = gParent1 + self.__c(parent1,s2)
      else: 
         if g1 + self.__c(s1,s2) < g2:
            parent[s2[0]][s2[1]] = s1
            g[s2[0]][s2[1]] = g1 + self.__c(s1,s2)

   def __ComputeCostAstar(self,s1,s2):
      global g, parent
      g1 = g[s1[0]][s1[1]]
      g2 = g[s2[0]][s2[1]]
      parent1 = parent[s1[0]][s1[1]]
      if g1 + self.__c(s1,s2) < g2:
         parent[s2[0]][s2[1]] = s1
         g[s2[0]][s2[1]] = g1 + self.__c(s1,s2)

   def __UpdateVertex(self,s1,s2):
      i2,j2 = s2
      g_old = g[i2][j2]
      self.__ComputeCostBasicThetastar(s1,s2)
      if g[i2][j2] < g_old:
         if fopen[i2][j2]:
            f2 = g_old + self.__h(s2)
            openlist.remove([f2,s2])
         openlist.append([g[i2][j2]+self.__h(s2), s2])
         fopen[i2][j2] = 1

   def __line_update(self,s1,s2,path):
      ## Check for line of sight using Bresenham's alg.
      dx, dy = s2[1]-s1[1], -(s2[0]-s1[0])  #Rotation of coordinates
      X = (dx>=0)
      Y = (dy>=0)
      Z = ((abs(dx)-abs(dy)) >= 0)
      F = [X and Z, Y and not Z, not X and Z, not Y and not Z]
      G = [X and Y, not X and Y, not X and not Y, X and not Y]    
      m1 = mB[2*F.index(True)]
      m2 = mB[2*G.index(True)+1]
      if Z: 
         da,db = abs(dx),abs(dy)
      else:
         da,db = abs(dy),abs(dx)
      nmax = max(abs(dx),abs(dy))
      D = 2*db-da
      i,j = s1
      newpath = []
      for steps in range(nmax):
         newpath.append([i,j])
         if D >= 0:
            D = D+2*db-2*da
            i,j = i+m2[0],j+m2[1]
         else:
            D = D+2*db
            i,j = i+m1[0],j+m1[1]

      newpath.extend(path)
      return newpath  

   def __PathGeneration(self):
      path = [self.sn]
      s = self.sn
      while not s == self.s0:
         s2 = s 
         s = parent[s[0]][s[1]]
         path = self.__line_update(s,s2,path)   

      return path

   def ComputeShorestPath(self):
      global openlist, fclosed

      while openlist[-1][0] < g[self.sn[0]][self.sn[1]]:  #f_TopKey < g(goal) + h(goal)
         s = openlist.pop()
         f = s[0]
         i = s[1][0]
         j = s[1][1]
         fclosed[i][j] = 1
         for n in range(len(self.action)):
            i2 = i + self.action[n][0]
            j2 = j + self.action[n][1]
            if i2 >= 0 and i2 < self.height and j2 >=0 and j2 < self.width and self.map[i2,j2] < self.P_ocp:  #Freespace assumption
                if not fclosed[i2][j2]:
                   if not fopen[i2][j2]:
                      self.__InitializeVertex([i2,j2])
                   self.__UpdateVertex([i,j],[i2,j2])
         if len(openlist) == 0:
            break
         openlist.sort()
         openlist.reverse()
   
      if g[self.sn[0]][self.sn[1]] < float("inf"):
         self.path = self.__PathGeneration()
      else:
         self.path = None

      return self.path

   def path2waypoints(self):
      self.waypoints = []
      if self.path == None:
         print "There isn't path"
         return 
      ds = 0
      for step in range(len(self.path)-1):
         dx = self.path[step][0]-self.path[step+1][0]
         dy = self.path[step][1]-self.path[step+1][1]
         ds += sqrt(dx**2 + dy**2)
         if ds > self.Waypoints_Dist:
            ds = 0
            self.waypoints.append(self.path[step+1])    # step 
      #self.waypoints.append(self.path[-1])
      
   def GrowingObstacles(self):
      dij = range(-self.ObstacleSize,self.ObstacleSize+1) 
      n = len(dij)
      newgrid = np.copy(self.map)
      for i in range(self.height):
         for j in range(self.width):
            if self.map[i,j] >= self.P_ocp:
               for r in range(n):
                  for c in range(n):
                     i2 = i+dij[r]
                     j2 = j+dij[c]
                     if i2 < 0 or i2 > self.height or j2 < 0 or j2 > self.width:
                        continue
                     newgrid[i2,j2] = 1.0
      self.map = newgrid;
      return 

