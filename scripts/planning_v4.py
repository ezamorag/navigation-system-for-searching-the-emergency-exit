#!/usr/bin/env python
# it requires "chmod +x mypythonscript.py" to be called by ROS
import rospy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseArray
import tf  
from find_emergency_exit.srv import on_off, motion_goal 
import numpy as np
from mAstar import *

from mVisualization import *
import matplotlib.pyplot as plt

Nplans = 0
flagStop = True
pastpath = []

def h(s):
   th = thA + 3.0/4.0*pi  
   dx = s[0] - sA[0]
   dy = s[1] - sA[1]
   p1 =  cos(th)*dx + sin(th)*dy
   p2 = -sin(th)*dx + cos(th)*dy 
   return max(p1,p2)

def compute_goal(): 
   th = thA + 3.0/4.0*pi
   a = cos(th)+sin(th)
   b = sin(th)-cos(th)
   c = - a*sA[0] - b*sA[1]

   minpoints = []
   ymax = height - 1
   xmax = width - 1
   yright = -(a*xmax+c)/b 
   if yright <= ymax and yright >= 0:
      s = [xmax,yright]
      minpoints.append([h(s),s])
   yleft = -c/b
   if yleft <= ymax and yleft >= 0:
      s = [0,yleft]
      minpoints.append([h(s),s])
   xdown = -(b*ymax+c)/a 
   if xdown <= xmax and xdown >= 0:
      s = [xdown,ymax]
      minpoints.append([h(s),s])
   xup = -c/a 
   if xup <= xmax and xup >= 0:
      s = [xup,0]
      minpoints.append([h(s),s])
   minpoints.sort()
   sn = [int(round(minpoints[0][1][0])),int(round(minpoints[0][1][1]))]
   return sn

def h_update_targetsign(req):
   global xsign0, ysign0, thsign0
   xsign0 = req.x
   ysign0 = req.y
   thsign0 = req.th
   return True

def h_stop(req):
   global flagStop
   flagStop = req.switch1 
   str1 = "Switch off Planner %s"%rospy.get_time()
   str2 = "Switch on Planner %s"%rospy.get_time()
   if flagStop:
      rospy.loginfo(str1)
   else:
      rospy.loginfo(str2)
   return True

def localization_cb(robot):
   global s0
   if not 'x0' in globals():
      return
   s0 = [int((robot.pose.position.x - x0)/delta), int((robot.pose.position.y - y0)/delta)]

def IsPathValid(gmap, path):
   for i in range(len(path)):
      if not gmap[path[i][0],path[i][1]] < 0.90:
         return False
   return True

def map_cb(gmap):
   global delta, x0, y0, height, width, Nplans, flagStop, sA, thA, pastpath
   if flagStop:
      return

   delta = gmap.info.resolution
   x0 = gmap.info.origin.position.x
   y0 = gmap.info.origin.position.y
   height = gmap.info.height
   width = gmap.info.width
   # Creating the map matrix
   nmap = np.zeros([gmap.info.height,gmap.info.width])
   for i in range(len(gmap.data)):
      y = i/gmap.info.width 
      x = i - y*gmap.info.width
      if gmap.data[i] == -1:
         nmap[x,y] = 0.5
      else:
         nmap[x,y] = gmap.data[i]/100.0

   if not 's0' in globals():
      return
   
   if not pastpath == None: 
      if IsPathValid(nmap, pastpath) and not len(pastpath)==0:
         return


   sA = [int((xsign0 - x0)/delta), int((ysign0  - y0)/delta)]      
   thA = thsign0
   sn = compute_goal()
   nmap[s0[0],s0[1]] = 0
   planner = Astarsearch(nmap,s0,sn)
   planner.ObstacleSize = 3
   planner.Waypoints_Dist = 10
   planner.GrowingObstacles()
   planner.Initialize()
   planner.ComputeShorestPath()
   pastpath = planner.path 
   planner.path2waypoints()

   # Update waypoints
   Nplans += 1
   waypoints = Path()
   waypoints.header.seq = Nplans
   waypoints.header.stamp = rospy.Time.now()
   waypoints.header.frame_id = "map"
   for i in range(len(planner.waypoints)):
      waypoint = PoseStamped()
      waypoint.header.seq = i
      waypoint.header.stamp = waypoints.header.stamp
      waypoint.header.frame_id = waypoints.header.frame_id
      waypoint.pose.position.x = x0 + planner.waypoints[i][0]*delta
      waypoint.pose.position.y = y0 + planner.waypoints[i][1]*delta
      waypoint.pose.position.z = 0.0
      waypoint.pose.orientation.x = 0.0
      waypoint.pose.orientation.y = 0.0
      waypoint.pose.orientation.z = 0.0
      waypoint.pose.orientation.w = 0.0  
      waypoints.poses.append(waypoint)
   if not flagStop:
      pub.publish(waypoints)
 

   #ima = CreateMapPathExpansion(nmap,None,np.array(planner.waypoints),planner.s0,None)
   #plt.imshow(ima,origin='upper')
   #plt.show() 


def planning():
    global pub
    rospy.init_node('arrow_planning', anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, map_cb)
    rospy.Subscriber("robotpose", PoseStamped , localization_cb)
    rospy.Service('stop_planner', on_off, h_stop)
    rospy.Service('motion_goal', motion_goal, h_update_targetsign)
    pub = rospy.Publisher('waypoints', Path, queue_size=1)
    rospy.spin()
    return
        
if __name__ == '__main__':
    planning()
