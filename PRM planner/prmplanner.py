# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import random
from math import sqrt
import math
import numpy as np

disk_robot = True #(change this to False for the advanced extension) 


obstacles = None # the obstacles 
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap, 
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class  

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius, nei_list

    obstacles = scene_obstacles # setting the global obstacle variable
    
    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)
     
    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    
    robot_radius = max(robot_width, robot_height)/2.

    # the roadmap 
    graph = Roadmap()
    #checking for optimum samples which will reside in the C-Space
    for i in range(0,10000): 
        
        x = random.uniform(-48, 48)
        y = random.uniform(-48, 48)
        q = (x, y)
        c1 = 0
        vertex_class = graph.getVertices()
        vertex_no = graph.getNrVertices()
        
        for u in range(0, vertex_no):        
            if distance(q, vertex_class[u].getConfiguration()) < 2: #minimum distance between 2 configurations = 2
                c1 = 50
                break                
        if c1 == 50:
            pass
        elif collision(q):
            pass
        else:
            graph.addVertex(q)
        
    vertex_class = graph.getVertices()
    vertex_no = graph.getNrVertices() 
 
#************ The below code checks for the redundant edges ******************
# It checks edges connecting to each neighbor and returns a list of children.
# For each of the children, it generates another set of children which check 
# for the vertex_id of the parent vertex and adds an edge if it is not in the list
    
    for h in range(0, vertex_no):
        config = vertex_class[h]
        config_p = vertex_class[h].getConfiguration()
        config_id = vertex_class[h].getId()
        nei, dist_list = nearest_neighbors(graph, config_p, 8)
        
        
        
        for r in range(0, len(nei)):
            edge_id = []
            edge = []
            n_id = nei[r].getId()
            edge = nei[r].getEdges() #checks edges connecting to each neighbor
           
            for l in range(0, len(edge)):
                edge_id.append(edge[l].getId())
            em = []
            for o in range(0, len(edge_id)):
                em.append(vertex_class[edge_id[o]])
                
            edge_cid = []
            for k in range(0, len(em)):
                edge_1 = em[k].getEdges()
                for u in range(0, len(edge_1)):
                    edge_cid.append(edge_1[u].getId())
                    
            em1 = []
            for o in range(0, len(edge_cid)):
                em1.append(vertex_class[edge_cid[o]])
            edge_cid1 = []
            for k in range(0, len(em1)):
                edge_11 = em1[k].getEdges()
                for u in range(0, len(edge_11)):
                    edge_cid1.append(edge_11[u].getId())
                    
            em11 = []
            for o in range(0, len(edge_cid1)):
                em11.append(vertex_class[edge_cid1[o]])
            edge_cid11 = []
            for k in range(0, len(em11)):
                edge_111 = em11[k].getEdges()
                for u in range(0, len(edge_111)):
                    edge_cid11.append(edge_111[u].getId())
            
            em111 = []
            for o in range(0, len(edge_cid11)):
                em111.append(vertex_class[edge_cid11[o]])
            edge_cid111 = []
            for k in range(0, len(em111)):
                edge_1111 = em111[k].getEdges()
                for u in range(0, len(edge_1111)):
                    edge_cid111.append(edge_1111[u].getId())
                    
            if config_id in edge_id:
                pass
            elif config_id in edge_cid:
                pass            
            elif config_id in edge_cid1:
                pass 
            elif config_id in edge_cid11:
                pass 
            elif config_id in edge_cid111:
                pass                    
            else:        
                il = interpolate(config_p, nei[r].getConfiguration(), 8)
                dist = dist_list[r]
                count = 0
                for j in range(0, len(il)):
                            
                    if collision(il[j]) == True:
                        count = 50
                    else:
                        pass
                if count == 0:               
                    graph.addEdge(config, nei[r],dist, path=[])     
                
    return graph
    

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None
    
def find_path(q_start, q_goal, graph):
   
    if collision(q_start) == True or collision(q_goal) == True:
        return None #does not return any path if the start or goal vertices are colliding with obstacles
    else:        
        path = []        
        graph.addVertex(q_start) #adding q_start to vertex list
        graph.addVertex(q_goal)  #adding q_goal to vertex list      
        vertex_class = graph.getVertices()
        vertex_no = graph.getNrVertices() 
        ver = [q_start, q_goal]
        #runs a for loop for the q_start and q_goal to find edges 
        for d in range(0,len(ver)):
            nei, dist_list = nearest_neighbors(graph, ver[d], 5)
            for r in range(0, len(nei)):
                il = interpolate(ver[d], nei[r].getConfiguration(), 8)
                dist = dist_list[r]
                count = 0
                for j in range(0, len(il)):                            
                    if collision(il[j]) == True: #does not add any edge
                        count = 50
                    else:
                        pass
                if count == 0: #adds an edge
                    if d == 0:
                        graph.addEdge(vertex_class[-2], nei[r],dist, path=[])
                    else:
                        graph.addEdge(vertex_class[-1], nei[r],dist, path=[])            
       
         # Use the OrderedSet for your closed list
        closed_set = OrderedSet()
        
        # Use the PriorityQueue for the open list
        open_set = PriorityQueue(order=min, f=lambda v: v.f) 
    
        start_id = vertex_class[-2].getId() #gets the id of the starting vertex
        goal_id = vertex_class[-1].getId() #gets the id of the goal vertex
        g = 0 #g value of the starting vertex
        h = distance(vertex_class[start_id].getConfiguration(), vertex_class[goal_id].getConfiguration()) #h value of the starting vertex
        f = g + h
        parent = {} #initializing a dictionary to store the parents 
        check = []   
        open_set.put(start_id, Value(f=f, g=g))
 
#************* Using A-star as the discrete planning algorithm ****************
       
        while len(open_set)>0:
            new_id, value = open_set.pop() #popping the openset
            closed_set.add(new_id) 
            g_p = value.g
            config = vertex_class[new_id] 
            config_p = config.getConfiguration()
            
            if new_id == goal_id:
                while not new_id == start_id:
                    new_id = parent[new_id]
                    parent_p = vertex_class[new_id].getConfiguration()
                    path.insert(0,parent_p)
                break
            else:
                edge = vertex_class[new_id].getEdges()
                check.append(new_id)
                for l in range(0, len(edge)):
                    e_id = edge[l].getId()
                    if e_id not in check:                
                        parent.update({e_id : new_id})
                        e_id_p = vertex_class[e_id].getConfiguration()                
                        dist = distance(e_id_p, config_p)                
                        g = g_p + dist
                        h = distance(e_id_p, vertex_class[goal_id].getConfiguration())
                        f = g + h                
                        if e_id not in closed_set:
                            open_set.put(e_id, Value(f=f, g=g))
                        else:
                            pass
                    else:
                        pass 
        return path   


# ----------------------------------------
# below are some functions that you may want to populate/modify and use above 
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances 
    """
    
    vertex_class = graph.getVertices()
    vertex_no = graph.getNrVertices()
    near = []
    distances = []
    
    for p in range(0, vertex_no):
        dist = distance(q, vertex_class[p].getConfiguration())
        if dist == 0:
            pass
        elif dist < max_dist:
            near.append(vertex_class[p])
            distances.append(dist)

    return near, distances


def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q. 
        You may also want to return the corresponding distances 
    """
  
    return None

def distance (q1, q2): 
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """
    
    x_1 = q1[0] - q2[0]
    y_1 = q1[1] - q2[1]
    
    dist = sqrt((x_1*x_1) + (y_1*y_1))

    
    return dist 

def collision(q):
    """
        Determines whether a configuration q will collide with the list of AABB obstacles.  
    """
    for k in range(0, len(obstacles)):
        o = obstacles[k].points
        o1 = o[0]
        o2 = o[1]
        o3 = o[2]            
        x1 = o1[0] 
        x1 = x1 - 2
        x2 = o2[0] 
        x2 = x2 + 2
        y1 = o2[1]
        y1 = y1 + 2
        y2 = o3[1] 
        y2 = y2 - 2
        
        if (x1 < q[0] < x2) and (y1 > q[1] > y2):
            return True
            break
   

def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path of vertices between two given configurations. 
        It can be used to determine whether an edge between vertices is collision-free. 
    """
    
    interpol_list = []
    
    x = (q1[0]+q2[0])/2
    y = (q1[1]+q2[1])/2
    q3 = (x, y)
    interpol_list.append(q3)
    
    x = (q1[0]+q3[0])/2
    y = (q1[1]+q3[1])/2
    q4 = (x, y)
    interpol_list.append(q4)
    
    x = (q2[0]+q3[0])/2
    y = (q2[1]+q3[1])/2
    q5 = (x, y)
    interpol_list.append(q5)
        
    return interpol_list


if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
