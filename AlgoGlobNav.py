#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import math
import heapq


# In[2]:


class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
    def get_coordinates(self):
        coordinates = [self.x,self.y]
        return coordinates
    
class Edge:
    def __init__(self,x1 = 0,y1= 0,x2= 0,y2=0):
        self.start = Point(x1,y1)
        self.end = Point(x2,y2)

    def len_(self):
        return np.sqrt((self.start.x - self.end.x)**2 + (self.start.y-self.end.y)**2)

    def compare_start(self,p2):
        if(self.start.x == p2.x and self.start.y == p2.y):
            return 1
        else:
            return 0
    def compare_end(self,p2):
        if(self.end.x == p2.x and self.end.y == p2.y):
            return 1
        else:
            return 0
    
        
    def get_coordinates(self):
        coordinates = [self.start.x,self.start.y,self.end.x,self.end.y]
        return coordinates


# In[3]:


def initNodes(inputN):

	Nodes = len(inputN)*[0]      
	for i in range(len(Nodes)):
		Nodes[i] = Point(inputN[i][0],inputN[i][1])
	return Nodes

def initEdges(inputN,inputE):
    Edges = 2*len(inputE)*[0]
    
    for i in range(len(inputE)):
        Edges[2*i] = Edge(inputN[inputE[i][0]][0],inputN[inputE[i][0]][1],
                    inputN[inputE[i][1]][0],inputN[inputE[i][1]][1])
        Edges[2*i+1] =Edge(inputN[inputE[i][1]][0],inputN[inputE[i][1]][1],
                         inputN[inputE[i][0]][0],inputN[inputE[i][0]][1])
    return Edges


# In[4]:


def dijkstra(inputNodes,inputeEdges,index_goal): 
    
    #Initilization of objects and tab
    Edges = initEdges(inputNodes,inputeEdges)
    Nodes = initNodes(inputNodes)
    tabLenPath = len(nodes)*[float('inf')]
    tabPath = [ [] for _ in range(len(Nodes))]
    tabIndex = {(k.x,k.y): v for v, k in enumerate(Nodes)}

    #Initial condition
    act_node = Nodes[-2]
    act_dist = 0
    act_path = [Nodes[-2]]
    iteration = 0
    
    #Check if all edges are used
    while(len(Edges) != 0 or iteration > 200): 
        
        for edge in Edges:
            #Check if starting node of edge is actual node
            if(edge.compare_start(act_node)):
                
                #Find the index of this node 
                idx = tabIndex[(edge.end.x,edge.end.y)]
                
                #Check if current path is shortest than the new one 
                if(act_dist +  edge.len_() < tabLenPath[idx]):
                    tabLenPath[idx] = act_dist +  edge.len_()
                    
                    #Check if current path is empty or not
                    if(len(tabPath[idx])== 0):

                        for i in act_path:
                            tabPath[idx].append(i)
                        tabPath[idx].append(edge.end)

                    else:
                        tabPath[idx].clear()
                        for i in act_path:
                            tabPath[idx].append(i)
                        tabPath[idx].append(edge.end)
                else:
                    continue
        #Removing all used edge on the list
        Edges = [item for item in Edges if (item.compare_end(act_node)== 0 and item.compare_start(act_node)== 0)]

        #new condition
        iteration += 1
        min_dist = heapq.nsmallest(iteration,tabLenPath)
        min_dist = min_dist[-1]
        idx_min = tabLenPath.index(min_dist)
        act_dist = tabLenPath[idx_min]
        act_node = Nodes[idx_min]
        act_path = tabPath[idx_min]

    return tabPath[index_goal]


# In[ ]:




