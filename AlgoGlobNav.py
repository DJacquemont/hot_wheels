import numpy as np
import math
import heapq
import vision


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
        if(self.start.x == p2[0] and self.start.y == p2[1]):
            return 1
        else:
            return 0
    def compare_end(self,p2):
        if(self.end.x == p2[0] and self.end.y == p2[1]):
            return 1
        else:
            return 0
    
        
    def get_coordinates(self):
        coordinates = [self.start.x,self.start.y,self.end.x,self.end.y]
        return coordinates


def initEdges(inputN,inputE):
    Edges = np.empty(2*len(inputE),dtype = object)
    
    for i in range(len(inputE)):
        Edges[2*i] = Edge(inputN[inputE[i][0]][0],inputN[inputE[i][0]][1],
                    inputN[inputE[i][1]][0],inputN[inputE[i][1]][1])
        Edges[2*i+1] =Edge(inputN[inputE[i][1]][0],inputN[inputE[i][1]][1],
                         inputN[inputE[i][0]][0],inputN[inputE[i][0]][1])
    return Edges


def dijkstra(inputNodes,inputeEdges,index_goal): 
    
    #Initilization of objects and tab
    Edges = initEdges(inputNodes,inputeEdges)
    Nodes = inputNodes
    tabLenPath = np.empty(len(nodes), dtype=float) 
    tabLenPath[:] = float('inf')
    tabPath = [ [] for _ in range(len(Nodes))]
    tabIndex = {(k[0],k[1]): v for v, k in enumerate(Nodes)}

    #Initial condition
    act_node = Nodes[-2]
    act_dist = 0
    act_path = [Nodes[-2]]
    iteration = 0
    
    #Check if all edges are used
    while(len(Edges) != 0 ): 
        
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
                            tabPath[idx].append([i[0],i[1]])
                        tabPath[idx].append([edge.end.x,edge.end.y])

                    else:
                        tabPath[idx].clear()
                        for i in act_path:
                            tabPath[idx].append([i[0],i[1]])
                        tabPath[idx].append([edge.end.x,edge.end.y])
                else:
                    continue
        #Removing all used edge on the list
        Edges = [item for item in Edges if (item.compare_end(act_node)== 0 and item.compare_start(act_node)== 0)]

        #new condition
        iteration += 1
        min_dist = heapq.nsmallest(iteration,tabLenPath)
        min_dist = min_dist[-1]
        idx_min  = int(np.where(tabLenPath == min_dist)[0])
        act_dist = tabLenPath[idx_min]
        act_node = Nodes[idx_min]
        act_path = tabPath[idx_min]
	
    return np.array(tabPath[index_goal]


def opt_path(vid):
    while(True):
        terrain = vision.terrainFetch(vid)
        if type(terrain) != bool:
            nodes,nodeCon, maskObsDilated = terrain
            break
    optimal_path = dijkstra(node,nodeCon,-1)
    return optimal_pathP*vision.fieldWidthM/vision.fieldWidthP
		



