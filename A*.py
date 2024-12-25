from collections import defaultdict
from math import inf
from typing import final
import pandas as pd 
import math
from matplotlib import  pyplot as plt
import argparse

class Vertex:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

#compute heuristic (Euclidean cost) to reach goal from given node
def heuristic(node):
    return math.sqrt((nodes[goal].x - nodes[node].x)**2 + (nodes[goal].y - nodes[node].y)**2)

#compute Euclidean distance between two nodes
def dist(a,b):
    return math.sqrt((nodes[a].x - nodes[b].x)**2 + (nodes[a].y - nodes[b].y)**2)

#compute the total path from start to current
def reconstruct_path(cameFrom, current):
    path=[]
     
    while current != start:
        path.append(current)
        current = cameFrom[current]

    path.append(start)
    path.reverse()
    return path

#compute total path cost 
def total_path_cost(total_path):
    total_cost=0
    for i in range(len(total_path)-1):
        total_cost=total_cost+dist(total_path[i],total_path[i+1])
    return total_cost


#compute optimal path from start to goal
#A* algorithm
def A_star(start, goal):
    
    open_list=[]
    open_list.append((0, start))

    cameFrom={}
    #gScore defines the cost of the cheapest path from start
    g_score=defaultdict(lambda:inf)
    g_score[start]=0
     #fScore represents our current best guess as to how short a path from start to finish can
    f_score=defaultdict(lambda:inf)
    f_score[start]=g_score[start]+heuristic(start)

    #openList is not empty
    while len(open_list)>0:
        total_path=[]
        open_list.sort(reverse=True)
        current_pop=open_list.pop()
        
        current=current_pop[1]
        
        
        if current==goal:
            total_path= reconstruct_path(cameFrom,current)
            total_cost= total_path_cost(total_path)
            return total_path,total_cost
        #each neighbor of current
        for i in neighbour[current]:
            #dist(current,neighbour) is the weight of the edge from current to  neighbour tentative_gScore is the distance from start to the neighbour through current
            ten_gscore=g_score[current]+dist(current,i)
            if ten_gscore < g_score[i]:
                #this path to neighbour is better than any previous one.
                cameFrom[i]=current
                g_score[i]=ten_gscore
                f_score[i]=g_score[i]+heuristic(i)
               
                #neighbor not in openSet
                not_present_in_open_list=0
                if not_present_in_open_list==0:
                    open_list.append((f_score[i],i))

    #openList is empty but goal was never reached
    print('No solution found')
    return -1,-1

    


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='A* algorithm for pathfinding using Euclidean distance heuristic')
    parser.add_argument('environment_file', type=str, help='Path to the environment CSV file')
    parser.add_argument('visibility_graph_file', type=str, help='Path to the visibility graph CSV file')

    args = parser.parse_args()

    #reading environment
    csv1=pd.read_csv(args.environment_file)
    
    df=pd.DataFrame(csv1)
    col=df.columns
    
    row_count, column_count = df.shape
    vertex_x=[]
    vertex_y=[]
    
    for i in range(row_count):


        vertex_x.append(df[col[1]].iloc[i])
        vertex_y.append(df[col[2]].iloc[i])

    #reading Visibility graph
    csv2=pd.read_csv(args.visibility_graph_file)
    df1=pd.DataFrame(csv2)  
    row_count1, column_count1= df1.shape
    vertex_vis_x=[]
    vertex__vis_y=[]
    col1=df1.columns
    for i in range(row_count1):


        vertex_vis_x.append(df1[col1[0]].iloc[i])
        vertex__vis_y.append(df1[col1[1]].iloc[i])
  
    nodes=[]

    #converting to Vertex type object
    for i in range(row_count):
        nodes.append(Vertex(vertex_x[i],vertex_y[i]))


    #creating Visibility Graph as neighbour dictionary
    neighbour = {} 
    [neighbour.setdefault(i, []) for i in range(row_count)] 
   
    for i in range(row_count):
        for j in range(row_count1):
            if(vertex_vis_x[j]==i):
                neighbour[i].append(vertex__vis_y[j])
    #print(neighbour)

    start=0
    goal=row_count-1

    total_path=[]

    #calling A* function
    total_path, total_cost=A_star(start,goal)

    print('Path',total_path)
    print('Distance',total_cost)

    # plotting A* on Visibility Graph
    
    plt.figure(figsize=[10,8])

    for i in range(row_count):
        plt.plot(nodes[i].x,nodes[i].y,'+','r')
        plt.text(nodes[i].x,nodes[i].y,i)
    
    for i in range(row_count1):
        plt.plot([nodes[vertex_vis_x[i]].x,nodes[vertex__vis_y[i]].x],[nodes[vertex_vis_x[i]].y,nodes[vertex__vis_y[i]].y],'g--')
    for i in range(len(total_path)-1):
        plt.plot([nodes[total_path[i]].x,nodes[total_path[i+1]].x],[nodes[total_path[i]].y,nodes[total_path[i+1]].y],'r')

    plt.xlabel('x - axis')
    plt.ylabel('y - axis')
    plt.title('A-Star on Visibility Graph')
    plt.show()