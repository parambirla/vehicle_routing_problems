import time
import pandas as pd
import numpy as np
import math
import networkx as nx
import gurobipy as gp
from gurobipy import GRB


#this function is for reading data from .txt file and storing it into a 2D array where i'th index in array contains data related to i'th customer
# [x_coord,y_coord,deadline]
def extract_coordinates_due_date(filename):
    data = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        data_start = False
        for line in lines:
            line = line.strip()
            if not line:
                continue
            if "CUST NO." in line:
                data_start = True
                continue
            if data_start:
                values = line.split()
                try:
                    xcoord = int(values[1])
                    ycoord = int(values[2])
                    ready_time = int(values[4])
                    due_date = int(values[5])
                    data.append([xcoord, ycoord,ready_time ,due_date])
                except ValueError:
                    continue
    return np.array(data)



def deadline_vrp_lp(n,start_time,end_time, t, S,active_customers,disc_fact):
    model = gp.Model("VRP_MinVehicles_Deadline_rlp")
    model.Params.OutputFlag = 1
    x = {}
    flow = {}
    edges = []
    
    vertices = set()
    vertex_tu = set()
    vertex_tu_tm = {}
    
    vert_custi = {}

    edges_in = {}
    edges_out = {}
    
    '''
    # Add vertices only for active customers
    for v in active_customers:
        for i in range(d[v] + 1):
            vertices.add((v, i))
            
        sink_node = (v, d[v] + 1)
        vertices.add(sink_node)
        vertex_tu.add(sink_node)
    '''
       
    for v in active_customers:
        arr=[]
        
        st=start_time[v]
        if(st%disc_fact!=0):
            st=st+(disc_fact-st%disc_fact)
        
        et=end_time[v]
        if(et%disc_fact!=0):
            et=et+(disc_fact-et%disc_fact)         #check once again
            
        i=st
        while(i<=et):
            vertices.add((v,i))
            arr.append(i)
            edges_in[(v,i)]=[]
            edges_out[(v,i)]=[]
            i=i+disc_fact
        
        vert_custi[v]=arr   
        sink_node=(v,i)
        vertex_tu.add(sink_node)
        vertex_tu_tm[v]=i
        edges_in[sink_node]=[]
        edges_out[sink_node]=[]
    
    
    # adding edge  e = ((v,i) -> (u,j))     if i+t_vu = j  .
    for v in active_customers:
        for i in vert_custi[v]:
            for u in active_customers:
                if u != v:
                    for j in vert_custi[u]:
                        to = max(vert_custi[u][0],i+t[v][u])
                        if(to%disc_fact!=0):
                            to = to + (disc_fact-to%disc_fact)
                        if to == j :             #  if i+t_vu = j , then add an edge from veriex (v,i)  to  (u,j)  
                            e = ((v,i), (u, j))
                            #print((v,i), (u, j))
                            edges.append(e)
                            edges_out[(v, i)].append((u,j))
                            edges_in[(u, j)].append((v, i))
                            x[e] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{u}_{j}")       # add x[e] variable corresponding to that edge ( Continuous variable between 0 and 1)
                            # adding flow variable for each customer for that edge
                            for c in active_customers:
                                flow[(c, e)] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"flow_{c}_{v}_{i}_{u}_{j}")
    
    # Adding edges from each node (v,i) to t_v  corresponding to v'th customer
    for v in active_customers:
        if(v!=S):
            for i in vert_custi[v]:
                e = ((v, i), (v, vertex_tu_tm[v]))                 # vertex t_v is of form  (v,d[v]+1)
                edges.append(e)
                edges_out[(v, i)].append((v, vertex_tu_tm[v]))
                edges_in[(v, vertex_tu_tm[v])].append((v, i))
                x[e] = model.addVar(lb=1.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{v}_{vertex_tu_tm[v]}")       # its edge capacity value is x[e]=1 corresponding to that edge . (Since these edges are only used to strengthen LP by maximizing submission of flow from these edges to t_v(sink node) for customer v )
                #adding flow variable for each customer for that edge
                for c in active_customers:
                    flow[(c, e)] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"flow_{c}_{v}_{i}_{v}_{vertex_tu_tm[v]}")


    # Add edges from each vertex (v,i) where 1<v<n (no need to add an edge from 0'th or S'th customer to itself) and 0<=i<=d[v]  to the start node (S,0) 
    for v in active_customers:
        if(v!=S):
            for i in vert_custi[v]:
                e = ((v, i), (S, 0))
                edges.append(e)
                edges_out[(v, i)].append((S,0))
                edges_in[(S,0)].append((v, i))
                x[e] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{S}_{0}")
                #adding flow variable for each customer for that edge
                for c in active_customers:
                    flow[(c, e)] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"flow_{c}_{v}_{i}_{S}_{0}")

    model.update()
        
    # Objective: Maximize total flow for each customer c , sum of total incoming flow in t_v (t_v of form (c,d[c]+1) )  
    incoming_flow_tv = []
    for v in active_customers:
        if(v!=S):
            for e in edges_in[(v, vertex_tu_tm[v])]:
                incoming_flow_tv.append(flow[(v,(e,(v,vertex_tu_tm[v])))])
    
    
    model.setObjective(gp.quicksum(incoming_flow_tv), GRB.MAXIMIZE)
    
    
    # Constraints: 
    # x_e for each vertex (v,i) sum incoming = sum outgoing  excluding t_v vertex
    for v, i in vertices:
        if((v,i) not in vertex_tu):
            incomingx = [x[(e,(v,i))] for e in edges_in[(v,i)]] 
            outgoingx = [x[((v,i),e)] for e in edges_out[(v,i)] if e!=(v,vertex_tu_tm[v])]       #not including outgoing edges from (v,i) to t_v , Since edge form vertex (v,i) -> (v,d[v]+1)  (incoming edges to t_v) has x_e=1 , 
            model.addConstr(gp.quicksum(incomingx) == gp.quicksum(outgoingx))
         
    
    # Flow conservation for each customer in every vertex (v,i) sum incoming = sum outgoing
    for c in active_customers:
        for v, i in vertices:
            if (v, i) != (S, 0) and (v, i) not in vertex_tu:              # excluding Start node and t_v node .
                incoming = [flow[(c, (e,(v,i)))] for e in edges_in[(v,i)]]
                outgoing = [flow[(c, ((v,i),e))] for e in edges_out[(v,i)]]
                model.addConstr(gp.quicksum(incoming) == gp.quicksum(outgoing))

            
     # Flow through an edge e for each customer c is between 0 and x_e
    for e in edges:
        for c in active_customers:
            model.addConstr(flow[(c, e)] <= x[e])
            model.addConstr(flow[(c, e)] >= 0)
    
    
    
    # Outgoing sum of x_e from S is 1
    outgoing_S = [x[((S,0),e)] for e in edges_out[(S,0)]]
    model.addConstr(gp.quicksum(outgoing_S) == 1)
    
    model.optimize()
    
    
    
    if model.status == GRB.OPTIMAL:
        #print('Optimal solution found!')
        edge_vals = {}
        G = nx.DiGraph()
        for e in edges:
            val = x[e].X
            if val > 0 and e[1] not in vertex_tu and e[1] != (S, 0):         # excluding edges that are incoming to vertex t_v and edges that are incoming to (S,0) .
                G.add_edge(e[0], e[1], weight=val)
                edge_vals[(e[0], e[1])] = val
                
        max_cust_visit=0
        max_cust_path = {}

        iteration = 0
        while G.number_of_edges() > 0:
            iteration += 1
            try:
                path = nx.dag_longest_path(G)              #gives longest path in graph G .    where path is of form [(v_1,t_1),(v_2,t_2),(v_3,t_3) , ........ ]  and for every vertex (v_i,t_i) comes before (v_(i+1),t_(i+1) ) 
            except nx.NetworkXUnfeasible:
                print("Graph is not a DAG")
                break

            if not path or len(path) < 2:
                break

            # Count how many unique customers visited
            customers_visited = len(set(v for v, _ in path))
            #print(f"Longest path: {path}")
            #print(f"Customers visited: {customers_visited}")
            if(customers_visited>max_cust_visit):
                max_cust_visit=customers_visited
                max_cust_path=path
                
            # Find minimum x_e value along the path
            edge_path = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
            x_min = min(edge_vals[e] for e in edge_path)
            #print(f"Minimum x_e in path: {x_min}")

            # substracting x values along the path by x_min
            for e in edge_path:
                edge_vals[e] -= x_min
                if edge_vals[e] <= 0:           
                    G.remove_edge(*e)
                else:
                    G[e[0]][e[1]]['weight'] = edge_vals[e]

    
        print(f"Objective value: {model.ObjVal}")
        print(f"maximum customer visited: {max_cust_visit}")
        print(f"path: {max_cust_path}")
        
        visited_customers = sorted(set(v for v, _ in max_cust_path if v != S))
        print(f"Customers visited in best path: {visited_customers}")
        return visited_customers
                
                
    else:
        print('No optimal solution found.')
    

'''

# Example usage:
n = 4  # Number of customers
d = [0, 5, 2,9]  # Deadlines for each customer
t = [[0,3,2,9], [3, 0, 3,4], [2, 3, 0,7],[9,4,7,0]]  # Travel times between nodes
S = 0
solver = deadline_vrp_lp(n, d, t,S)


'''

filename = "D:/college/sem_10/cod892/vrp_code/test_cases/solomon/C103.txt"  
data = extract_coordinates_due_date(filename)
#print(result)
n=25                   #len(data)
gamma = 1
S=0      
disc_fact=10
start_time = {i: data[i][2] for i in range(n)}
end_time = {i: data[i][3] for i in range(n)}

#print(start_times)
start_time[S]=0
end_time[S] = 0
    

t = [[0.0 for i in range(n)] for j in range(n)]

dist=0.0
for i in range(0,n):
    for j in range(0,n):
        if(i!=j):
            dist = math.sqrt(pow((data[i][0]-data[j][0]),2) + pow((data[i][1]-data[j][1]),2)) 
            t[i][j]=int(dist*gamma)

#print(t)
all_customers = set(range(n))
solver = deadline_vrp_lp(n, start_time,end_time, t,S,all_customers, disc_fact)

