from gurobipy import *
import time
import numpy as np
import math
import random

# ------------- Data Extraction from File ----------------
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
                    due_date = int(values[5])
                    data.append([xcoord, ycoord, due_date])
                except ValueError:
                    continue
    return np.array(data)

# ------------- Gurobi MILP Model ----------------
def vrp_with_deadlines_gurobi(V, E, t, S, deadlines):
    m = Model("VRP_Deadlines")
    m.Params.OutputFlag = 1
    M = 10000

    # Decision variables
    x = {}  # edge (i,j) used or not
    for i, j in E:
        x[(i, j)] = m.addVar(vtype=GRB.BINARY, name=f"x_{i}_{j}")

    T = {}  # arrival time at node i
    for i in V:
        T[i] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name=f"T_{i}")

    m.update()

    # Objective: Maximize number of nodes visited (edges used)
    m.setObjective(quicksum(x[i, j] for (i, j) in E), GRB.MAXIMIZE)

    # Constraints: ≤ 1 outgoing edge from each node
    for i in V:
        m.addConstr(quicksum(x[i, j] for j in V if (i, j) in E) <= 1, name=f"out_{i}")
    
    # Constraints: ≤ 1 incoming edge to each node
    for i in V:
        m.addConstr(quicksum(x[j, i] for j in V if (j, i) in E) <= 1, name=f"in_{i}")
    
    # Arrival time constraints
    for i in V:
        m.addConstr(T[i] <= deadlines[i], name=f"deadline_{i}")

    # Start time at source is 0
    m.addConstr(T[S] == 0, name="start_time")

    # Flow conservation: incoming ≥ outgoing
    for i in V:
        m.addConstr(
            quicksum(x[j, i] for j in V if (j, i) in E) >= quicksum(x[i, j] for j in V if (i, j) in E),
            name=f"flow_{i}"
        )

    # Time propagation constraints
    for i, j in E:
        if j != S:
            m.addConstr(T[j] >= T[i] + t[i, j] - M * (1 - x[i, j]), name=f"time_{i}_{j}")

    m.update()

    start_time = time.time()
    m.optimize()
    end_time = time.time()

    
    sol_x = {(i, j): x[i, j].X for (i, j) in E if x[i, j].X > 0.0}
    sol_T = {i: T[i].X for i in V}
    return {
        "route": sol_x,
        "visited_nodes": m.objVal,
        "arrival_times": sol_T,
        "time_needed": end_time - start_time
        }
    


filename = "test_cases/t2_n10.txt" 
data = extract_coordinates_due_date(filename)
#print(result)
n=len(data)
gamma = 1
S=0
V=[]
deadlines = {}
for i in range(0,n):
    V.append(i)
    deadlines[i] = data[i][2]
    
#print(deadlines)
deadlines[S]=0

E = []
t = {}

edges = []
dist=0.0
for i in range(0,n):
    for j in range(0,n):
        if(i!=j):
            dist = math.sqrt(pow((data[i][0]-data[j][0]),2) + pow((data[i][1]-data[j][1]),2)) 
            edges.append((i,j))
            t[(i,j)]=math.ceil(dist*gamma)
            
'''
# Shuffle and select a fraction `p` of edges
p=1
random.shuffle(edges)
k = int(p * len(edges))  # number of edges to keep
selected_edges = edges[:k]

for (i, j), dist in selected_edges:
    E.append((i, j))
    t[(i, j)] = math.ceil(dist * gamma)
        
#print(t)   
'''

solution = vrp_with_deadlines_gurobi(V, edges, t, S, deadlines)
if solution:
    print("Visited Nodes:", solution["visited_nodes"])
    print("edges taken:", solution["route"])
    print("Arrival Times:", solution["arrival_times"])
    print("Time :", solution["time_needed"])
