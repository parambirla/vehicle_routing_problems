import gurobipy as gp
from gurobipy import GRB
import numpy as np
import math
import networkx as nx
import time

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

def deadline_vrp_lp_gurobi(n, d, t, S):
    model = gp.Model("Deadline_VRP")
    model.Params.OutputFlag = 1
    x = {}
    flow = {}
    edges = []
    
    # for each customer v , there will be vertices of form { (v,i)  | s.t.  0 <= i <= d_i  }  
    vertices = {(v, i) for v in range(n) for i in range(d[v] + 1)}
    
    # vertex_tu contains sink vertices (t_u) corresponding to each customer u .(only to strenthen LP by sending max flow to t_u)
    vertex_tu = set()
    for i in range(n):
        vertices.add((i, d[i] + 1))
        vertex_tu.add((i, d[i] + 1))

    # adding edge  e = ((v,i) -> (u,j))     if i+t_vu = j  .
    for v in range(n):
        for i in range(d[v] + 1):
            for u in range(n):
                if u != v:
                    for j in range(d[u] + 1):
                        if i + t[v][u] == j :             #  if i+t_vu = j , then add an edge from veriex (v,i)  to  (u,j)  
                            e = ((v, i), (u, j))
                            edges.append(e)
                            x[e] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{u}_{j}")       # add x[e] variable corresponding to that edge ( Continuous variable between 0 and 1)
                            # adding flow variable for each customer for that edge
                            for c in range(n):
                                flow[(c, e)] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"flow_{c}_{v}_{i}_{u}_{j}")

    # Adding edges from each node (v,i) to sink node t_v  corresponding to each v'th customer
    for v in range(1, n):
        for i in range(d[v] + 1):
            e = ((v, i), (v, d[v] + 1))                 # vertex t_v is of form  (v,d[v]+1)
            edges.append(e)
            x[e] = model.addVar(lb=1.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{v}_{d[v]+1}")       # its edge capacity value is x[e]=1 corresponding to that edge . (Since these edges are only used to strengthen LP by maximizing submission of flow from these edges to t_v(sink node) for customer v )
            #adding flow variable for each customer for that edge
            for c in range(n):
                flow[(c, e)] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"flow_{c}_{v}_{i}_{v}_{d[v]+1}")


    # Add edges from each vertex (v,i) where 1<v<n (no need to add an edge from 0'th or S'th customer to itself) and 0<=i<=d[v]  to the start node (S,0) 
    for v in range(1, n):
        for i in range(d[v] + 1):
            e = ((v, i), (S, 0))
            edges.append(e)
            x[e] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{S}_{0}")
            #adding flow variable for each customer for that edge
            for c in range(n):
                flow[(c, e)] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"flow_{c}_{v}_{i}_{S}_{0}")

    model.update()

    # Objective: Maximize total flow for each customer c , sum of total incoming flow in t_v (t_v of form (c,d[c]+1) )  
    objective = gp.quicksum(flow[(c, e)] for c in range(1, n) for e in edges if e[1] == (c, d[c] + 1))
    model.setObjective(objective, GRB.MAXIMIZE)

    # Constraints: 
    # x_e for each vertex (v,i) , sum of incoming  = sum of outgoing ( excluding t_v vertex )
    for v, i in vertices:
        if (v, i) not in vertex_tu:
            incomingx = [x[e] for e in edges if e[1] == (v, i)]
            outgoingx = [x[e] for e in edges if e[0] == (v, i) and e[1] != (v, d[v] + 1)]         # not including outgoing edges from (v,i) to t_v , Since edge form vertex (v,i) -> (v,d[v]+1)  (incoming edges to t_v) has x_e=1 , and this edges are only used for strengthen LP by maximizing flow .
            model.addConstr(gp.quicksum(incomingx) == gp.quicksum(outgoingx))

    # Flow conservation for each customer in every vertex (v,i) sum incoming = sum outgoing
    for c in range(n):
        for v, i in vertices:
            if (v, i) != (S, 0) and (v, i) not in vertex_tu:              # excluding Start node and t_v node .
                incoming = [flow[(c, e)] for e in edges if e[1] == (v, i)]
                outgoing = [flow[(c, e)] for e in edges if e[0] == (v, i)]
                model.addConstr(gp.quicksum(incoming) == gp.quicksum(outgoing))


    # Flow through an edge e for each customer c is between 0 and x_e
    for e in edges:
        for c in range(n):
            model.addConstr(flow[(c, e)] <= x[e])
            model.addConstr(flow[(c, e)] >= 0)

    # Outgoing sum of x_e from S is 1
    model.addConstr(gp.quicksum(x[e] for e in edges if e[0] == (S, 0)) == 1)

    start_time = time.time()
    model.optimize()
    end_time = time.time()

    if model.status == GRB.OPTIMAL:
        # We will use an iterative rounding procedure using this strengthened LP formulation to obtain solution
        
        # Constructing a graph with x_e > 0 
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
            print(f"Longest path: {path}")
            print(f"Customers visited: {customers_visited}")
            if(customers_visited>max_cust_visit):
                max_cust_visit=customers_visited
                max_cust_path=path
                
            # Find minimum x_e value along the path
            edge_path = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
            x_min = min(edge_vals[e] for e in edge_path)
            print(f"Minimum x_e in path: {x_min}")

            # substracting x values along the path by x_min
            for e in edge_path:
                edge_vals[e] -= x_min
                if edge_vals[e] <= 0:           
                    G.remove_edge(*e)
                else:
                    G[e[0]][e[1]]['weight'] = edge_vals[e]

        print(f"Time taken: {end_time - start_time:.2f} seconds")
        print(f"Objective value: {model.ObjVal}")
        print(f"maximum customer visited: {max_cust_visit}")
        print(f"path: {max_cust_path}")
              
    else:
        print("No optimal solution found.")
    return model



filename = "test_cases/t3_n10.txt"
data = extract_coordinates_due_date(filename)
n = len(data)
S = 0
gamma = 1
deadlines = {i: data[i][2] for i in range(n)}
deadlines[S] = 0

t = [[0 for _ in range(n)] for _ in range(n)]
for i in range(n):
    for j in range(n):
        if i != j:
            dist = math.sqrt((data[i][0] - data[j][0])**2 + (data[i][1] - data[j][1])**2)
            t[i][j] = math.ceil(dist * gamma)

deadline_vrp_lp_gurobi(n, deadlines, t, S)
