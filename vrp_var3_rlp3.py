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
                    ready_time = int(values[4])
                    due_date = int(values[5])
                    service_time = int(values[6])
                    demand = int(values[3])
                    data.append([xcoord, ycoord,ready_time ,due_date,service_time,demand])
                except ValueError:
                    continue
    return np.array(data)

def deadline_vrp_lp_gurobi(n,start_time,end_time,service_time,demands, C,t, S,active_customers,disc_fact,disc_demand_fact):
    model = gp.Model("VRP_MinVehicles_Deadline_demands_rlp")
    model.Params.OutputFlag = 1
    x = {}
    flow = {}
    edges = []
    
    vertices = set()
    vert_custi = {}
    vertex_tu_tm = {}
    vertex_tu = set()

    edges_in = {}
    edges_out = {}
   
    vertices.add((S,0,0))
    edges_in[(S,0,0)]=[]
    edges_out[(S,0,0)]=[]
    
    disc_demand = {}
    for v in active_customers:
        arr = []
        ed = demands[v]
        if(ed%disc_demand_fact!=0):
            ed=ed+(disc_demand_fact-ed%disc_demand_fact)
            
        i=0
        while(i<=ed):
            arr.append(i)
            i=i+disc_demand_fact
        
        disc_demand[v]=arr
   
    #print(vertices)
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
            #for k in disc_demand[v]:
            k=0
            while(k<=C):
                vertices.add((v,i,k))
                edges_in[(v,i,k)]=[]
                edges_out[(v,i,k)]=[]
                k=k+disc_demand_fact
        
            arr.append(i)
            i=i+disc_fact
            
        vert_custi[v]=arr   
        sink_node=(v,i,k)
        vertex_tu.add(sink_node)
        vertex_tu_tm[v]=i
        edges_in[sink_node]=[]
        edges_out[sink_node]=[]
        
    
    # add edge (S,0,0) -> (v,t_0v,r)
    for v in active_customers:
        to = max(vert_custi[v][0],t[0][v])      #+service_time[v]
        if(to%disc_fact!=0):
            to = to + (disc_fact-to%disc_fact)
        for r in disc_demand[v]:
            if(r>0):
                e=((S,0,0),(v,to,r))
                edges.append(e)
                edges_in[(v,to,r)].append((S,0,0))
                edges_out[(S,0,0)].append((v,to,r))
                x[e] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{S}_{0}_{0}_{v}_{t[S][v]}_{r}")
                #x[e] = model.addVar(vtype=GRB.BINARY, name=f"x_{S}_{0}_{0}_{v}_{t[S][v]}_{r}")
            
                
    # adding edge  e = ((v,i,p) -> (u,j,r))     if i+t_vu = j  .
    for v in active_customers:
        for i in vert_custi[v]:
            #for p in disc_demand[v]:
            p=0
            while(p<=C):
                for u in active_customers:
                    if u != v:
                        for j in vert_custi[u]:
                            to = max(vert_custi[u][0],i+t[v][u])      #+service_time[v]
                            if(to%disc_fact!=0):
                                to = to + (disc_fact-to%disc_fact)
                            if to == j :       #  if i+t_vu = j , then add an edge from veriex (v,i)  to  (u,j) 
                                #for r in disc_demand[u]:  #range(p+1,min(C,p+demands[u])+1):    
                                r=p+disc_demand_fact
                                stp = min(C,p+disc_demand[u][-1])
                                while(r<stp):         
                                    e = ((v, i, p), (u, j, r))
                                    edges.append(e)
                                    edges_in[(u, j, r)].append((v, i, p))
                                    edges_out[(v, i, p)].append((u, j, r))
                                    x[e] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{p}_{u}_{j}_{r}")       # add x[e] variable corresponding to that edge ( Continuous variable between 0 and 1)
                                    #x[e] = model.addVar(vtype=GRB.BINARY, name=f"x_{v}_{i}_{p}_{u}_{j}_{r}")
                                    r=r+disc_demand_fact
                
                p=p+disc_demand_fact
                



    # Add edges from each vertex (v,i,p) where 1<v<n (no need to add an edge from 0'th or S'th customer to itself) and 0<=i<=d[v]  to the start node (S,0) 
    for v in active_customers:
        for i in vert_custi[v]:
            #for j in disc_demand[v]:
            j=0
            while(j<=C):
                e = ((v, i, j), (S, 0, 0))
                edges.append(e)
                edges_in[(S, 0, 0)].append((v, i, j))
                edges_out[(v, i, j)].append((S, 0, 0))
                x[e] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{j}_{S}_{0}_{0}")
                #x[e] = model.addVar(vtype=GRB.BINARY, name=f"x_{v}_{i}_{j}_{S}_{0}_{0}")
                j=j+disc_demand_fact
                
                
    # Adding edges from each node (v,i,p) to sink node t_v  corresponding to each v'th customer
    for v in active_customers:
        if(v!=S):
            for i in vert_custi[v]:
                j=0
                while(j<=C):
                    e = ((v, i, j), (v, vertex_tu_tm[v],C+disc_demand_fact))                 # vertex t_v is of form  (v,d[v]+1)
                    edges.append(e)
                    edges_out[(v, i, j)].append((v, vertex_tu_tm[v],C+disc_demand_fact))
                    edges_in[(v, vertex_tu_tm[v],C+disc_demand_fact)].append((v, i, j))
                    x[e] = model.addVar(lb=1.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"x_{v}_{i}_{j}_{v}_{vertex_tu_tm[v]}_{C+disc_demand_fact}")       # its edge capacity value is x[e]=1 corresponding to that edge . (Since these edges are only used to strengthen LP by maximizing submission of flow from these edges to t_v(sink node) for customer v )
                    #adding flow variable for each customer for that edge
                    for c in active_customers:
                        flow[(c, e)] = model.addVar(lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name=f"flow_{c}_{v}_{i}_{j}_{v}_{vertex_tu_tm[v]}_{C+disc_demand_fact}")
                        
                    j=j+disc_demand_fact


                
    model.update()
    
    

    #objective
    model.setObjective(gp.quicksum(x[((S,0,0),e)] for e in edges_out[(S,0,0)]), GRB.MINIMIZE)

    # Constraints: 
    
    '''
    # 1. Customer coverage constraint   ,  #incoming x_e for every customer = 1 
    for v in active_customers:
        if(v!=S):
            model.addConstr(gp.quicksum(x[e] for e in edges if e[1][0] == v) == 1, name=f"visit_{v}")
    '''
    #print(len(vertices))
    #2.  x_e for each vertex (v,i,j) , sum of incoming  = sum of outgoing 
    for v, i, j in vertices:
        if (v, i, j) not in vertex_tu:
            #incomingx = [x[e] for e in edges if e[1] == (v, i)]
            #outgoingx = [x[e] for e in edges if e[0] == (v, i) and e[1] != (v, d[v] + 1)]         # not including outgoing edges from (v,i) to t_v , Since edge form vertex (v,i) -> (v,d[v]+1)  (incoming edges to t_v) has x_e=1 , and this edges are only used for strengthen LP by maximizing flow .
            incomingx = [x[(e,(v,i,j))] for e in edges_in[(v,i,j)]] 
            outgoingx = [x[((v,i,j),e)] for e in edges_out[(v,i,j)] if e!=(v,vertex_tu_tm[v],C+disc_demand)]
            model.addConstr(gp.quicksum(incomingx) == gp.quicksum(outgoingx))

            
            
    # Flow conservation for each customer in every vertex (v,i,j) sum incoming = sum outgoing
    for c in active_customers:
        for v,i,j in vertices:
            if (v, i, j) != (S, 0, 0) and (v, i, j) not in vertex_tu:              # excluding Start node and t_v node .
                incoming = [flow[(c, (e,(v,i,j)))] for e in edges_in[(v,i,j)]]
                outgoing = [flow[(c, ((v,i,j),e))] for e in edges_out[(v,i,j)]]
                model.addConstr(gp.quicksum(incoming) == gp.quicksum(outgoing))


    # Flow through an edge e for each customer c is between 0 and x_e
    for e in edges:
        for c in active_customers:
            model.addConstr(flow[(c, e)] <= x[e])
            model.addConstr(flow[(c, e)] >= 0)
            
    # incoming flow in t_v = 1
    for v in active_customers:
        if(v!=S):
            sink_node = (v, vertex_tu_tm[v],C+disc_demand_fact)
            incoming_edges_flow = [flow[(v,(e,sink_node))] for e in edges_in[sink_node]]
            model.addConstr(gp.quicksum(incoming_edges_flow) == 1, name=f"flow_to_sink_{v}")

     
  
  
    start_time = time.time()
    model.optimize()
    end_time = time.time()

    if model.status == GRB.OPTIMAL:
        # We will use an iterative rounding procedure using this strengthened LP formulation to obtain solution
        
        satisfied_customers = set(v for v in active_customers if demands[v] <= 0)
        remaining_demands = demands.copy()
        demand_served={}
        
        # Constructing a graph with x_e > 0 
        edge_vals = {}
        G = nx.DiGraph()
        for e in edges:
            val = x[e].X
            if val > 0 and e[1] != (S, 0, 0):         # excluding edges that are incoming to (S,0) .
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
            customers_visited = len(set(v for v, _, _ in path))
            #print(f"Longest path: {path}")
            #print(f"Customers visited: {customers_visited}")
            if(customers_visited>max_cust_visit):
                max_cust_visit=customers_visited
                max_cust_path=path
                demand_served = {v: 0 for v in active_customers}
                edge_path = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
                for e in edge_path:
                    v1, t1, u1 = e[0]
                    v2, t2, u2 = e[1]
                    if v2 != S:  # not depot
                        served = u2 - u1
                        demand_served[v2] += served
                
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
              
              
        # Deduct demand from customers
        for cust, served in demand_served.items():
            if cust in remaining_demands:
                remaining_demands[cust] -= served
                if remaining_demands[cust] <= 0:
                    satisfied_customers.add(cust)    
        
        print(f" Remaining demands after iteration {iteration}: {remaining_demands}")
            
                      
        '''

        #print(f"Time taken: {end_time - start_time:.2f} seconds")
        print(f"Objective value: {model.ObjVal}")
        '''
        
        print(f"maximum customer visited: {max_cust_visit}")
        print(f"path: {max_cust_path}")
        
        visited_customers = sorted(set(v for v, _, _ in max_cust_path if v != S))
        print(f"Customers visited in best path: {visited_customers}")
        return (satisfied_customers,remaining_demands)  
              
    else:
        print("No optimal solution found.")
        return []




filename = "D:/college/sem_10/cod892/vrp_code/test_cases/solomon/C102.txt"
data = extract_coordinates_due_date(filename)
n = 101
S = 0
C=200
gamma = 1
disc_fact=10
disc_demand_fact=10
demands = {i: data[i][5] for i in range(n)}
#print(demands)
start_time = {i: data[i][2] for i in range(n)}
end_time = {i: data[i][3] for i in range(n)}
service_time = {i: data[i][4] for i in range(n)}
#print(start_times)
start_time[S]=0
end_time[S] = 0


t = [[0 for _ in range(n)] for _ in range(n)]
for i in range(n):
    for j in range(n):
        if i != j:
            dist = math.sqrt((data[i][0] - data[j][0])**2 + (data[i][1] - data[j][1])**2)
            t[i][j] = int(dist*gamma)

active_cust = set(range(1,n))
#print(active_cust)

#all_customers = set(range(1,n))
#deadline_vrp_lp_gurobi(n,start_time,end_time,service_time,demands,C,t,S,all_customers,disc_fact,disc_demand_fact)


def solve_all_customers_iteratively(n,start_time,end_time,service_time,demands,C,t,S,disc_fact,disc_demand_fact):
    
    unvisited_customers = set(range(1,n))
    full_visited_set = []
    round_num = 0
    st_times = time.time()
    print(unvisited_customers)
    while len(unvisited_customers)>0:
        print(f"\n--- Iteration {round_num} ---")
        unvisited_customers.add(S)
        active_customers = unvisited_customers  #list(unvisited_customers.union({S}))  # include S each time
        visited = deadline_vrp_lp_gurobi(n,start_time,end_time,service_time,demands,C,t,S,active_customers,disc_fact,disc_demand_fact)

        if not visited:
            print("Infeasible or no customers visited in this iteration. Stopping.")
            break

        print(f"Visited in iteration {round_num}: {visited}")
        full_visited_set.append(visited)

        # Remove visited customers from unvisited list
        unvisited_customers.difference_update(visited)
        #print(f"unvisited: {unvisited_customers}")
        round_num += 1
        
    ed_times = time.time()
    print(f"Time taken: {ed_times - st_times:.2f} seconds")   
    print(f"no of vehicles: {round_num} ")

    return full_visited_set



solve_all_customers_iteratively(n,start_time,end_time,service_time,demands,C,t,S,disc_fact,disc_demand_fact)




