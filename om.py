import time
from heapq import *
import numpy as np
from math import sqrt, inf, isinf
# from math import inf, isinf
# from operator import itemgetter

from utils.search_methods import FRASTAR, FRASTAR_3D
from utils.class_definitions import Graph, Edge
from utils.utility_functions import find_MinVertexCover, get_initial_match, maximize_match


left_vertices = set()
right_vertices = set()
path = {}
no_of_explorations = 0

OL_dict = {}
CL_dict = {}
distance_lookup = {}


t1 = 0
t2 = 0

# ---------------------------------------------------------------------------------------------------------------------
# collect_uncovered_costs : Function to collect delta (weight) for edges between uncovered robots and uncovered goals
# ---------------------------------------------------------------------------------------------------------------------

def collect_uncovered_costs(G, mvc):
	

	# Collecting delta:

	uncovered_robots = left_vertices - mvc
	uncovered_goals = right_vertices - mvc
	
	
	deltas = []
	for i in uncovered_robots:
		for j in uncovered_goals: 
			edge = G.vertices[i].incident_edges[j]
			# slack = edge.weight - ( G.vertices[i].label + G.vertices[j].label )
			# if not math.isinf(delta):
			# deltas.append( ( i, j, slack, edge.typeOfWeight ) )
			# DELTA.append( ( edge.weight, slack, edge.typeOfWeight, i, j ) )  # c77
			deltas.append( ( edge.weight, edge.typeOfWeight, i, j ) )  # c77

	
	heapify(deltas) # c77
			
	return deltas

# **********************************************************************************************************************




# ----------------------------------------------------------------------------------------------------------------------
# get_min_delta (get_global_delta_min) : This function fetches the minimum delta (cost / slack) value between pairs of uncov robots & goals
# ----------------------------------------------------------------------------------------------------------------------


# Optimized

def get_min_delta( deltas, G, workSpace, all_start_loc, all_goal_loc, ws_type, makespan = inf, makespan_flag = False ):

	global no_of_explorations, OL_dict, CL_dict, distance_lookup

	while (1):
		
		d_min = deltas[0] # c77

		if d_min[1] == 'a':
			
			if makespan_flag == True:
			
				if makespan < d_min[0]:
					makespan = d_min[0]

				return makespan, G
			
			else:
				
				return d_min[0], G

		else:
			# Explore

			edge = G.vertices[d_min[2]].incident_edges[d_min[3]]  # edge variable will store the Edge object
			if d_min[2][0] == 'r':
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[d_min[0]], all_goal_loc[d_min[1]], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]] = FRASTAR_3D( workSpace, all_start_loc[d_min[2]], all_goal_loc[d_min[3]], OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]], rtype = 'pathAndCost' )
				else:
					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[d_min[0]], all_goal_loc[d_min[1]], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]] = FRASTAR( workSpace, all_start_loc[d_min[2]], all_goal_loc[d_min[3]], OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]], rtype = 'pathAndCost' )
				path[d_min[2]][d_min[3]] = one_path
			else:
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]] = FRASTAR_3D( workSpace, all_goal_loc[d_min[2]], all_start_loc[d_min[3]], OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]], rtype = 'pathAndCost' )
				else:
					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], rtype = 'pathAndCost' )
					# one_path, edge.weight, OL_dict[d_min[0]], CL_dict[d_min[0]] = FRASTAR( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], OL_dict[d_min[0]], CL_dict[d_min[0]], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]] = FRASTAR( workSpace, all_goal_loc[d_min[2]], all_start_loc[d_min[3]], OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]], rtype = 'pathAndCost' )
				path[d_min[3]][d_min[2]] = one_path[::-1]
			
			edge.typeOfWeight = 'a'
			no_of_explorations = no_of_explorations + 1
			

			# new_slack = edge.weight - ( G.vertices[d_min[3]].label + G.vertices[d_min[4]].label )
			
			if makespan_flag == False:
				new_slack = edge.weight - ( G.vertices[d_min[2]].label + G.vertices[d_min[3]].label )
				new_delta = ( new_slack, 'a', d_min[2], d_min[3] )
			else:
				new_delta = ( edge.weight, 'a', d_min[2], d_min[3] )

			if ( edge.weight <= makespan ) or ( makespan_flag == True ):
				heapreplace( deltas, new_delta )
			else:
				heappop(deltas)
				# Here, we are in Hungarian zone, so remove the edge having wt > makespan from G_th, as G_th is being processed by Hungarian method and we don't need edges.wt > makespan
				G.vertices[d_min[2]].neighbors.discard(d_min[3])
				G.vertices[d_min[3]].neighbors.discard(d_min[2])
				G.vertices[d_min[2]].incident_edges.pop(d_min[3])
				G.vertices[d_min[3]].incident_edges.pop(d_min[2])



	
# **********************************************************************************************************************


# ---------------------------------------------------------------------------------------------------------------------
# 
# ---------------------------------------------------------------------------------------------------------------------


def find_min_Zcost( x, type_of_wt ):  # receives a Vertex object as parameter, Z refers to 'h' or 'a'

	minZcost = inf
	minZindex = None

	for edge in x.incident_edges.values():
		if ( edge.typeOfWeight == type_of_wt ) and ( edge.weight < minZcost ):
			minZcost = edge.weight
			minZindex = edge.get_pair_vertex(x)  # minZindex carries goal g


	return minZcost, minZindex

# **********************************************************************************************************************


# ---------------------------------------------------------------------------------------------------------------------
# Function to explore min actual cost for each robot (ROW wise)
# ---------------------------------------------------------------------------------------------------------------------

def explore_min_actual_cost_rowwise( G, workSpace, all_start_loc, all_goal_loc, ws_type ):

	minAcost = {}
	minAindex = {}
	global no_of_explorations, OL_dict, CL_dict, distance_lookup

	
	for x in left_vertices:    # for each robot
		
		OL_dict[x] = []
		CL_dict[x] = {}

		if ws_type == '3D':
			distance_lookup[x] = np.empty(shape=(workSpace.shape[0], workSpace.shape[1], workSpace.shape[2]))
		else:
			distance_lookup[x] = np.empty(shape=(workSpace.shape[0], workSpace.shape[1]))
		
		distance_lookup[x].fill(inf)

		

		minHcost, minHindex = find_min_Zcost( G.vertices[x], 'h' )
		
		minAcost[x] = inf
		minAindex[x] = None
		
		
		while ( minHindex != None and minHcost <= minAcost[x] ):
			
			edge = G.vertices[x].incident_edges[minHindex]
		
			if x[0] == 'r':
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[minHindex], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[minHindex], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
				else:
					one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_start_loc[x], all_goal_loc[minHindex], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
				path[x][minHindex] = one_path
			else:
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[minHindex], all_goal_loc[x], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_goal_loc[x], all_start_loc[minHindex], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
				else:
					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[minHindex], all_goal_loc[x], rtype = 'pathAndCost' )
					# Commented below for incorrect R > G costs. 
					# one_path, edge.weight, OL_dict[x], CL_dict[x] = FRASTAR( workSpace, all_start_loc[minHindex], all_goal_loc[x], OL_dict[x], CL_dict[x], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_goal_loc[x], all_start_loc[minHindex], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
				path[minHindex][x] = one_path[::-1]
			
			edge.typeOfWeight = 'a'
			no_of_explorations = no_of_explorations + 1

			minHcost, minHindex = find_min_Zcost( G.vertices[x], 'h' )
			minAcost[x], minAindex[x] = find_min_Zcost( G.vertices[x], 'a' )


	return G, minAcost

# **********************************************************************************************************************

# colwise 

# ---------------------------------------------------------------------------------------------------------------------
# Function to explore min actual cost for each goal (COLUMNWISE wise)
# ---------------------------------------------------------------------------------------------------------------------

def explore_min_actual_cost_columnwise( G, workSpace, all_start_loc, all_goal_loc, ws_type ):

	minAcost = {}
	minAindex = {}
	global no_of_explorations, OL_dict, CL_dict, distance_lookup

	
	for x in right_vertices:    # for each robot
		
		minHcost, minHindex = find_min_Zcost( G.vertices[x], 'h' )  # for R <=G, here Hindex will have robot
		
		minAcost[x], minAindex[x] = find_min_Zcost( G.vertices[x], 'a' )

		
		
		while ( minHindex != None and minHcost <= minAcost[x] ):
			
			edge = G.vertices[minHindex].incident_edges[x]
			
			if minHindex[0] == 'r':
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[minHindex], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[minHindex], CL_dict[minHindex], distance_lookup[minHindex] = FRASTAR_3D( workSpace, all_start_loc[minHindex], all_goal_loc[x], OL_dict[minHindex], CL_dict[minHindex], distance_lookup[minHindex], rtype = 'pathAndCost' )
				else:
					one_path, edge.weight, OL_dict[minHindex], CL_dict[minHindex], distance_lookup[minHindex] = FRASTAR( workSpace, all_start_loc[minHindex], all_goal_loc[x], OL_dict[minHindex], CL_dict[minHindex], distance_lookup[minHindex], rtype = 'pathAndCost' )
				path[minHindex][x] = one_path
			else:
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[minHindex], all_goal_loc[x], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[minHindex], CL_dict[minHindex], distance_lookup[minHindex] = FRASTAR_3D( workSpace, all_goal_loc[minHindex], all_start_loc[x], OL_dict[minHindex], CL_dict[minHindex], distance_lookup[minHindex], rtype = 'pathAndCost' )
				else:
					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[minHindex], all_goal_loc[x], rtype = 'pathAndCost' )
					# Commented below for incorrect R > G costs. 
					# one_path, edge.weight, OL_dict[x], CL_dict[x] = FRASTAR( workSpace, all_start_loc[minHindex], all_goal_loc[x], OL_dict[x], CL_dict[x], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[minHindex], CL_dict[minHindex], distance_lookup[minHindex] = FRASTAR( workSpace, all_goal_loc[minHindex], all_start_loc[x], OL_dict[minHindex], CL_dict[minHindex], distance_lookup[minHindex], rtype = 'pathAndCost' )
				path[x][minHindex] = one_path[::-1]
			
			edge.typeOfWeight = 'a'
			no_of_explorations = no_of_explorations + 1

			minHcost, minHindex = find_min_Zcost( G.vertices[x], 'h' )
			minAcost[x], minAindex[x] = find_min_Zcost( G.vertices[x], 'a' )


	return G, minAcost

# **********************************************************************************************************************







# Get threshold graph - heuristic case

def get__threshold_subgraph_H( G, makespan, workSpace, all_start_loc, all_goal_loc, ws_type ):

	
	global no_of_explorations, OL_dict, CL_dict, distance_lookup

	G_th = Graph( eq_flag = True )
	
	
	# Creating vertices
	for x in left_vertices:
		G_th.add_vertex( x, G.vertices[x].label, G.vertices[x].in_left )

	for y in right_vertices:
		G_th.add_vertex( y, G.vertices[y].label, G.vertices[y].in_left )
	

	# Creating edges
	for x in left_vertices:
		for y in right_vertices:

			edge = G.vertices[x].incident_edges[y]  # edge variable will store the Edge object

			
			if ( edge.typeOfWeight == 'h' ) and (edge.weight <= makespan):
				# We need to confirm whether the A-cost is really <= makespan; if yes, the edge must be allowed in the threshold graph, else no
				if x[0] == 'r':
					if ws_type == '3D':
						one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
					else:
						one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_start_loc[x], all_goal_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
					path[x][y] = one_path
				else:
					if ws_type == '3D':
						one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_goal_loc[x], all_start_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
					else:
						one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_goal_loc[x], all_start_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
					path[y][x] = one_path[::-1]
				
				edge.typeOfWeight = 'a'
				no_of_explorations = no_of_explorations + 1


			
			
			if ( edge.typeOfWeight != 'h' ) and (edge.weight <= makespan):
				# add edge
				if x[0] == 'r':
					e = Edge(x, y, edge.weight)
				else:
					e = Edge(y, x, edge.weight)
				e.typeOfWeight = edge.typeOfWeight


				G_th.vertices[x].neighbors.add(y)
				G_th.vertices[y].neighbors.add(x)
				G_th.vertices[x].incident_edges[y] = e
				G_th.vertices[y].incident_edges[x] = e


	return G_th, G


# Update threshold graph - heuristic case

def update__threshold_subgraph_H( G_th, G, mvc, makespan, workSpace, all_start_loc, all_goal_loc, ws_type ):


	global no_of_explorations, OL_dict, CL_dict, distance_lookup

	# Revising edges between uncovered robot - uncovered goal pairs (new edges may appear)

	uncovered_robots = left_vertices - mvc
	uncovered_goals = right_vertices - mvc
	

	for x in uncovered_robots:
		for y in uncovered_goals:

			edge = G.vertices[x].incident_edges[y]  # edge variable will store the Edge object

			
			if ( edge.typeOfWeight == 'h' ) and (edge.weight <= makespan):  # if min uncov wt < makespan, then == will prevent it to come in G_th, so we should have <= 
				if x[0] == 'r':
					# Explore A.cost
					if ws_type == '3D':
						one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
					else:
						one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_start_loc[x], all_goal_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
					path[x][y] = one_path
				else:
					if ws_type == '3D':
						one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_goal_loc[x], all_start_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
					else:
						one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_goal_loc[x], all_start_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
					path[y][x] = one_path[::-1]
				
				edge.typeOfWeight = 'a'
				no_of_explorations = no_of_explorations + 1

			
			
			
			if ( edge.typeOfWeight != 'h' ) and (edge.weight <= makespan):
				# add edge
				if x[0] == 'r':
					e = Edge(x, y, edge.weight)
				else:
					e = Edge(y, x, edge.weight)
				e.typeOfWeight = edge.typeOfWeight


				G_th.vertices[x].neighbors.add(y)
				G_th.vertices[y].neighbors.add(x)
				G_th.vertices[x].incident_edges[y] = e
				G_th.vertices[y].incident_edges[x] = e

	
	return G_th, G


# ---------------------------------------------------------------------------------------------------------------------


# ---------------------------------------------------------------------------------------------------------------------
# main function starts
# ---------------------------------------------------------------------------------------------------------------------


def find_assignment_H( _G, workSpace, all_start_loc, all_goal_loc, ws_type ):
	

	global no_of_explorations, OL_dict, CL_dict, distance_lookup, t1, t2
	no_of_explorations = 0
	path.clear()
	left_vertices.clear()
	right_vertices.clear()
	OL_dict.clear()
	CL_dict.clear()
	distance_lookup.clear()
	
	
	# Construct a bipartite graph 
	
	G = Graph( _G, heur = True )
	
		
	for x in G.vertices:
		if G.vertices[x].in_left:
			left_vertices.add(x)
			if x[0] == 'r':
				path[x] = {}
		else:
			right_vertices.add(x)
			if x[0] == 'r':
				path[x] = {}
	


	no_of_robots = len( all_start_loc )
	no_of_goals = len( all_goal_loc )



	G, minAcost = explore_min_actual_cost_rowwise( G, workSpace, all_start_loc, all_goal_loc, ws_type )
	
	if no_of_robots == no_of_goals:

		G, minAcost_col = explore_min_actual_cost_columnwise( G, workSpace, all_start_loc, all_goal_loc, ws_type )
		all_min = np.array( list(minAcost.values()) + list(minAcost_col.values()) )
	else:
		all_min = np.array( list(minAcost.values()) )
	
	
	# Get initial makespan
	# makespan = np.max( list(minAcost.values()) + list(minAcost_col.values()) )
	
	try:
		makespan = np.max(all_min[np.isfinite(all_min)])  # a robot (goal) can be trapped in case of unequal R & G. So, we need to ignore infinite cost for such entity
	except ValueError:
		return None, None, None, None, None, None, None, None


	# # Generate an initial feasible labeling
	# G.generate_feasible_labeling_H( left_vertices, right_vertices, minAcost ) 


	# Get the threshold subgraph
	G_th, G = get__threshold_subgraph_H( G, makespan, workSpace, all_start_loc, all_goal_loc, ws_type )

	
	# Finding an initial matching
	M = get_initial_match(G_th, left_vertices)

	# Maximize the current matching
	M = maximize_match(G_th, M, left_vertices)

	
	# change 999 - for handling case when no of robots != no of goals
	min_robots_goals = min( len(left_vertices), len(right_vertices) )  # change 999

	
	# while len(M) < int(len(eq_G.vertices)/2):   # change 999
	while len(M) < min_robots_goals:

		mvc = find_MinVertexCover(G_th, M, left_vertices, right_vertices)
		
	
		deltas = collect_uncovered_costs(G, mvc)
		makespan, G = get_min_delta( deltas, G, workSpace, all_start_loc, all_goal_loc, ws_type, makespan, True )
			
		
		G_th, G = update__threshold_subgraph_H( G_th, G, mvc, makespan, workSpace, all_start_loc, all_goal_loc, ws_type )
		
		M = maximize_match(G_th, M, left_vertices)
		


	# print("Final MAKESPAN: ", makespan, "\n")

	
	t2 = time.time()

	
	# Save a copy of nexp until now, so that it will represent # exp needed to get optimal makespan
	makespan_nexp = no_of_explorations
	
	
	# counting number of nodes expanded BEGINS (for optimal makespan)
	makespan_CL = 0

	for robo in CL_dict:
		makespan_CL = makespan_CL + len(CL_dict[robo])
	
	

	# ------------------ End Result -----------------------

	total = 0
	unassigned_count = 0
	assigned_count = 0
	if (len(M) != 0):
		for e in M:
			if not isinf( e.weight ):
				total = total + e.weight
				assigned_count = assigned_count + 1
	

	unassigned_count = min_robots_goals - assigned_count
		
	
	return list(map(lambda e: ((e.vertices[0], e.vertices[1]), e.weight), M)), total, path, unassigned_count, makespan_nexp, makespan, makespan_CL
	
# **********************************************************************************************************************



# --------------------------------------------------------------------------------------------------------------
#                          Directly - exposed function for Heuristic-based approach
# --------------------------------------------------------------------------------------------------------------

def heuristic ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type  ):


	global t1, t2
	
	t1 = time.time()

	costMatrix = np.zeros((no_of_robots, no_of_goals))
	# Generating heuristic costs for all pairs of robots and goals
	for i in range(no_of_robots):
		robot_name = 'r' + str(i)
		for j in range(no_of_goals):
			goal_name = 'g' + str(j)
			
			if ws_type == '3D':
				sq_of_distance = ((all_start_loc[robot_name][0] - all_goal_loc[goal_name][0]) ** 2) + (
				(all_start_loc[robot_name][1] - all_goal_loc[goal_name][1]) ** 2) + (
					(all_start_loc[robot_name][2] - all_goal_loc[goal_name][2]) ** 2)
			else:
				sq_of_distance = ((all_start_loc[robot_name][0] - all_goal_loc[goal_name][0]) ** 2) + (
					(all_start_loc[robot_name][1] - all_goal_loc[goal_name][1]) ** 2)

			# costMatrix[i][j] = float(format(sqrt(sq_of_distance), '.2f'))      
			costMatrix[i][j] = sqrt(sq_of_distance)
			

		
	result_h, total_cost_h, path_h, u_count_h, makespan_nexp_h, makespan, makespan_CL = find_assignment_H( costMatrix, workSpace, all_start_loc, all_goal_loc, ws_type )

	
	if result_h is None:
		return None, None, None, None, None, None, None, None, None, None, None

	
	t_om = t2 - t1

	return result_h, total_cost_h, path_h, u_count_h, makespan_nexp_h, makespan, makespan_CL, t_om


# **********************************************************************************************************************
