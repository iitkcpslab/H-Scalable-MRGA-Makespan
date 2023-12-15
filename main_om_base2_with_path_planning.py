# Trigger file -- executes OM and Base-2 with path planning (generates results for Table 3)

import numpy as np
from pathlib import Path

import om
import base2

import tswap_path_planner


results_folder_name = ""



def get_initial_locations_2D(robots, goals, workSpace):     # Fetches initial location of robots and the goals

	all_start_loc = {}  # for robots
	all_goal_loc  = {}  # for tasks

	
	# print("\n-------------------------------------------------------------------")
	# print("-----------Fetching input for robots' initial locations------------")
	# print("-------------------------------------------------------------------\n")
	

	i = 0
	
	while ( i < len(robots) ):

		a = np.random.randint(0, len(workSpace))
		b = np.random.randint(0, len(workSpace[0]))
		aa = (a, b)

		if aa in all_start_loc.values():  # two robots cannot have same start loc
			continue
		
		if aa[0] > (len(workSpace) - 1) or aa[0] < 0 or aa[1] > (len(workSpace[0]) - 1) or aa[1] < 0:
			# print("Input is out of bounds of workspace, please re-enter\n")
			continue  
		
		if workSpace[aa[0]][aa[1]] != 0:    # i.e. if obstacle is present, then it is not a valid cell
			# print("Invalid input! Obstacle is present; please re-enter\n")
			continue  

		all_start_loc[robots[i]] = aa
		i = i + 1
		# print("\n")
	# print("Fetching of initial locations of robots complete...")
	# print("-------------------------------------------------------------------\n")



	# print("-------------------------------------------------------------------")
	# print("--------------Fetching input for Goals' locations------------------")
	# print("-------------------------------------------------------------------\n")
	# print("\nTaking input for Goal locations (task locations): \n")
	i = 0
	while ( i < len(goals) ):

		a = np.random.randint(0, len(workSpace))
		b = np.random.randint(0, len(workSpace[0]))
		bb = (a, b)

		
		if bb in all_goal_loc.values():  # two robots cannot have same goal loc
			continue
		
		
		if bb in all_start_loc.values():  # goal loc and start loc cannot be same [NEW ADDITION]
			continue


		if bb[0] > (len(workSpace) - 1) or bb[0] < 0 or bb[1] > (len(workSpace[0]) - 1) or bb[1] < 0:
			# print("Input is out of bounds of workspace, please re-enter")
			continue  
		
		if workSpace[bb[0]][bb[1]] != 0:    # i.e. if obstacle is present, then it is not a valid cell
			# print("Invalid input! Obstacle is present; please re-enter")
			continue  
		
		all_goal_loc[goals[i]] = bb
		i = i + 1
		# print("\n")
	# print("Fetching of locations of goals complete...")
	# print("-------------------------------------------------------------------\n")
   


	return all_start_loc, all_goal_loc



def get_initial_locations_3D(robots, goals, workSpace):     # Fetches initial location of robots and the goals

	all_start_loc = {}  # for robots
	all_goal_loc  = {}  # for tasks

	
	# print("\n-------------------------------------------------------------------")
	# print("-----------Fetching input for robots' initial locations------------")
	# print("-------------------------------------------------------------------\n")
	
	# print("[Note]: locations can lie from ( 0, 0 ) up till (", len(workSpace) - 1, ", ", len(workSpace[0]) - 1, ")\n" )

	i = 0
	
	while ( i < len(robots) ):

		a = np.random.randint(0, workSpace.shape[0])
		b = np.random.randint(0, workSpace.shape[1])
		c = np.random.randint(0, workSpace.shape[2])
		aa = (a, b, c)

		if aa in all_start_loc.values():  # two robots cannot have same start loc
			continue
		
		if aa[0] > (workSpace.shape[0] - 1) or aa[0] < 0 or \
			aa[1] > (workSpace.shape[1] - 1) or aa[1] < 0 or \
			aa[2] > (workSpace.shape[2] - 1) or aa[2] < 0:
			# print("Input is out of bounds of workspace, please re-enter\n")
			continue  
		
		if workSpace[aa[0]][aa[1]][aa[2]] != 0:    # i.e. if obstacle is present, then it is not a valid cell
			# print("Invalid input! Obstacle is present; please re-enter\n")
			continue  

		all_start_loc[robots[i]] = aa
		i = i + 1
		# print("\n")
	# print("Fetching of initial locations of robots complete...")
	# print("-------------------------------------------------------------------\n")



	# print("-------------------------------------------------------------------")
	# print("--------------Fetching input for Goals' locations------------------")
	# print("-------------------------------------------------------------------\n")
	# print("\nTaking input for Goal locations (task locations): \n")
	i = 0
	while ( i < len(goals) ):

		a = np.random.randint(0, workSpace.shape[0])
		b = np.random.randint(0, workSpace.shape[1])
		c = np.random.randint(0, workSpace.shape[2])
		bb = (a, b, c)

		
		if bb in all_goal_loc.values():  # two robots cannot have same goal loc
			continue
		

		if bb in all_start_loc.values():  # goal loc and start loc cannot be same [NEW ADDITION]
			continue
		
		
		if bb[0] > (workSpace.shape[0] - 1) or bb[0] < 0 or \
			 bb[1] > (workSpace.shape[1] - 1) or bb[1] < 0 or \
			 bb[2] > (workSpace.shape[2] - 1) or bb[2] < 0:
			# print("Input is out of bounds of workspace, please re-enter")
			continue  
		
		if workSpace[bb[0]][bb[1]][bb[2]] != 0:    # i.e. if obstacle is present, then it is not a valid cell
			# print("Invalid input! Obstacle is present; please re-enter")
			continue  
		
		all_goal_loc[goals[i]] = bb
		i = i + 1
		# print("\n")
	# print("Fetching of locations of goals complete...")
	# print("-------------------------------------------------------------------\n")
   


	return all_start_loc, all_goal_loc




def get_workspace( ws_type, ws_l, ws_b, obs_den ):

	
	if (ws_type == 'random'):  # Random placement of obstacles

		workSpace = np.zeros((ws_l, ws_b))             
			
		no_of_obs = ( obs_den * ws_l * ws_b ) // 100
				

		i = 1
		obstacles = set()
		while i <= no_of_obs:
			a = np.random.randint(0, ws_l)
			b = np.random.randint(0, ws_b)

			if (a, b) not in obstacles:
				obstacles.add((a, b))
				workSpace[a][b] = 1
				i = i + 1
			

	elif (ws_type == 'replay'):	
		# workSpace = np.loadtxt( 'ws_10_4_4_20_r67.txt', dtype = int, skiprows=0 )
		# workSpace = np.loadtxt( 'ws_10_5_5_20_r26.txt', dtype = int, skiprows=0 )
		workSpace = np.loadtxt( 'ws_50_4_2_20_r1.txt', dtype = int, skiprows=0 )
		
	
	elif (ws_type == 'warehouse'):	
		workSpace = np.loadtxt( 'benchmark_maps/benchmark5_warehouse-20-40-10-2-1.txt', dtype = int, skiprows=0 ) 
	
	elif (ws_type == 'boston'):
		workSpace = np.loadtxt( 'benchmark_maps/benchmark4_Boston_0_256.txt', dtype = int, skiprows=0 ) 
	
	elif (ws_type == 'paris'):
		workSpace = np.loadtxt( 'benchmark_maps/Paris_1_256.txt', dtype = int, skiprows=0 )

	elif (ws_type == 'sydney'):
		workSpace = np.loadtxt( 'benchmark_maps/Sydney_0_256.txt', dtype = int, skiprows=0 )

	elif (ws_type == 'shanghai'):
		workSpace = np.loadtxt( 'benchmark_maps/Shanghai_2_256.txt', dtype = int, skiprows=0 )

	elif (ws_type == 'mansion'):
		workSpace = np.loadtxt( 'benchmark_maps/benchmark8_ht_mansion_n.txt', dtype = int, skiprows=0 ) 
	
	elif (ws_type == 'den'):
		workSpace = np.loadtxt( 'benchmark_maps/den520d.txt', dtype = int, skiprows=0 )
	
	elif (ws_type == '3D'):    # Configuring 3D workspace using a given configuration file
		
		with open('benchmark_maps/Complex.3dmap', 'r') as f:
    
			first_line = f.readline()
			first_line_split = first_line.split()
			

			workSpace = np.zeros((int(first_line_split[1]), int(first_line_split[2]), int(first_line_split[3])), dtype=int)
			
			
			for line in f:
				split_nos = line.split()
				workSpace[int(split_nos[0])][int(split_nos[1])][int(split_nos[2])] = 1


		f.close()
		xs,ys,zs = np.where(workSpace != 0) # main line 1
		workSpace = workSpace[min(xs):max(xs)+4,min(ys):max(ys)+4,min(zs):max(zs)+4]    # main line 2
		print("Reduced 3D workspace's shape: ", workSpace.shape)





	return workSpace






def execute_set( set_param ):

	
	global results_folder_name

	ws_type = set_param['ws_type']
	no_of_robots = set_param['n_o_r']
	no_of_goals = set_param['n_o_g']
	
	loop_no = 1
	total_loops = set_param['total_loops']
	verbosity = set_param['verbosity']
	
	
	# H = Heuristic, i.e. OM, i.e. our approach
	# T = TSWAP's version
	

	collect_times_taken_H = []
	
	collect_times_taken_T = []
	
	
	collect_nexp_makespan_H = []
	collect_nexp_makespan_T = []
	
	collect_total_cost_H = []
	collect_total_cost_T = []
	
	collect_makespan_H = []
	collect_makespan_T = []
	
	
	collect_heur_CL_om = []
	collect_tswap_CL_om = []
	

	collect_col_free_time_H = []
	
	collect_col_free_cost_H = []
	
	collect_col_free_makespan_H = []
	

	

	log_grouped_data = open(results_folder_name + '1_grouped_data.txt', 'w')

	
	while ( loop_no <= total_loops ):
		

		workSpace = get_workspace(ws_type, set_param['ws_l'], set_param['ws_b'], set_param['obs_den'] )
		

		robots = []
		goals  = []

		
		if ws_type == 'replay':
			all_start_loc = {'r0': (9, 6), 'r1': (6, 5), 'r2': (0, 6), 'r3': (8, 1), 'r4': (4, 4)}
						
			all_goal_loc = {'g0': (0, 8), 'g1': (6, 4), 'g2': (7, 8), 'g3': (3, 8), 'g4': (9, 9)}
			
			


			no_of_robots = len(all_start_loc)
			no_of_goals = len(all_goal_loc)

			for i in range(no_of_robots):
				robots.append('r' + str(i))

			for j in range(no_of_goals):
				goals.append('g' + str(j))
		
		
		else:
			
		
			for i in range(no_of_robots):
				robots.append('r' + str(i))

			for j in range(no_of_goals):
				goals.append('g' + str(j))
				
		
			# Fetching start locations of all robots and goal locations of all tasks 

			if ws_type == '3D':
				all_start_loc, all_goal_loc = get_initial_locations_3D(robots, goals, workSpace)
			else:
				all_start_loc, all_goal_loc = get_initial_locations_2D(robots, goals, workSpace)


		
		# exec_id = ws_type + '_r' + str(loop_no)
		set_name = str(workSpace.shape[0]) + '_' + str(no_of_robots) + '_' + str(no_of_goals) + '_' + str(set_param['obs_den']) 
		exec_id = set_name + '_r' + str(loop_no)
		
		ws_fname =   results_folder_name + 'ws_' + exec_id  + '.txt'   # this is filename in which workSpace would be saved
		log_fname = results_folder_name + 'log_' + exec_id  + '.txt'   # this is filename in which all output would be saved
		
		logf = open(log_fname, 'w')
		

		
		print("Execution ID: ", exec_id, file=logf)	
		print("-----------------------------------------------------------------------------------------\n", file=logf)
		print("Initial details: \n", file=logf)
		
		if ws_type == '3D':
			print("WorkSpace size: ", workSpace.shape[0], "*", workSpace.shape[1], "*", workSpace.shape[2], file=logf)
		else:
			print("WorkSpace size: ", workSpace.shape[0], "*", workSpace.shape[1], file=logf)

		if ws_type == 'random':
			print("Obstacle density: ", set_param['obs_den'], "%", file=logf)

		print("Number of robots: ", no_of_robots, file=logf)
		print("Number of goals: ", no_of_goals, file=logf)
		
		print("-----------------------------------------------------------------------------------------\n", file=logf)
		

				
		print("Initial location of robots : ", all_start_loc, file=logf)
		print("\nLocation of goals / tasks  : ", all_goal_loc, file=logf)
		print("\n", file=logf)
		
		logf.close()
		logf = open(log_fname, 'a')

		if ws_type == 'random':
			np.savetxt( ws_fname, workSpace, fmt='%d' )




		if verbosity > 0:
			print("\nWorkspace is ready")

		
	

		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                                 OM begins ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		


		total_Htime = 0
		
		
		# # Invoking function for OM case
		# ----------------------------------------------------------------------------------------------------------------------------------------
		result_h, total_cost_h, path_h, u_count_h, nexp_makespan_h, makespan_h, makespan_CL, heur_t_om = om.heuristic ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type )
		# ----------------------------------------------------------------------------------------------------------------------------------------


		if result_h is None or u_count_h > 0:
			if verbosity > 0:
				print("Found trapped robot/goal... retrying...\n")
			continue


		total_Htime = heur_t_om
		

		# Calling TSWAP's path planning module 
		col_free_time_h, col_free_path_h, col_free_cost_h, col_free_makespan_h = tswap_path_planner.tswap_path_planning ( result_h, path_h, workSpace, all_start_loc, all_goal_loc, ws_type )



		print("Printing results for OM case: ", file=logf)
		print("-------------------------------\n", file=logf)
		print("Time taken for OM case: ", total_Htime, "seconds", file=logf)
		print("No of robot-goal paths explored in OM case: ", nexp_makespan_h, file=logf)
		print("Total cost for OM case: ", total_cost_h, file=logf)
		print("MAKESPAN-H: ", makespan_h, file=logf)
		print("Number of goals (or robots) unassigned for OM case: ", u_count_h, file=logf)

		print("COL-FREE-TIME for OM case: ", col_free_time_h, file=logf)
		print("COL-FREE-COST for OM case: ", col_free_cost_h, file=logf)

		print("CL count for OM case: ", makespan_CL, file=logf)
		print("\nDetailed result of assignment: ", file=logf)
		print("-------------------------------\n", file=logf)
		print(result_h, file=logf)
		

		print("\nPrinting all paths: ", file=logf)
		print("-------------------\n", file=logf)
		
		for res in result_h:
			print("\nPath for ", res[0][0], " to ", res[0][1], ":", file=logf)
			print( path_h [res[0][0]] [res[0][1]], file=logf )  # res[0][0] is robot, and res[0][1] is goal
		
		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                                 OM completed ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		

		if verbosity > 0:
			print("\nExecution of our approach completed")
			print("\nStarting execution of Baseline - BASE-2 \n")



		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                                BASE-2 (TSWAP) begins ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		
			

		total_Ttime = 0

		
		# TSWAP with FRAstar
		result_tswap, total_cost_tswap, path_tswap, u_count_tswap, makespan_nexp_tswap, makespan_tswap, makespan_CL_tswap, t_om_tswap = base2.heuristic_tswap( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type )
		# ------------------------------------------------------------------------------------------------------------------------------

		total_Ttime = t_om_tswap

		
		print("\nPrinting results for BASE-2 (TSWAP) case: ", file=logf)
		print("--------------------------------\n", file=logf)
		print("Time taken for TSWAP case: ", total_Ttime, "seconds", file=logf)
		print("No of robot-goal paths explored in TSWAP case: ", makespan_nexp_tswap, file=logf)
		print("Total cost for TSWAP case: ", total_cost_tswap, file=logf)
		print("MAKESPAN-TSWAP: ", makespan_tswap, file=logf)
		print("Number of goals (or robots) unassigned for TSWAP case: ", u_count_tswap, file=logf)
		print("CL count for TSWAP case: ", makespan_CL_tswap, file=logf)
		print("\nDetailed result of assignment: ", file=logf)
		print("-------------------------------\n", file=logf)
		print(result_tswap, file=logf)
		
		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                               BASE-2 (TSWAP) completed ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		


		if verbosity > 0:
			print("\nExecution of baseline BASE-2 (TSWAP) completed")



		print("-------------------", file=logf)
		print("-------------------", file=logf)
		print("    Conclusion: ", file=logf)
		print("-------------------", file=logf)
		print("-------------------\n", file=logf)
		

		
		print("\n\nOUR ALGO:--", file=logf)
		print("\nTime taken for OM case (seconds):", total_Htime, file=logf)
		
		print("No of robot-goal paths explored in OM case: ", nexp_makespan_h, file=logf)
		print("Total cost for OM case: ", total_cost_h, file=logf)
		print("MAKESPAN-H: ", makespan_h, file=logf)
		print("COL-FREE MAKESPAN-H: ", col_free_makespan_h, file=logf)
		
		
		print("COL-FREE-TIME for OM case: ", col_free_time_h, file=logf)
		print("COL-FREE-COST for OM case: ", col_free_cost_h, file=logf)
		
		print("OM CL count: ", makespan_CL, file=logf)



		

		print("\n\nBASE-2 TSWAP CASE:--", file=logf)
		print("\nTime taken for TSWAP case (seconds): ", total_Ttime, file=logf)
		
		print("No of robot-goal paths explored in TSWAP case: ", makespan_nexp_tswap, file=logf)
		print("Total cost for TSWAP case: ", total_cost_tswap, file=logf)
		print("MAKESPAN-TSWAP: ", makespan_tswap, file=logf)
		print("CL count for TSWAP: ", makespan_CL_tswap, file=logf)
		print("----------------------------------------------", file=logf)



		if ws_type == 'random':
			# np.savetxt( ws_fname, workSpace, fmt='%d' )
			print("\n----------------------------------------------", file=logf)
			print("Workspace saved in file ", ws_fname, file=logf)
		print("----------------------------------------------\n", file=logf)
		
		logf.close()
		
		
		# Runtime
		
		collect_times_taken_H.append( total_Htime )
		

		collect_times_taken_T.append( total_Ttime )
		

		# NEXP

		collect_nexp_makespan_H.append( nexp_makespan_h ) # om
		collect_nexp_makespan_T.append( makespan_nexp_tswap ) # om
		
		
		# COST

		collect_total_cost_H.append( total_cost_h )
		collect_total_cost_T.append( total_cost_tswap )
		

		# MAKESPAN

		collect_makespan_H.append( makespan_h )
		collect_makespan_T.append( makespan_tswap )


		# CL count 

		collect_heur_CL_om.append( makespan_CL )  # om
		collect_tswap_CL_om.append( makespan_CL_tswap )


		# COL-FREE-TIME

		collect_col_free_time_H.append( col_free_time_h )
		

		# COL-FREE-COST

		collect_col_free_cost_H.append( col_free_cost_h )
		
		# COL-FREE-MAKESPAN

		collect_col_free_makespan_H.append( col_free_makespan_h )
		
		
		if verbosity > 0:
			print("\nRound ", loop_no, " completed - - - - - - - - - - - - - - - \n")
		loop_no = loop_no + 1


	# Summarizing

	time_h_mean = float( format( np.mean( collect_times_taken_H ), '.2f' ) )
	time_h_std  = float( format( np.std( collect_times_taken_H ), '.2f' ) )
	
	
	time_t_mean = float( format( np.mean( collect_times_taken_T ), '.2f' ) )
	time_t_std  = float( format( np.std( collect_times_taken_T ), '.2f' ) )


	# nexp upto optimal makespan
	
	nexp_makespan_h_mean  = float( format( np.mean( collect_nexp_makespan_H ), '.2f' ) )
	nexp_makespan_h_std   = float( format( np.std( collect_nexp_makespan_H ), '.2f' ) )

	nexp_makespan_tswap_mean  = float( format( np.mean( collect_nexp_makespan_T ), '.2f' ) )
	nexp_makespan_tswap_std   = float( format( np.std( collect_nexp_makespan_T ), '.2f' ) )
	
	
	# cost
	
	cost_h_mean = float( format( np.mean( collect_total_cost_H ), '.2f' ) )
	cost_h_std  = float( format( np.std( collect_total_cost_H ), '.2f' ) )
	
	cost_t_mean = float( format( np.mean( collect_total_cost_T ), '.2f' ) )
	cost_t_std  = float( format( np.std( collect_total_cost_T ), '.2f' ) )
	
	# makespan

	makespan_h_mean = float( format( np.mean( collect_makespan_H ), '.2f' ) )
	makespan_h_std  = float( format( np.std( collect_makespan_H ), '.2f' ) )

	makespan_t_mean = float( format( np.mean( collect_makespan_T ), '.2f' ) )
	makespan_t_std  = float( format( np.std( collect_makespan_T ), '.2f' ) )


	# CL count
	
	heur_CL_om_mean = float( format( np.mean( collect_heur_CL_om ), '.2f' ) )
	heur_CL_om_std  = float( format( np.std( collect_heur_CL_om ), '.2f' ) )
	
	tswap_CL_om_mean = float( format( np.mean( collect_tswap_CL_om ), '.2f' ) )
	tswap_CL_om_std  = float( format( np.std( collect_tswap_CL_om ), '.2f' ) )



	netResult_filename = results_folder_name + 'netResult.txt'
	logf_netResult = open(netResult_filename, 'a')
	print( "----------------------------------------------------------", file=logf_netResult )
	print( "Set : ", set_name, file=logf_netResult )
	print( "----------------------------------------------------------", file=logf_netResult )
	
	print("\nRUNTIME:", file=logf_netResult)	

	
	print( "\nMean runtime for OM case (in seconds): ", time_h_mean, file=logf_netResult )
	print( "Stdv runtime for OM case (in seconds): ", time_h_std, file=logf_netResult )
	
	
	print( "- - - - - - - ", file=logf_netResult )

	
	print( "\nMean runtime for BASE-2 BASE-2 case (in seconds): ", time_t_mean, file=logf_netResult )
	print( "Stdv runtime for BASE-2 BASE-2 case (in seconds): ", time_t_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	
	
	print( "\nMean runtime for OM case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.mean( collect_col_free_time_H ), '.2f' ) ), file=logf_netResult )
	print( "Stdv runtime for OM case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.std( collect_col_free_time_H ), '.2f' ) ), file=logf_netResult )
		
	
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )


	print("\n\nNEXP:", file=logf_netResult)	

	print( "\nMean nexp for OM case: ", nexp_makespan_h_mean, file=logf_netResult )
	print( "Stdv nexp for OM case: ", nexp_makespan_h_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean nexp for BASE-2 case : ", nexp_makespan_tswap_mean, file=logf_netResult )
	print( "Stdv nexp for BASE-2 case: ", nexp_makespan_tswap_std, file=logf_netResult )


	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )

	
	print("\n\nMAKESPAN:", file=logf_netResult)	

	print( "\n\nMean MAKESPAN for OM case: ", makespan_h_mean, file=logf_netResult )
	print( "Stdv MAKESPAN for OM case: ", makespan_h_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean MAKESPAN for BASE-2 case: ", makespan_t_mean, file=logf_netResult )
	print( "Stdv MAKESPAN for BASE-2 case: ", makespan_t_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean MAKESPAN for OM case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.mean( collect_col_free_makespan_H ), '.2f' ) ), file=logf_netResult )
	print( "Stdv MAKESPAN for OM case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.std( collect_col_free_makespan_H ), '.2f' ) ), file=logf_netResult )
	
	
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )

	print("\nCOST:", file=logf_netResult)	

	print( "\n\nMean cost for OM case: ", cost_h_mean, file=logf_netResult )
	print( "Stdv cost for OM case: ", cost_h_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )
	
	print( "\nMean cost for BASE-2 case: ", cost_t_mean, file=logf_netResult )
	print( "Stdv cost for BASE-2 case: ", cost_t_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean cost for OM case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.mean( collect_col_free_cost_H ), '.2f' ) ), file=logf_netResult )
	print( "Stdv cost for OM case TO GET COLL-FREE PATHS (in seconds): ", float( format( np.std( collect_col_free_cost_H ), '.2f' ) ), file=logf_netResult )
	
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )


	print("\n\nCL COUNT:", file=logf_netResult)	

	print( "\nMean CL count for OM case: ", heur_CL_om_mean, file=logf_netResult )
	print( "Stdv CL count for OM case: ", heur_CL_om_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean CL count for BASE-2 case: ", tswap_CL_om_mean, file=logf_netResult )
	print( "Stdv CL count for BASE-2 case: ", tswap_CL_om_std, file=logf_netResult )

	print( "----------------------------------------------------------\n\n", file=logf_netResult )
	logf_netResult.close()


	# Printing individual datum in a single file

	print("\n RUNTIMES \n", file=log_grouped_data)

	print("\n\nOM runtimes - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_times_taken_H:
		print(tt_h, file=log_grouped_data)


	print("\n\nOM runtimes TO GET COLLISON-FREE PATHS - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_col_free_time_H:
		print(tt_h, file=log_grouped_data)




	print("\nBase-2 runtimes - TAKE IT:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_T:
		print(tt_a, file=log_grouped_data)


	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	
	print("\n NEXP \n", file=log_grouped_data)


	print("\nOM nexp:\n", file=log_grouped_data)
	
	for tnexp_h in collect_nexp_makespan_H:
		print(tnexp_h, file=log_grouped_data)

	

	print("\nBase-2 nexp:\n", file=log_grouped_data)
	
	for tnexp_t in collect_nexp_makespan_T:
		print(tnexp_t, file=log_grouped_data)


	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)
	
	print("\n CL count \n", file=log_grouped_data)

	print("\nCL count - OM:\n", file=log_grouped_data)
	
	for clc in collect_heur_CL_om:
		print(clc, file=log_grouped_data)
	

	print("\nBase-2 CL count:\n", file=log_grouped_data)
	
	for clc in collect_tswap_CL_om:
		print(clc, file=log_grouped_data)


	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	print("\n MAKESPAN \n", file=log_grouped_data)


	print("\nOM MAKESPAN:\n", file=log_grouped_data)
	
	for tmakespan_h in collect_makespan_H:
		print(tmakespan_h, file=log_grouped_data)

	print("\nBase-2 MAKESPAN:\n", file=log_grouped_data)
	
	for tmakespan_a in collect_makespan_T:
		print(tmakespan_a, file=log_grouped_data)


	print("\n\nOM MAKESPAN for COLLISON-FREE PATHS - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_col_free_makespan_H:
		print(tt_h, file=log_grouped_data)


	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	print("\n COST \n", file=log_grouped_data)


	print("\nOM cost:\n", file=log_grouped_data)
	
	for tcost_h in collect_total_cost_H:
		print(tcost_h, file=log_grouped_data)

	print("\nBase-2 cost:\n", file=log_grouped_data)
	
	for tcost_a in collect_total_cost_T:
		print(tcost_a, file=log_grouped_data)

	
	print("\n\nOM cost for COLLISON-FREE PATHS - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_col_free_cost_H:
		print(tt_h, file=log_grouped_data)
	

	print("\n--------------E-N-D-------------------:", file=log_grouped_data)

	log_grouped_data.close()





	return


	
	

def main():

	# Preparation for running code for multiple times with varied input settings. 

	global results_folder_name

	
	settings = {}
	

	workspace_length = 0
	workspace_width = 0
	obs_density = 0
	n_o_r = 0
	n_o_g = 0


	print("\n\n________________________________________________\n")
	print("\t\tW E L C O M E")
	print("________________________________________________\n")


	while True:
		
		print("\n\n________________________________________________\n")
		print("Following are the supported types of workspaces:\n")

		print("1. Random workspace")
		print("2. Standard workspace")
		print("3. Replay past execution")
		
		print("________________________________________________\n")

		try:
			choice = int(input("\nPlease select the workspace type or press 9 to exit: "))
		except ValueError:
			print("\nThat's not a number! Please retry...")
			continue

		if choice == 9:
			break
		
		
		
		if choice == 1:
			ws_type = 'random'

			print("\n\nPlease enter the following details --")
			workspace_length = int(input("\n\tLength of workspace: "))
			workspace_width  = int(input("\n\tWidth of workspace: "))

			obs_density  = int(input("\n\tObstacle density: "))


		elif choice == 2:

			print(" \nThese are standard workspaces supported:\n")

			print("1. Boston")
			print("2. Paris")
			print("3. Sydney")
			print("4. Shanghai")
			print("5. Warehouse")
			print("6. Mansion")
			print("7. Den")
			print("8. 3D Warframe")
			

			try:
				choice_b = int(input("\nPlease select the standard workspace or press 9 to exit: "))
			except ValueError:
				print("\nThat's not a number! Please retry...")
				continue

			if choice_b == 9:
				break


			if choice_b == 1:
				ws_type = 'boston'

			elif choice_b == 2:
				ws_type = 'paris'

			elif choice_b == 3:
				ws_type = 'sydney'
			
			elif choice_b == 4:
				ws_type = 'shanghai'

			elif choice_b == 5:
				ws_type = 'warehouse'

			elif choice_b == 6:
				ws_type = 'mansion'

			elif choice_b == 7:
				ws_type = 'den'

			elif choice_b == 8:
				ws_type = '3D'

			# elif choice_b == 7:
			# 	ws_type = 'replay'


			else:
				print("\n\nInvalid choice, please retry...")
				continue

			print("\nPlease enter the following details --")
		
		elif choice == 3:
			ws_type = 'replay'
		
		else:
			print("\n\nInvalid choice, please retry...")
			continue


		if choice != 3:
			n_o_r = int(input("\n\tNumber of robots: "))
			n_o_g = int(input("\n\tNumber of goals: "))


		total_loops = int(input("\n\tNumber of rounds for which experiment should run: "))
		results_folder_name = input("\n\tRelative folder name to store result files: ")
		results_folder_name = results_folder_name + '/'
		Path(results_folder_name).mkdir(exist_ok=True)
		verbosity = int(input("\n\tPlease select the verbosity level(0, 1, or 2): "))
		# verbosity = 2
		print("\n-------------------------------------------------------------\n")



		settings[ 'main_set'  ] = { 'ws_l': workspace_length, 'ws_b': workspace_width, 'n_o_r': n_o_r, 'n_o_g': n_o_g, 'obs_den': obs_density, 'total_loops': total_loops,'ws_type': ws_type, 'verbosity': verbosity }	


		print("Thank you for the inputs, execution has started...\n")
		print("-------------------------------------------------------------\n")

		execute_set( settings['main_set'] )

		print("-------------------------------------------------------------\n")
		print("Execution for given inputs completed...Restarting...\n")
		print("-------------------------------------------------------------\n")
	

	print("\n_______________________________")
	print("\nExecution completed, thank you!")
	print("_______________________________\n\n")



if __name__ == '__main__':
    main()

