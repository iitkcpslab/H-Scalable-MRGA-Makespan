# Trigger file -- executes OM, Base-1 and Base-2 (generates results for Table 1, Figure 3, and Table 2)

import numpy as np
from pathlib import Path

import om
import base1
import base2



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
		workSpace = np.loadtxt( 'ws_200_200_200_20_r2.txt', dtype = int, skiprows=0 )
		# workSpace = np.loadtxt( 'ws_15_6_6_15_r534.txt', dtype = int, skiprows=0 )
		# workSpace = np.loadtxt( 'ws_10_4_4_10_r2128.txt', dtype = int, skiprows=0 )
		
		
		
	
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
	# T = BASE-2 (TSWAP's) version
	# D = BASE-1 (Dijkstra's) approach
	

	collect_times_taken_H = []
	
	collect_times_taken_T = []
	
	collect_times_taken_D = []
	
	
	
	collect_nexp_makespan_H = []
	collect_nexp_makespan_T = []
	# collect_nexp_makespan_D = []
	collect_no_of_exp_D = []

	collect_total_cost_H = []
	collect_total_cost_T = []
	collect_total_cost_D = []

	collect_makespan_H = []
	collect_makespan_T = []
	collect_makespan_D = []

	
	collect_heur_CL_om = []
	collect_tswap_CL_om = []
	collect_dijk_CL = []
	

	log_grouped_data = open(results_folder_name + '1_grouped_data.txt', 'w')

	
	while ( loop_no <= total_loops ):
		

		workSpace = get_workspace(ws_type, set_param['ws_l'], set_param['ws_b'], set_param['obs_den'] )
		

		robots = []
		goals  = []

		
		if ws_type == 'replay':
			# all_start_loc = {'r0': (9, 6), 'r1': (6, 5), 'r2': (0, 6), 'r3': (8, 1)}
			all_start_loc = {'r0': (193, 2), 'r1': (14, 170), 'r2': (172, 91), 'r3': (40, 124), 'r4': (37, 3), 'r5': (189, 165), 'r6': (174, 28), 'r7': (87, 196), 'r8': (131, 5), 'r9': (182, 102), 'r10': (53, 67), 'r11': (1, 47), 'r12': (37, 170), 'r13': (126, 190), 'r14': (56, 198), 'r15': (150, 60), 'r16': (130, 125), 'r17': (161, 108), 'r18': (29, 80), 'r19': (120, 17), 'r20': (55, 50), 'r21': (77, 69), 'r22': (85, 73), 'r23': (158, 139), 'r24': (69, 3), 'r25': (134, 109), 'r26': (184, 39), 'r27': (141, 172), 'r28': (25, 24), 'r29': (175, 176), 'r30': (66, 13), 'r31': (89, 26), 'r32': (185, 35), 'r33': (81, 27), 'r34': (192, 77), 'r35': (144, 57), 'r36': (47, 7), 'r37': (45, 192), 'r38': (86, 145), 'r39': (47, 30), 'r40': (113, 12), 'r41': (67, 81), 'r42': (3, 56), 'r43': (199, 85), 'r44': (49, 164), 'r45': (103, 0), 'r46': (3, 11), 'r47': (74, 178), 'r48': (56, 150), 'r49': (123, 73), 'r50': (143, 18), 'r51': (143, 89), 'r52': (100, 161), 'r53': (142, 155), 'r54': (26, 148), 'r55': (82, 52), 'r56': (186, 175), 'r57': (68, 70), 'r58': (35, 38), 'r59': (165, 76), 'r60': (85, 0), 'r61': (122, 28), 'r62': (86, 180), 'r63': (26, 116), 'r64': (56, 86), 'r65': (44, 162), 'r66': (146, 91), 'r67': (168, 36), 'r68': (184, 33), 'r69': (185, 150), 'r70': (133, 25), 'r71': (66, 81), 'r72': (144, 193), 'r73': (37, 134), 'r74': (118, 58), 'r75': (107, 16), 'r76': (99, 57), 'r77': (154, 31), 'r78': (95, 61), 'r79': (164, 90), 'r80': (143, 95), 'r81': (129, 51), 'r82': (184, 60), 'r83': (102, 104), 'r84': (105, 132), 'r85': (101, 51), 'r86': (58, 68), 'r87': (1, 195), 'r88': (143, 65), 'r89': (163, 113), 'r90': (80, 15), 'r91': (2, 24), 'r92': (140, 171), 'r93': (28, 182), 'r94': (57, 31), 'r95': (135, 55), 'r96': (191, 119), 'r97': (57, 113), 'r98': (30, 187), 'r99': (19, 131), 'r100': (116, 72), 'r101': (4, 15), 'r102': (32, 35), 'r103': (163, 197), 'r104': (136, 181), 'r105': (9, 95), 'r106': (9, 124), 'r107': (6, 124), 'r108': (97, 158), 'r109': (99, 188), 'r110': (119, 132), 'r111': (95, 70), 'r112': (177, 74), 'r113': (38, 179), 'r114': (90, 111), 'r115': (171, 155), 'r116': (87, 83), 'r117': (12, 165), 'r118': (41, 149), 'r119': (18, 29), 'r120': (20, 117), 'r121': (52, 57), 'r122': (96, 75), 'r123': (189, 27), 'r124': (80, 32), 'r125': (150, 145), 'r126': (177, 54), 'r127': (47, 134), 'r128': (3, 88), 'r129': (151, 192), 'r130': (164, 62), 'r131': (159, 167), 'r132': (193, 38), 'r133': (46, 163), 'r134': (187, 168), 'r135': (5, 91), 'r136': (10, 89), 'r137': (45, 125), 'r138': (139, 194), 'r139': (130, 18), 'r140': (28, 71), 'r141': (196, 170), 'r142': (57, 151), 'r143': (40, 97), 'r144': (130, 99), 'r145': (25, 14), 'r146': (121, 113), 'r147': (21, 110), 'r148': (34, 60), 'r149': (197, 77), 'r150': (7, 63), 'r151': (134, 141), 'r152': (20, 45), 'r153': (198, 159), 'r154': (127, 194), 'r155': (41, 51), 'r156': (3, 20), 'r157': (45, 196), 'r158': (149, 26), 'r159': (9, 113), 'r160': (164, 86), 'r161': (19, 13), 'r162': (47, 81), 'r163': (133, 41), 'r164': (171, 134), 'r165': (161, 8), 'r166': (35, 138), 'r167': (108, 29), 'r168': (90, 110), 'r169': (199, 61), 'r170': (140, 193), 'r171': (113, 49), 'r172': (80, 68), 'r173': (132, 117), 'r174': (130, 144), 'r175': (33, 116), 'r176': (195, 13), 'r177': (151, 142), 'r178': (156, 61), 'r179': (177, 141), 'r180': (163, 106), 'r181': (38, 85), 'r182': (119, 113), 'r183': (148, 131), 'r184': (147, 124), 'r185': (190, 14), 'r186': (7, 46), 'r187': (54, 177), 'r188': (42, 143), 'r189': (179, 111), 'r190': (117, 37), 'r191': (111, 61), 'r192': (96, 184), 'r193': (54, 145), 'r194': (95, 67), 'r195': (137, 56), 'r196': (76, 0), 'r197': (131, 133), 'r198': (83, 7), 'r199': (79, 156)}
						
			# all_goal_loc = {'g0': (0, 8), 'g1': (6, 4), 'g2': (7, 8), 'g3': (3, 8), 'g4': (9, 9)}
			all_goal_loc = {'g0': (165, 42), 'g1': (119, 186), 'g2': (38, 41), 'g3': (94, 176), 'g4': (91, 2), 'g5': (29, 17), 'g6': (129, 11), 'g7': (80, 92), 'g8': (120, 56), 'g9': (46, 197), 'g10': (166, 125), 'g11': (71, 134), 'g12': (109, 67), 'g13': (131, 74), 'g14': (63, 29), 'g15': (170, 74), 'g16': (156, 154), 'g17': (88, 130), 'g18': (102, 122), 'g19': (7, 60), 'g20': (26, 89), 'g21': (91, 32), 'g22': (145, 20), 'g23': (165, 61), 'g24': (92, 72), 'g25': (55, 151), 'g26': (15, 157), 'g27': (157, 172), 'g28': (113, 0), 'g29': (35, 154), 'g30': (170, 28), 'g31': (190, 153), 'g32': (95, 49), 'g33': (166, 83), 'g34': (106, 65), 'g35': (162, 33), 'g36': (199, 131), 'g37': (117, 35), 'g38': (130, 47), 'g39': (177, 9), 'g40': (156, 14), 'g41': (44, 174), 'g42': (85, 104), 'g43': (47, 131), 'g44': (90, 193), 'g45': (190, 154), 'g46': (142, 185), 'g47': (74, 171), 'g48': (168, 160), 'g49': (0, 97), 'g50': (5, 2), 'g51': (177, 184), 'g52': (127, 168), 'g53': (109, 27), 'g54': (69, 16), 'g55': (121, 93), 'g56': (9, 132), 'g57': (169, 167), 'g58': (124, 182), 'g59': (95, 95), 'g60': (141, 187), 'g61': (111, 91), 'g62': (137, 133), 'g63': (50, 170), 'g64': (34, 107), 'g65': (122, 37), 'g66': (167, 163), 'g67': (129, 12), 'g68': (32, 163), 'g69': (59, 120), 'g70': (14, 129), 'g71': (68, 105), 'g72': (159, 135), 'g73': (17, 190), 'g74': (73, 142), 'g75': (164, 28), 'g76': (157, 182), 'g77': (148, 36), 'g78': (129, 97), 'g79': (82, 75), 'g80': (164, 162), 'g81': (55, 125), 'g82': (132, 72), 'g83': (90, 127), 'g84': (35, 156), 'g85': (79, 140), 'g86': (18, 136), 'g87': (156, 54), 'g88': (31, 167), 'g89': (198, 93), 'g90': (149, 9), 'g91': (156, 115), 'g92': (193, 149), 'g93': (34, 31), 'g94': (168, 79), 'g95': (88, 78), 'g96': (113, 54), 'g97': (10, 189), 'g98': (109, 112), 'g99': (184, 190), 'g100': (54, 95), 'g101': (28, 7), 'g102': (127, 139), 'g103': (197, 5), 'g104': (115, 2), 'g105': (81, 144), 'g106': (75, 97), 'g107': (69, 28), 'g108': (81, 86), 'g109': (173, 61), 'g110': (152, 160), 'g111': (27, 185), 'g112': (103, 164), 'g113': (106, 28), 'g114': (56, 143), 'g115': (62, 138), 'g116': (42, 22), 'g117': (3, 30), 'g118': (106, 64), 'g119': (141, 165), 'g120': (93, 36), 'g121': (98, 114), 'g122': (40, 121), 'g123': (174, 121), 'g124': (111, 44), 'g125': (110, 86), 'g126': (121, 191), 'g127': (82, 137), 'g128': (50, 156), 'g129': (197, 155), 'g130': (150, 4), 'g131': (91, 113), 'g132': (64, 179), 'g133': (6, 73), 'g134': (195, 88), 'g135': (127, 0), 'g136': (100, 123), 'g137': (35, 193), 'g138': (165, 9), 'g139': (145, 172), 'g140': (50, 179), 'g141': (189, 10), 'g142': (157, 71), 'g143': (10, 165), 'g144': (46, 53), 'g145': (34, 185), 'g146': (119, 180), 'g147': (175, 120), 'g148': (155, 1), 'g149': (168, 47), 'g150': (115, 67), 'g151': (44, 156), 'g152': (66, 196), 'g153': (65, 59), 'g154': (123, 147), 'g155': (29, 13), 'g156': (12, 1), 'g157': (136, 116), 'g158': (88, 114), 'g159': (38, 111), 'g160': (139, 108), 'g161': (28, 199), 'g162': (112, 161), 'g163': (152, 94), 'g164': (174, 69), 'g165': (165, 38), 'g166': (113, 198), 'g167': (130, 115), 'g168': (24, 95), 'g169': (154, 64), 'g170': (60, 155), 'g171': (163, 62), 'g172': (64, 10), 'g173': (109, 50), 'g174': (154, 86), 'g175': (65, 56), 'g176': (164, 146), 'g177': (62, 26), 'g178': (70, 124), 'g179': (125, 129), 'g180': (125, 66), 'g181': (116, 114), 'g182': (143, 159), 'g183': (1, 48), 'g184': (42, 175), 'g185': (146, 55), 'g186': (175, 121), 'g187': (193, 42), 'g188': (189, 42), 'g189': (123, 86), 'g190': (179, 163), 'g191': (187, 120), 'g192': (146, 155), 'g193': (199, 62), 'g194': (162, 21), 'g195': (151, 41), 'g196': (34, 165), 'g197': (81, 187), 'g198': (154, 35), 'g199': (152, 134)}
			


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
		

		print("Printing results for OM case: ", file=logf)
		print("-------------------------------\n", file=logf)
		print("Time taken for OM case: ", heur_t_om, "seconds", file=logf)
		print("No of robot-goal paths explored in OM case: ", nexp_makespan_h, file=logf)
		print("Total cost for OM case: ", total_cost_h, file=logf)
		print("MAKESPAN-H: ", makespan_h, file=logf)
		print("Number of goals (or robots) unassigned for OM case: ", u_count_h, file=logf)
		print("CL count for OM case: ", makespan_CL, file=logf)
		print("\nDetailed result of assignment: ", file=logf)
		print("-------------------------------\n", file=logf)
		print(result_h, file=logf)
		

		print("\nPrinting all paths: ", file=logf)
		print("-------------------\n", file=logf)
		
		# print("cons paths:", file=logf)
		# print(path_h, file=logf)
		# print("aks")

		for res in result_h:
			print("\nPath for ", res[0][0], " to ", res[0][1], ":", file=logf)
			print( path_h [res[0][0]] [res[0][1]], file=logf )  
		
		
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


		# Invoking function for TSWAP case
		# ------------------------------------------------------------------------------------------------------------------------------
		
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
			print("\nStarting execution of Baseline - BASE-1 (Dijkstra) \n")
		

		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                                BASE-1 (DIJK) begins ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		
		
		# Invoking function for BASE-1 (Dijkstra) case
		# ------------------------------------------------------------------------------------------------------------------------------
		
		# Naive with Dijkstra
		result_dijk, total_cost_dijk, path_dijk, u_count_dijk, no_of_exp_dijk, makespan_dijk, CL_dijk, t_om_dijk = base1.naive( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type, verbosity )
		
		total_Dtime = t_om_dijk


		print("\nPrinting results for BASE-1 (DIJK) case: ", file=logf)
		print("--------------------------------\n", file=logf)
		print("Time taken for DIJK case: ", total_Dtime, "seconds", file=logf)
		print("No of robot-goal paths explored in DIJK case: ", no_of_exp_dijk, file=logf)
		print("Total cost for DIJK case: ", total_cost_dijk, file=logf)
		print("MAKESPAN-DIJK: ", makespan_dijk, file=logf)
		print("Number of goals (or robots) unassigned for DIJK case: ", u_count_dijk, file=logf)
		print("CL count for DIJK case: ", CL_dijk, file=logf)
		print("\nDetailed result of assignment: ", file=logf)
		print("-------------------------------\n", file=logf)
		print(result_tswap, file=logf)
		
		
		print("\n\n-----------------------------------------------------------------------------------------", file=logf)
		print("                               BASE-1 (DIJK) completed ", file=logf)
		print("-----------------------------------------------------------------------------------------\n\n", file=logf)
		


		if verbosity > 0:
			print("\nExecution of baseline BASE-1 (DIJK) completed")
		



		print("-------------------", file=logf)
		print("-------------------", file=logf)
		print("    Conclusion: ", file=logf)
		print("-------------------", file=logf)
		print("-------------------\n", file=logf)
		

		
		print("\n\nOUR ALGO:--", file=logf)
		print("\nTime taken for OM case (seconds):", total_Htime, file=logf)
		
		print("No of robot-goal paths explored in OM: ", nexp_makespan_h, file=logf)
		print("Total cost for OM case: ", total_cost_h, file=logf)
		print("MAKESPAN-H: ", makespan_h, file=logf)
		print("OM CL count: ", makespan_CL, file=logf)

		

		print("\n\nBASE-2 TSWAP CASE:--", file=logf)
		print("\nTime taken for TSWAP case (seconds): ", total_Ttime, file=logf)
		
		print("No of robot-goal paths explored in TSWAP case: ", makespan_nexp_tswap, file=logf)
		print("Total cost for TSWAP case: ", total_cost_tswap, file=logf)
		print("MAKESPAN-TSWAP: ", makespan_tswap, file=logf)
		print("CL count for TSWAP: ", makespan_CL_tswap, file=logf)
		print("----------------------------------------------", file=logf)


		print("\n\nBASE-1 DIJK CASE:--", file=logf)
		print("\nTime taken for DIJK case (seconds): ", total_Dtime, file=logf)
		
		print("No of robot-goal paths explored in DIJK case: ", no_of_exp_dijk, file=logf)
		print("Total cost for DIJK case: ", total_cost_dijk, file=logf)
		print("MAKESPAN-DIJK: ", makespan_dijk, file=logf)
		print("CL count for DIJK: ", CL_dijk, file=logf)
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
		

		collect_times_taken_D.append( total_Dtime )
		


		# NEXP

		collect_nexp_makespan_H.append( nexp_makespan_h ) # om
		collect_nexp_makespan_T.append( makespan_nexp_tswap ) # om
		
		collect_no_of_exp_D.append( no_of_exp_dijk )
		
		# COST

		collect_total_cost_H.append( total_cost_h )
		collect_total_cost_T.append( total_cost_tswap )
		collect_total_cost_D.append( total_cost_dijk )
		

		# MAKESPAN

		collect_makespan_H.append( makespan_h )
		collect_makespan_T.append( makespan_tswap )
		collect_makespan_D.append( makespan_dijk )


		# CL count 

		collect_heur_CL_om.append( makespan_CL )  # om
		collect_tswap_CL_om.append( makespan_CL_tswap )
		collect_dijk_CL.append( CL_dijk )

	
		

		if verbosity > 0:
			print("\nRound ", loop_no, " completed - - - - - - - - - - - - - - - \n")
		loop_no = loop_no + 1


	# Summarizing

	time_h_mean = float( format( np.mean( collect_times_taken_H ), '.2f' ) )
	time_h_std  = float( format( np.std( collect_times_taken_H ), '.2f' ) )
	
	
	time_t_mean = float( format( np.mean( collect_times_taken_T ), '.2f' ) )
	time_t_std  = float( format( np.std( collect_times_taken_T ), '.2f' ) )

	time_d_mean = float( format( np.mean( collect_times_taken_D ), '.2f' ) )
	time_d_std  = float( format( np.std( collect_times_taken_D ), '.2f' ) )


	# nexp upto optimal makespan
	
	nexp_makespan_h_mean  = float( format( np.mean( collect_nexp_makespan_H ), '.2f' ) )
	nexp_makespan_h_std   = float( format( np.std( collect_nexp_makespan_H ), '.2f' ) )

	nexp_makespan_tswap_mean  = float( format( np.mean( collect_nexp_makespan_T ), '.2f' ) )
	nexp_makespan_tswap_std   = float( format( np.std( collect_nexp_makespan_T ), '.2f' ) )

	exp_d_mean  = float( format( np.mean( collect_no_of_exp_D ), '.2f' ) )
	exp_d_std   = float( format( np.std( collect_no_of_exp_D ), '.2f' ) )

	
	
	# cost
	
	cost_h_mean = float( format( np.mean( collect_total_cost_H ), '.2f' ) )
	cost_h_std  = float( format( np.std( collect_total_cost_H ), '.2f' ) )
	
	cost_t_mean = float( format( np.mean( collect_total_cost_T ), '.2f' ) )
	cost_t_std  = float( format( np.std( collect_total_cost_T ), '.2f' ) )

	cost_d_mean = float( format( np.mean( collect_total_cost_D ), '.2f' ) )
	cost_d_std  = float( format( np.std( collect_total_cost_D ), '.2f' ) )

	
	
	# makespan

	makespan_h_mean = float( format( np.mean( collect_makespan_H ), '.2f' ) )
	makespan_h_std  = float( format( np.std( collect_makespan_H ), '.2f' ) )

	makespan_t_mean = float( format( np.mean( collect_makespan_T ), '.2f' ) )
	makespan_t_std  = float( format( np.std( collect_makespan_T ), '.2f' ) )

	makespan_d_mean = float( format( np.mean( collect_makespan_D ), '.2f' ) )
	makespan_d_std  = float( format( np.std( collect_makespan_D ), '.2f' ) )


	# CL count
	
	heur_CL_om_mean = float( format( np.mean( collect_heur_CL_om ), '.2f' ) )
	heur_CL_om_std  = float( format( np.std( collect_heur_CL_om ), '.2f' ) )
	
	tswap_CL_om_mean = float( format( np.mean( collect_tswap_CL_om ), '.2f' ) )
	tswap_CL_om_std  = float( format( np.std( collect_tswap_CL_om ), '.2f' ) )

	dijk_CL_mean = float( format( np.mean( collect_dijk_CL ), '.2f' ) )
	dijk_CL_std  = float( format( np.std( collect_dijk_CL ), '.2f' ) )




	netResult_filename = results_folder_name + 'netResult.txt'
	logf_netResult = open(netResult_filename, 'a')
	print( "----------------------------------------------------------", file=logf_netResult )
	print( "Set : ", set_name, file=logf_netResult )
	print( "----------------------------------------------------------", file=logf_netResult )
	
	print("\nRUNTIME:", file=logf_netResult)	

	
	print( "\nMean runtime for OM case (in seconds): ", time_h_mean, file=logf_netResult )
	print( "Stdv runtime for OM case (in seconds): ", time_h_std, file=logf_netResult )
	
	
	print( "- - - - - - - ", file=logf_netResult )

	
	print( "\nMean runtime for BASE-2 case (in seconds): ", time_t_mean, file=logf_netResult )
	print( "Stdv runtime for BASE-2 case (in seconds): ", time_t_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	
	print( "\nMean runtime for BASE-1 case (in seconds): ", time_d_mean, file=logf_netResult )
	print( "Stdv runtime for BASE-1 case (in seconds): ", time_d_std, file=logf_netResult )

	
	
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )


	print("\n\nNEXP:", file=logf_netResult)	

	print( "\nMean nexp for OM case: ", nexp_makespan_h_mean, file=logf_netResult )
	print( "Stdv nexp for OM case: ", nexp_makespan_h_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean nexp for BASE-2 case : ", nexp_makespan_tswap_mean, file=logf_netResult )
	print( "Stdv nexp for BASE-2 case: ", nexp_makespan_tswap_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean nexp for BASE-1 case: ", exp_d_mean, file=logf_netResult )
	print( "Stdv nexp for BASE-1 case: ", exp_d_std, file=logf_netResult )


	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )

	
	print("\n\nMAKESPAN:", file=logf_netResult)	

	print( "\n\nMean MAKESPAN for OM case: ", makespan_h_mean, file=logf_netResult )
	print( "Stdv MAKESPAN for OM case: ", makespan_h_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean MAKESPAN for BASE-2 case: ", makespan_t_mean, file=logf_netResult )
	print( "Stdv MAKESPAN for BASE-2 case: ", makespan_t_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean MAKESPAN for base-1 case: ", makespan_d_mean, file=logf_netResult )
	print( "Stdv MAKESPAN for base-1 case: ", makespan_d_std, file=logf_netResult )
	
	
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )

	print("\nCOST:", file=logf_netResult)	

	print( "\n\nMean cost for OM case: ", cost_h_mean, file=logf_netResult )
	print( "Stdv cost for OM case: ", cost_h_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )
	
	print( "\nMean cost for BASE-2 case: ", cost_t_mean, file=logf_netResult )
	print( "Stdv cost for BASE-2 case: ", cost_t_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )
	
	print( "\nMean cost for base-1 case: ", cost_d_mean, file=logf_netResult )
	print( "Stdv cost for base-1 case: ", cost_d_std, file=logf_netResult )

	
	
	print( "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ", file=logf_netResult )


	print("\n\nCL COUNT:", file=logf_netResult)	

	print( "\nMean CL count for OM case: ", heur_CL_om_mean, file=logf_netResult )
	print( "Stdv CL count for OM case: ", heur_CL_om_std, file=logf_netResult )
	
	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean CL count for BASE-2 case: ", tswap_CL_om_mean, file=logf_netResult )
	print( "Stdv CL count for BASE-2 case: ", tswap_CL_om_std, file=logf_netResult )

	print( "- - - - - - - ", file=logf_netResult )

	print( "\nMean CL count for base-1 case: ", dijk_CL_mean, file=logf_netResult )
	print( "Stdv CL count for base-1 case: ", dijk_CL_std, file=logf_netResult )


	print( "----------------------------------------------------------\n\n", file=logf_netResult )
	logf_netResult.close()


	# Printing individual datum in a single file

	print("\n RUNTIMES \n", file=log_grouped_data)

	print("\n\nOM runtimes - TAKE IT:\n", file=log_grouped_data)

	for tt_h in collect_times_taken_H:
		print(tt_h, file=log_grouped_data)



	print("\nBASE-2 runtimes - TAKE IT:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_T:
		print(tt_a, file=log_grouped_data)



	# BASE-1 DIJK
	print("\nBase-1 runtimes - TAKE IT:\n", file=log_grouped_data)
	
	for tt_a in collect_times_taken_D:
		print(tt_a, file=log_grouped_data)



	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	
	print("\n NEXP \n", file=log_grouped_data)


	print("\nOM nexp:\n", file=log_grouped_data)
	
	for tnexp_h in collect_nexp_makespan_H:
		print(tnexp_h, file=log_grouped_data)

	

	print("\nBase-2 nexp:\n", file=log_grouped_data)
	
	for tnexp_t in collect_nexp_makespan_T:
		print(tnexp_t, file=log_grouped_data)


	print("\nBase-1 nexp - TAKE IT:\n", file=log_grouped_data)
	
	for texp_t in collect_no_of_exp_D:
		print(texp_t, file=log_grouped_data)


	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)
	
	print("\n CL count \n", file=log_grouped_data)

	print("\nCL count - OM:\n", file=log_grouped_data)
	
	for clc in collect_heur_CL_om:
		print(clc, file=log_grouped_data)


	print("\nBase-2 CL count:\n", file=log_grouped_data)
	
	for clc in collect_tswap_CL_om:
		print(clc, file=log_grouped_data)

	
	print("\nBase-1 CL count - TAKE IT:\n", file=log_grouped_data)
	
	for clc in collect_dijk_CL:
		print(clc, file=log_grouped_data)



	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	print("\n MAKESPAN \n", file=log_grouped_data)


	print("\nOM MAKESPAN:\n", file=log_grouped_data)
	
	for tmakespan_h in collect_makespan_H:
		print(tmakespan_h, file=log_grouped_data)

	print("\nBase-2 MAKESPAN:\n", file=log_grouped_data)
	
	for tmakespan_a in collect_makespan_T:
		print(tmakespan_a, file=log_grouped_data)


	print("\nBase-1 MAKESPAN:\n", file=log_grouped_data)
	
	for tmakespan_a in collect_makespan_D:
		print(tmakespan_a, file=log_grouped_data)


	print("\n - - - - - - - - - -- - - - - -- - - -- - \n", file=log_grouped_data)

	print("\n COST \n", file=log_grouped_data)


	print("\nOM cost:\n", file=log_grouped_data)
	
	for tcost_h in collect_total_cost_H:
		print(tcost_h, file=log_grouped_data)

	print("\nBase-2 cost:\n", file=log_grouped_data)
	
	for tcost_a in collect_total_cost_T:
		print(tcost_a, file=log_grouped_data)

	print("\nBase-1 cost:\n", file=log_grouped_data)
	
	for tcost_a in collect_total_cost_D:
		print(tcost_a, file=log_grouped_data)

	

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

