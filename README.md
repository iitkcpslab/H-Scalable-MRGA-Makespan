-------------------------------------------------------------------------------------------------------------
A Scalable Multi-Robot Goal Assignment Algorithm for Minimizing Mission Time
-------------------------------------------------------------------------------------------------------------

This repository contains the implementation of the proposed approach OM as well as the baseline algorithms Base-1 and Base-2 discussed in the following paper: 

**Aakash and Indranil Saha. Optimal Makespan in a Minute Timespan! A Scalable Multi-Robot Goal Assignment Algorithm for Minimizing Mission Time. In AAAI 2024.**

***We would greatly appreciate your acknowledgment of our paper through citations when using or modifying any code, whether it's a snippet or an entire module obtained from this repository.***


### Guidelines for Experiments

Either 
- run ```python3 main_om_base1_base2.py``` command to execute OM, Base-1 and Base-2, or
- run ```python3 main_om_base2_with_path_planning.py``` command to execute OM and Base-2, with path planning module of TSWAP after OM


The program prompts the user to provide on-screen inputs.

om.py, base1.py, and base2.py contains the implementation of OM, BASE-1, and BASE-2 respectively.

Both our algorithm and the baselines use the common functionalities from the utils folder. 

The standard workspaces are provided in benchmark_maps folder and are ready-to-use. 

The benchmark_ws_translator.py file can be used to translate any new map into the required format.

The 3D maps in .map file format do not need translation. 


For any query, kindly contact Aakash at aakashp@cse.iitk.ac.in
