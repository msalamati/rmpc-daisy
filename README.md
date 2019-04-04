# rmpc-daisy

# Requirements
*  Clone daisy from [here](https://gitlab.mpi-sws.org/AVA/daisy);
*  Run `git checkout robustMPC`;
*  Run `sbt compile` and `sbt script` in the home directory of daisy project;
*  Matlab TODO;
*  YALMIP TODO;
*  tbxmanager TODO;

# Install
*  Clone this repository;
*  Copy the all content of folder */RMPC_scripts* in the home directory of daisy project;
*  In both *RMPC_Aircraft.m* and *RMPC_Pendulum.m* substitute the following lines:
    * addpath(genpath('/home/roccosalvia/Documents/MATLAB/YALMIP-master')) with the path of your local YALMIP;
    * addpath(genpath('/home/roccosalvia/Documents/MATLAB/MATLAB_files')) TODO;
    * addpath(genpath('/home/roccosalvia/Documents/MATLAB/MPT/tbxmanager')) with the path of your local tbxmanager;
*  Run `python executor.py`;
