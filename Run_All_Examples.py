#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 30 15:43:27 2022

@author: bayat2
"""

import sys
import os
# Navigate two levels up and then to the utils folder
current_dir = os.path.dirname(os.path.abspath(__file__))
utils_dir = os.path.join(current_dir, "utils")  # Add the utils folder
sys.path.insert(0, utils_dir)  # Add utils folder to sys.path

import numpy as np
import math
#from LGR_Data import LGR_Class
#from cyipopt import minimize_ipopt
from scipy.optimize import rosen, rosen_der
from scipy.interpolate import interp1d
from scipy.optimize import minimize
from scipy.interpolate import lagrange
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.font_manager import FontProperties
import pickle as pkl
fontP=FontProperties()
#fontP.set_size('x-large')
from MPC_Class import Read_Result

import runpy

# Run the file
runpy.run_path(os.path.dirname(os.path.abspath(__file__))+"/Examples/Ex1"+"/Ex1_Comparison_Plots.py")
runpy.run_path(os.path.dirname(os.path.abspath(__file__))+"/Examples/Ex2"+"/Ex2_Comparison_Plots.py")
runpy.run_path(os.path.dirname(os.path.abspath(__file__))+"/Examples/Ex3"+"/Ex3_Comparison_Plots.py")
runpy.run_path(os.path.dirname(os.path.abspath(__file__))+"/Examples/Ex4"+"/Ex4_Comparison_Plots.py")