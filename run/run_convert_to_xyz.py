import time 
import os 
import argparse
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument("--path_to_exe",default="/home/llg/workspace/paper_massive/multi_thread_cpp/build/mproc",help="input")
parser.add_argument("--mode",default="convert_to_xyz",help="input")
parser.add_argument("--path_to_raw",default="/home/llg/workspace/experiment_massive/T01/ply/luming_crop.ply",help="input")
parser.add_argument("--path_to_out",default="/home/llg/workspace/experiment_massive/T01/ply/luming_crop.xyz",help="ouput")
args = parser.parse_args()

run_str="{} {} {} {}".format(args.path_to_exe,args.mode,args.path_to_raw, args.path_to_out)
Tstart=time.time()
os.system(run_str)
Tend=time.time()
print(Tend-Tstart)
