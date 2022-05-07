import time 
import os 
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--path_to_exe",default="/home/llg/workspace/paper_massive/multi_thread_cpp/build/mproc",help="input")
parser.add_argument("--mode",default="01_Downsampling",help="input")
parser.add_argument("--path_to_raw",default="/home/llg/workspace/experiment_massive/T01/ply/luming_crop.ply",help="input")
parser.add_argument("--number_of_output",default="8000000",help="input")
parser.add_argument("--path_to_save",default="/home/llg/workspace/experiment_massive/T01/ply/luming_crop_low.ply",help="ouput")
args = parser.parse_args()

run_str="{} {} {} {} {}".format(args.path_to_exe,args.mode,args.path_to_raw,args.number_of_output, args.path_to_save)
Tstart=time.time()
os.system(run_str)
Tend=time.time()
print(Tend-Tstart)