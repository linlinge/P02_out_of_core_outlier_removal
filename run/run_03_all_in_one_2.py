import os 
import argparse
import time
parser = argparse.ArgumentParser()
parser.add_argument("--path_to_exe",default="/media/i9/MyPassport/start/workspace/paper_massive/multi_thread_cpp/build/mproc",help="input")
parser.add_argument("--mode",default="all_in_one_part2",help="input")
parser.add_argument("--path_to_raw",default="/media/i9/MyPassport/experiment_massive/A_shared_data/huizhongli.ply",help="input")
parser.add_argument("--path_to_component",default="/media/i9/MyPassport/experiment_massive/F08/ply/hzl_cleaned_components.txt",help="input")
parser.add_argument("--path_to_hash_table",default="/media/i9/MyPassport/experiment_massive/F08/ply/ht.txt",help="input")
parser.add_argument("--path_to_merge",default="/media/i9/MyPassport/experiment_massive/F08/ply/hzl_out.ply",help="output")
args = parser.parse_args()

run_str="{} {} {} {} {} {}".format(args.path_to_exe,
                                    args.mode,
                                    args.path_to_raw,                                    
                                    args.path_to_component,
                                    args.path_to_hash_table,
                                    args.path_to_merge)

print(run_str)
# Tstart=time.time()
os.system(run_str)
# Tstop=time.time()
# print(Tstop-Tstart)
