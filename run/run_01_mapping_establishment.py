import os 
import argparse
import time
parser = argparse.ArgumentParser()
parser.add_argument("--path_to_exe",default="/media/i9/MyPassport/start/workspace/paper_massive/multi_thread_cpp/build/mproc",help="input")
parser.add_argument("--mode",default="Mapping_Establishment",help="input")
parser.add_argument("--path_to_raw",default="/media/i9/MyPassport/experiment_massive/00_Raw/luming_crop_2_raw.ply",help="input")
parser.add_argument("--path_to_low",default="/media/i9/MyPassport/experiment_massive/T04/ply/luming_crop_2_low.ply",help="output")
parser.add_argument("--path_to_cleaned_low",default="/media/i9/MyPassport/experiment_massive/T04/ply/luming_crop_2_cleaned_low.ply",help="output")
parser.add_argument("--path_to_hash_table",default="/media/i9/MyPassport/experiment_massive/T04/ply/luming_crop_2_ht.txt",help="output")
args = parser.parse_args()

run_str="{} {} {} {} {} {}".format(args.path_to_exe,
                                    args.mode,
                                    args.path_to_raw,
                                    args.path_to_low,
                                    args.path_to_cleaned_low,
                                    args.path_to_hash_table)

Tstart=time.time()
os.system(run_str)
Tstop=time.time()
print(Tstop-Tstart)
    