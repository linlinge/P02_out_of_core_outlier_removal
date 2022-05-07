import time 
import os 
import argparse
import pandas as pd

# parser = argparse.ArgumentParser()
# parser.add_argument("--path_to_exe",default="/home/llg/workspace/paper_massive/multi_thread_cpp/build/mproc",help="input")
# parser.add_argument("--mode",default="01_Downsampling",help="input")
# parser.add_argument("--path_to_raw",default="/home/llg/workspace/experiment_massive/T01/ply/luming_crop.ply",help="input")
# parser.add_argument("--number_of_output",default="8000000",help="input")
# parser.add_argument("--path_to_save",default="/home/llg/workspace/experiment_massive/T01/ply/luming_crop_low.ply",help="ouput")
# args = parser.parse_args()

# run_str="{} {} {} {} {}".format(args.path_to_exe,args.mode,args.path_to_raw,args.number_of_output, args.path_to_save)
# Tstart=time.time()
# os.system(run_str)
# Tend=time.time()
# print(Tend-Tstart)



def read_quantity(path_to_record):
    if os.path.exists(path_to_record)==True:
        df=pd.read_csv(path_to_record,header=None)
        dat=df.values
        return dat[0]
    else:
        os.system("touch record.txt")
        return -2


# specify
path_to_raw="/media/i9/MyPassport/start/workspace/experiment_massive/T02/ply/luming_crop_10^8.ply"
path_to_record="/media/i9/MyPassport/start/workspace/paper_massive/run/record.txt"

# parameters
stop_condition=0.001
target_quantity=50000000
upper_bound=1
lower_bound=0
cell_size=(upper_bound+lower_bound)/2
pre_cell_size=0
path_to_exe="/home/llg/workspace/paper_massive/multi_thread_cpp/build/mproc"

cmd_str="{} {} {} {} {}".format(path_to_exe,"grid_sampling",path_to_raw,cell_size,path_to_record)
print(cmd_str)
os.system(cmd_str)
current_quantity= read_quantity(path_to_record)

while abs(current_quantity-target_quantity)>1:
    print("cell size: {}     current_quantity: {}".format(cell_size,current_quantity))
    if current_quantity> target_quantity:
        lower_bound=cell_size        
    else:
        upper_bound=cell_size
    cell_size=(upper_bound+lower_bound)/2

    # run again with new cell_size
    cmd_str="{} {} {} {} {}".format(path_to_exe,"grid_sampling",path_to_raw,cell_size,path_to_record)
    os.system(cmd_str)
    current_quantity=read_quantity(path_to_record) 