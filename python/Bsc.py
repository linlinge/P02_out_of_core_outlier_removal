import os
import sys 
import numpy as np
def list_all_files(rootdir):
    import os
    _files = []

    #列出文件夹下所有的目录与文件
    list_file = os.listdir(rootdir)
    
    for i in range(0,len(list_file)):

        # 构造路径
        path = os.path.join(rootdir,list_file[i])

        # 判断路径是否是一个文件目录或者文件
        # 如果是文件目录，继续递归        
        if os.path.isdir(path):
            _files.extend(list_all_files(path))
        if os.path.isfile(path):
             _files.append(path)
    return _files

def mkdir(path):
    # 引入模块
    import os
 
    # 去除首位空格
    path=path.strip()
    # 去除尾部 \ 符号
    path=path.rstrip("\\")
 
    # 判断路径是否存在
    # 存在     True
    # 不存在   False
    isExists=os.path.exists(path)
 
    # 判断结果
    if not isExists:
        # 如果不存在则创建目录
        # 创建目录操作函数
        os.makedirs(path) 
        print(path+' create sucess')
        return True
    else:
        # 如果目录存在则不创建，并提示目录已存在
        print(path+' path exist !')
        return False


def get_ply_header(path,mode):
    f=open(path,"a")
    if mode=="xyz":        
        head1=f.readlines()
        print(head1)
    elif mode=="xyzrgb":
        head2=f.readlines()        
        print(head2)
    f.close()
