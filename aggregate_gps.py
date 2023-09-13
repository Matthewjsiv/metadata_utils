import rosbag
import yaml
import subprocess
import datetime
import parsing
import argparse
import os
import glob
import numpy as np
from tqdm import tqdm
import shutil

def main(args):

    prefix = args.folder
    savedir = args.savedir

    if not os.path.exists(savedir):
        os.mkdir(savedir)
    exp_dirs = os.listdir(prefix)
    print(prefix)
    print(exp_dirs)

    # print(exp_dirs)
    for dir in tqdm(exp_dirs):
        fname = prefix + '/' + dir + '/'
        fdirs = os.listdir(fname)

        inname = prefix + '/' + dir + '/gps.npy'
        outname = savedir + '/' + dir + '_gps.npy'

        shutil.copy(inname,outname)



if __name__ == "__main__":
    # main()
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', help='folder to run in')
    parser.add_argument('--savedir', help='folder to save in')
    args = parser.parse_args()
    main(args)
