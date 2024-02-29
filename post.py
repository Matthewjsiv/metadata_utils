import rosbag
import yaml
import subprocess
import datetime
import parsing
import argparse
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)

def process(fname):
    #TODO: Fix directories
    with open(fname) as f:
        md = yaml.safe_load(f)

    bn = md['folder'] + '/' + md['bagname'].replace('.bag','/') + md['bagname']
    bag = rosbag.Bag(bn)

    info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bn], stdout=subprocess.PIPE).communicate()[0])
    md['duration'] = info_dict['duration']
    # print(info_dict)

    parsing.sensors(md, bag)
    parsing.interventions(md, bag)
    parsing.top_speed(md, bag)

    with open(fname, "w") as f:
        yaml.dump(md, f)

def main(args):

    prefix = args.folder
    exp_dirs = os.listdir(prefix)
    print(prefix)
    print(exp_dirs)

    # print(exp_dirs)
    for rdir in tqdm(exp_dirs):
        fname = prefix + '/' + rdir + '/'
        fdirs = os.listdir(fname)
        bn = glob.glob(fname + "*.bag")
        # print(bn)

        baglist = []
        for b in sorted(bn):
            bag = rosbag.Bag(b)
            baglist.append(bag)

        with open(fname + 'info.yaml') as f:
            md = yaml.safe_load(f)

        total_duration = 0
        for b in bn:
            info_dict = yaml.safe_load(subprocess.Popen(['rosbag', 'info', '--yaml', b], stdout=subprocess.PIPE).communicate()[0])
            total_duration += info_dict['duration']
        md['duration'] = total_duration
        # print(info_dict)

        metrics = {}
        try:
            parsing.gps_poses(md, baglist, metrics)
            parsing.interventions(md, baglist, metrics)
            parsing.compute_metrics(md, baglist, metrics)

            fig = parsing.make_metrics_fig(md, args.map_fp, metrics)

            plt.savefig(os.path.join(args.folder, rdir, 'viz.png'))
            plt.close()
            np.savez(os.path.join(args.folder, rdir, 'metrics_data'), **metrics)
        except:
            print('skipping...')

#        with open(fname + 'info.yaml', "w") as f:
#            yaml.dump(md, f)

if __name__ == "__main__":
    # main()
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=str, required=True, help='folder to run in')
    parser.add_argument('--map_fp', type=str, required=True, help='path to gps data (.tif, default in gps_maps)')
    args = parser.parse_args()
    main(args)
