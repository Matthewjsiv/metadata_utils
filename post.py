import rosbag
import yaml
import subprocess
import datetime
import parsing
import argparse
import os
import glob

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

    with open(fname, "w") as f:
        yaml.dump(md, f)

def main(args):

    prefix = args.folder
    exp_dirs = os.listdir(prefix)

    # print(exp_dirs)
    for dir in exp_dirs:
        fname = prefix + '/' + dir + '/'
        fdirs = os.listdir(fname)
        bn = glob.glob(fname + "*.bag")[0]
        # print(bn)

        bag = rosbag.Bag(bn)

        with open(fname + 'info.yaml') as f:
            md = yaml.safe_load(f)

        info_dict = yaml.safe_load(subprocess.Popen(['rosbag', 'info', '--yaml', bn], stdout=subprocess.PIPE).communicate()[0])
        md['duration'] = info_dict['duration']
        # print(info_dict)
    #
        parsing.sensors(md, bag)
        parsing.interventions(md, bag)

        with open(fname + 'info.yaml', "w") as f:
            yaml.dump(md, f)

if __name__ == "__main__":
    # main()
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', help='folder to run in')
    args = parser.parse_args()
    main(args)
