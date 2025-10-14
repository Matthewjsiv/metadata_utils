import yaml, json
import subprocess
import datetime
import parsing
import argparse
import os
import glob
import numpy as np
from tqdm import tqdm
from rosbags.highlevel import AnyReader
from pathlib import Path
import rasterio
import matplotlib.pyplot as plt

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)

DEFAULT_TIFF = '/home/tartandriver/tartandriver_ws/src/core/mission_manager/gps_maps/gascola.tif'

def get_ros2_bag_info(bag_path):
    """Read metadata.yaml from a ROS 2 bag directory and extract duration + start date."""
    meta_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.exists(meta_path):
        raise FileNotFoundError(f"No metadata.yaml found in {bag_path}")

    with open(meta_path, "r") as f:
        meta = yaml.safe_load(f)

    info = meta.get("rosbag2_bagfile_information", meta)

    # Duration (convert from nanoseconds)
    duration_ns = info.get("duration", {}).get("nanoseconds", 0)
    duration_s = duration_ns * 1e-9

    # Start time (convert nanoseconds_since_epoch to datetime)
    start_ns = info.get("starting_time", {}).get("nanoseconds_since_epoch", 0)
    start_dt = datetime.datetime.fromtimestamp(start_ns * 1e-9)

    return {
        "duration": duration_s,
        "start_time": start_dt,
        "start_time_str": start_dt.strftime("%Y-%m-%d_%H-%M-%S"),
    }

def gen_gps_summary(gps, dir, tif_path = None):
    tif_path = DEFAULT_TIFF if tif_path is None else tif_path

    tif = rasterio.open(tif_path)
    rgb_map = tif.read([1,2,3])
    rgb_map = np.transpose(rgb_map, [1,2,0])

    rows, cols = tif.index(-gps[:,1], gps[:,0])
    rows = np.array(rows)
    cols = np.array(cols)

    plt.imshow(rgb_map)
    plt.plot(cols, rows, '-r')

    margin = 50
    plt.xlim(cols.min() - margin, cols.max() + margin)
    plt.ylim(rows.max() + margin, rows.min() - margin) 

    # plt.show()
    plt.savefig(os.path.join(dir, 'traj.png'), dpi=300, bbox_inches='tight')
    plt.clf()
    plt.close('all')

    vels = np.linalg.norm(gps[:,3:6], axis=-1)
    plt.hist(vels, bins=14, range=(0,15))
    # plt.show()
    plt.savefig(os.path.join(dir, 'vels.png'), dpi=300, bbox_inches='tight')
    plt.clf()
    plt.close('all')

def main(args):

    prefix = args.run_dir
    exp_dirs = os.listdir(prefix)
    print(prefix)
    # print(exp_dirs)

    # print(exp_dirs)
    for dir in tqdm(exp_dirs):
        fname = os.path.join(prefix, dir)
        
        with open(os.path.join(fname, 'info.yaml')) as f:
            md = yaml.safe_load(f)

        print(fname)

        info_dict = get_ros2_bag_info(fname)
        md['duration'] = info_dict['duration']
        md['date'] = info_dict['start_time_str']
 
        with AnyReader([Path(fname)]) as reader:
            connections = [c for c in reader.connections]

            parsing.sensors_algz(md, connections)

            # parsing.interventions(md, reader, connections)
            if 'top_speed' not in md:
                gps = parsing.top_speed(md, reader, connections)

        np.save(os.path.join(fname, 'gps'),gps)

        with open(os.path.join(fname, 'info.yaml'), "w") as f:
            yaml.dump(md, f)

        gen_gps_summary(gps, fname)


        

# def main(args):
#
#     prefix = args.folder
#     exp_dirs = os.listdir(prefix)
#     print(prefix)
#     print(exp_dirs)
#
#     duration = 0
#     # print(exp_dirs)
#     for dir in tqdm(exp_dirs):
#         fname = prefix + '/' + dir + '/'
#         fdirs = os.listdir(fname)
#         print(fdirs)
#         # bn = glob.glob(fname + "*.bag")
#         # print(fname)
#
#         for dir in fdirs:
#             if 'active' in dir:
#                 print(dir)
#
#
#
#     #     if os.path.exists(fname + 'gps.npy'):
#     #         print('skipping')
#     #         continue
#     #     # print(bn)
#     #
#         # baglist = []
#         # for b in bn:
#         #     bag = rosbag.Bag(b)
#         #     baglist.append(bag)
#         #
#         #
#         # total_duration = 0
#         # for b in bn:
#         #     info_dict = yaml.safe_load(subprocess.Popen(['rosbag', 'info', '--yaml', b], stdout=subprocess.PIPE).communicate()[0])
#         #     # total_duration += info_dict['duration']
#         # # md['duration'] = total_duration
#         # duration += total_duration
#         # print(duration)
#         # print(info_dict)
#     # #
#     #     # parsing.sensors(md, baglist)
#     #     # parsing.interventions(md, baglist)
#     #     gps = parsing.top_speed(md, baglist)
#     #
#     #     np.save(fname + 'gps',gps)
#     #
#     #     with open(fname + 'info.yaml', "w") as f:
#     #         yaml.dump(md, f)

if __name__ == "__main__":
    # main()
    parser = argparse.ArgumentParser()
    parser.add_argument('--run_dir', help='folder to run in')
    args = parser.parse_args()
    main(args)
