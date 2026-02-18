import yaml
import datetime
import parsing
import argparse
import os
import numpy as np
from tqdm import tqdm
from rosbags.highlevel import AnyReader
from pathlib import Path
import rasterio
import matplotlib.pyplot as plt

from gps_plotting import *

from tartandriver_utils.os_utils import load_yaml

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)

DEFAULT_TIFF = os.path.join(os.environ['TARTANDRIVER_HOME'], 'src/core/mission_manager/gps_maps/gascola.tif')

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

def bagdata_is_autonomous(bagdata):
    """
    Detect if bagdata is autonomous. Check for nonzero velocity and no intervention flag
    """
    imask = bagdata['intervention'] > 0.5
    sdata = bagdata['speed']

    if imask.all():
        return False
    
    auto_speeds = sdata[~imask]
    return auto_speeds.max() > 1.

def compute_default_metrics(bagdata):
    """
    Compute teleop metrics from run data. This includes:
        1. still time
        2. avg/top speed
        3. traversed distance
    """
    poses = bagdata['gps']
    imask = bagdata['intervention'] > 0.5
    speeds = bagdata['speed']
    times = bagdata['times']

    top_speed = speeds.max()
    avg_speed = speeds.mean()

    ds = np.linalg.norm(poses[1:, :2] - poses[:-1, :2], axis=-1)
    ds = np.concatenate([np.zeros(1), ds], axis=0)

    dist_traveled = ds.sum()

    dt = times[1:] - times[:-1]
    still_mask = speeds[1:] < 0.2
    still_time  = dt[still_mask].sum()

    return {
        'top_speed (m/s)' : top_speed.item(),
        'avg_speed (m/s)' : avg_speed.item(),
        'dist_traveled (km)': dist_traveled.item() / 1000.,
        'still_time (s)': still_time.item(),
    }

def compute_auto_metrics(bagdata):
    """
    Compute autonomy metrics from run data. This includes:
        1. num interventions
        2. avg/top autonomous speed
        3. traversed auto distance
    """
    poses = bagdata['gps']
    imask = bagdata['intervention'] > 0.5
    speeds = bagdata['speed']
    times = bagdata['times']

    #num interventions = num times of False->True
    num_interventions = (imask[1:] & ~imask[:-1]).sum()

    top_auto_speed = speeds[~imask].max()

    avg_auto_speed = speeds[~imask].mean()

    ds = np.linalg.norm(poses[1:, :2] - poses[:-1, :2], axis=-1)
    ds = np.concatenate([np.zeros(1), ds], axis=0)

    dist_auto_traveled = ds[~imask].sum()

    dt = times[1:] - times[:-1]
    dt = np.concatenate([np.zeros(1), dt], axis=0)
    auto_time = dt[~imask].sum()

    return {
        'num_interventions': num_interventions.item(),
        'top_auto_speed (m/s)' : top_auto_speed.item(),
        'avg_auto_speed (m/s)' : avg_auto_speed.item(),
        'dist_auto_traveled (km)': dist_auto_traveled.item() / 1000.,
        'auto_time (s)': auto_time.item()
    }

def main(args):
    prefix = args.run_dir
    exp_dirs = os.listdir(prefix)
    print(prefix)

    tif_path = DEFAULT_TIFF if args.tif_fp is None else args.tif_fp
    tif = rasterio.open(tif_path)

    # print(exp_dirs)
    for dir in tqdm(exp_dirs):
        try:
            fname = os.path.join(prefix, dir)
            
            info_fp = os.path.join(fname, 'info.yaml')
            if os.path.exists(info_fp):
                md = load_yaml(info_fp)
            else:
                md = {}

            print(fname)

            info_dict = get_ros2_bag_info(fname)
            md['duration'] = info_dict['duration']
            md['date'] = info_dict['start_time_str']

            with AnyReader([Path(fname)]) as reader:
                connections = [c for c in reader.connections]

                parsing.sensors_algz(md, connections)

                bag_data = parsing.get_bag_data(md, reader, connections)

            md['metrics'] = {}
            md['metrics']['default'] = compute_default_metrics(bag_data)
            is_auto = bagdata_is_autonomous(bag_data)
            if is_auto:
                print('auto-detected run has auto data. Computing metrics...')
                md['metrics']['autonomy'] = compute_auto_metrics(bag_data)

            np.savez(os.path.join(fname, 'run_data'), **bag_data)

            with open(os.path.join(fname, 'info.yaml'), "w") as f:
                yaml.dump(md, f)

            ##make gps plots
            for plt_fn in [
                basic_gps_plot,
                speed_gps_plot,
                intervention_gps_plot,
                speed_histogram,
                traj_plot
            ]:
                plt_fn(bag_data, fname, tif, legend=False)
        except:
            print(f'ruh roh {dir} failled')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--run_dir', help='folder to run in')
    parser.add_argument('--tif_fp', help='path to site TIF (leave empty for gascola)')
    args = parser.parse_args()
    main(args)
