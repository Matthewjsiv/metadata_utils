import os
import yaml
import rosbag
import datetime
import rasterio
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)

def gps_poses(md, baglist, metrics):
    """
    Extract gps postiions and timestamps from bags
    """
    gps_pos = []
    gps_ts = []

    for bag in baglist:
        for topic, msg, t in bag.read_messages(topics=[CONFIG['metrics']['gps']['topic']]):
            p = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
            gps_pos.append(p)
            gps_ts.append(t.to_sec())

    gps_pos = np.stack(gps_pos, axis=0)
    gps_ts = np.array(gps_ts)

    ss = np.linalg.norm(gps_pos[1:, :3] - gps_pos[:-1, :3], axis=-1)
    total_dist = np.sum(ss[ss < 10.])

    metrics['gps_poses'] = gps_pos
    metrics['gps_times'] = gps_ts
    metrics['total_distance'] = total_dist

def sensors(md, baglist):
    #TODO: validate somehow by # of messages
    sensors = []
    for bag in baglist:
        topics = bag.get_type_and_topic_info()[1].keys()

        for sensor in CONFIG['sensors']:
            req_topics = CONFIG['sensors'][sensor]
            valid = True
            for topic in req_topics:
                if topic not in topics:
                    valid = False
                    break
            if valid:
                sensors.append(sensor)
    md['sensors'] = sensors

def interventions(md, baglist, metrics):
    """
    Count/postprocess interventions
    """
    interventions = []
    intervention_ts = []

    for bag in baglist:
        for topic, msg, t in bag.read_messages(topics=[CONFIG['metrics']['intervention']['topic']]):
            interventions.append(msg.data)
            intervention_ts.append(t.to_sec())

    interventions = np.array(interventions)
    intervention_ts = np.array(intervention_ts)

    start_intervention_idxs = np.argwhere(interventions[1:] & ~interventions[:-1]).reshape(-1)
    stop_intervention_idxs = np.argwhere(interventions[:-1] & ~interventions[1:]).reshape(-1)

    ## real interventions start from a start idx and persist until the nearest bigger stop idx
    res = []
    for sidx in start_intervention_idxs:
        idiffs = stop_intervention_idxs - sidx

        if np.any(idiffs > 0):
            idiffs[idiffs < 0] = 1e10
            fidx = np.argmin(idiffs)
            fidx = stop_intervention_idxs[fidx]
            res.append(np.array([sidx, fidx]))

    res = np.stack(res, axis=0)
    intervention_ts = intervention_ts[res]

    metrics['intervention_ts'] = intervention_ts

def compute_metrics(md, baglist, metrics):
    """
    Compute metrics off of data. Current list:
        total distance
        total auto distance
        avg auto speed
        top auto speed
        num interventions (filtered)
    """
    imask = np.zeros(len(metrics['gps_times'])).astype(bool)
    for iinterval in metrics['intervention_ts']:
        mask = (metrics['gps_times'] > iinterval[0]) & (metrics['gps_times'] < iinterval[1])
        imask = mask | imask

    auto_poses = metrics['gps_poses'][~imask]
    auto_ts = metrics['gps_times'][~imask]

    ss = np.linalg.norm(auto_poses[1:, :2] - auto_poses[:-1, :2], axis=-1)
    ss = ss[ss < 10.] #check skips
    
    auto_dist = ss.sum()
    auto_speed = np.linalg.norm(auto_poses[:, 7:10], axis=-1)

    metrics['auto_distance'] = auto_dist
    metrics['avg_auto_speed'] = auto_speed.mean()
    metrics['max_auto_speed'] = auto_speed.max()
    metrics['auto_mask'] = ~imask
    metrics['auto_speed'] = auto_speed

def make_metrics_fig(md, tiff_fp, metrics):
    dat = rasterio.open(tiff_fp, 'r')

    gps_map = dat.read()[:3]/255.

    plt.imshow(np.transpose(gps_map,(1,2,0)))

    ## plot trajectory ##
    utmx = -metrics['gps_poses'][metrics['auto_mask']][:, 1]
    utmy = metrics['gps_poses'][metrics['auto_mask']][:, 0]

    row, col = dat.index(utmx, utmy)
    row = np.array(row)
    col = np.array(col)

    m1 = plt.scatter(col, row, c=metrics['auto_speed'], cmap='plasma', s=1., label='traj')
    plt.colorbar(m1)

    utmx = -metrics['gps_poses'][~metrics['auto_mask']][:, 1]
    utmy = metrics['gps_poses'][~metrics['auto_mask']][:, 0]

    row, col = dat.index(utmx, utmy)
    row = np.array(row)
    col = np.array(col)

    plt.scatter(col, row, c='r', marker='x', label='traj')

    plt.title('{:.2f}km traveled ({} interventions, {:.2f}m/s avg, {:.2f}m/s max)'.format(
        metrics['auto_distance']/1000.,
        metrics['intervention_ts'].shape[0],
        metrics['avg_auto_speed'],
        metrics['max_auto_speed'],
    ))

    return plt.gcf()
