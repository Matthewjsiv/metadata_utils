import yaml
import subprocess
import datetime
import numpy as np
from tqdm import tqdm

from tartandriver_utils.ros_utils import stamp_to_time

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)


def sensors_algz(md, connections):
    sensors = []
    algz = []
    topics = [c.topic for c in connections]

    for sensor, req_topics in CONFIG['sensors'].items():
        if all(topic in topics for topic in req_topics):
            sensors.append(sensor)
    for alg, req_topics in CONFIG['algz'].items():
        if all(topic in topics for topic in req_topics):
            algz.append(alg)

    md['sensors'] = list(set(sensors))
    md['algz'] = list(set(algz))

def get_bag_data(md, reader, connections):
    """
    Extract relevant auto data from bag
    TODO think about config for this
    """
    output_data = {
        'intervention': np.zeros([0, 2]), #[intervention, time]
        'gps': np.zeros([0, 14]), #[13dof pose, time]
    }

    all_topics = {
        'intervention': CONFIG['intervention_topic'],
        'gps': CONFIG['gps_topic']
    }

    conn = [c for c in connections if c.topic in all_topics.values()]

    for connection, timestamp, rawdata in tqdm(reader.messages(connections=conn)):
        msg = reader.deserialize(rawdata, connection.msgtype)

        if connection.topic == all_topics['intervention']:
            t = stamp_to_time(msg.header.stamp)
            i = msg.data
            output_data['intervention'] = np.concatenate([
                output_data['intervention'],
                np.array([i, t]).reshape(1, 2)
            ], axis=0)

        if connection.topic == all_topics['gps']:
            t = stamp_to_time(msg.header.stamp)
            posedata = np.array([
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
                msg.twist.twist.angular.z,
                t
            ])

            output_data['gps'] = np.concatenate([
                output_data['gps'],
                posedata.reshape(1,14)
            ], axis=0)

    #rescale intervention data to gps timestamps
    if len(output_data['intervention']) > 0:
        idata = output_data['intervention'][:, :-1]
        i_ts = output_data['intervention'][:, -1]
        gdata = output_data['gps'][:, :-1]
        gps_ts = output_data['gps'][:, -1]

        gps_data_new = []
        intervention_data_new = []
        times_new = []

        for gi, gt in enumerate(gps_ts):
            dts = np.abs(gt - i_ts)
            ii = np.argmin(dts, )
            dtmin = dts[ii]

            if dtmin < 0.1:
                gps_data_new.append(gdata[gi])
                intervention_data_new.append(idata[ii])
                times_new.append(gt)

        gps_data_new = np.stack(gps_data_new, axis=0)
        intervention_data_new = np.concatenate(intervention_data_new, axis=0)
        times = np.stack(times_new, axis=0)
        speed = np.linalg.norm(gps_data_new[:, 7:10], axis=-1)

    else:
        print('couldnt find intervention data in bag!')
        gps_data_new = output_data['gps'][:, :-1]
        times = output_data['gps'][:, -1]
        intervention_data_new = np.ones(times.shape[0])
        speed = np.linalg.norm(gps_data_new[:, 7:10], axis=-1)
    
    return {
        'gps': gps_data_new,
        'speed': speed,
        'intervention': intervention_data_new,
        'times': times
    }

def top_speed(md, reader, connections):
    topic = CONFIG['top_speed']
    if topic not in [c.topic for c in connections]:
        md['top_speed'] = 0
        md['average_speed'] = 0
        return np.empty((0, 6))

    conn = [c for c in connections if c.topic == topic][0]
    all_speed = []
    timestamps = []
    speed_total = 0
    measurement_num = 0
    gps = []

    for connection, timestamp, rawdata in tqdm(reader.messages(connections=[conn])):
        msg = reader.deserialize(rawdata, connection.msgtype)
        v = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        p = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        speed = np.linalg.norm(v)

        gps.append(np.concatenate([p, v]))
        timestamps.append(timestamp * 1e-9)

        all_speed.append(speed)
        speed_total += speed
        measurement_num += 1

    all_speed = np.asarray(all_speed)
    timestamps = np.asarray(timestamps)

    sort_idx = np.argsort(timestamps)
    timestamps, all_speed = timestamps[sort_idx], all_speed[sort_idx]

    dt = np.diff(timestamps)
    still_mask = all_speed[:-1] < .2

    total_still_time = dt[still_mask].sum()

    top_speed = all_speed.max()
    average_speed = all_speed.mean() if measurement_num else 0
    md['motion'] = {}
    md['motion']['max_vel'] = float(top_speed)
    md['motion']['mean_vel'] = float(average_speed)
    md['motion']['still_time'] = float(total_still_time)
    return np.array(gps)

