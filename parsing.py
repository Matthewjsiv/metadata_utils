import yaml
import subprocess
import datetime
import numpy as np
from tqdm import tqdm


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

