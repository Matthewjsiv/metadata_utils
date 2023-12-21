import rosbag
import yaml
import subprocess
import datetime
import numpy as np

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)

def sensors_algz(md, baglist):
    #TODO: validate somehow by # of messages
    sensors = []
    algz = []
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
        for alg in CONFIG['algz']:
            req_topics = CONFIG['algz'][alg]
            valid = True
            for topic in req_topics:
                if topic not in topics:
                    valid = False
                    break
            if valid:
                algz.append(alg)
    md['sensors'] = list(set(sensors))
    md['algz'] = list(set(algz))

def interventions(md, baglist):
    intervening = True
    num_interventions = 0

    for bag in baglist:
        for topic, msg, t in bag.read_messages(topics=[CONFIG['intervention']]):
            if (not intervening) and msg.data:
                num_interventions += 1
            intervening = msg.data
    md['interventions'] = num_interventions

def top_speed(md, baglist,savegps=True):
    top_speed = 0
    speed_total = 0
    measurement_num = 0
    if savegps:
        gps = []
    for bag in baglist:
        for topic, msg, t in bag.read_messages(topics=[CONFIG['top_speed']]):
            speed = np.linalg.norm([msg.twist.twist.linear.x,msg.twist.twist.linear.y, msg.twist.twist.linear.z])

            if savegps:
                gps.append([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.twist.twist.linear.x,msg.twist.twist.linear.y, msg.twist.twist.linear.z])
            # print(speed)
            if speed > top_speed:
                top_speed = speed
            speed_total += speed
            measurement_num += 1




    if measurement_num >= 1:
        average_speed = speed_total/measurement_num
    else:
        average_speed = 0
        print("NO MEASUREMENTS")
    # print(type(average_speed), type(top_speed))
    md['top_speed'] = float(top_speed)
    md['average_speed'] = float(average_speed)

    if savegps:
        gps = np.array(gps)
        return gps
