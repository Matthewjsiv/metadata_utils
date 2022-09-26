import rosbag
import yaml
import subprocess
import datetime

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)
        
def sensors(md, bag):
    #TODO: validate somehow by # of messages
    topics = bag.get_type_and_topic_info()[1].keys()
    sensors = []
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

def interventions(md, bag):
    intervening = True
    num_interventions = 0
    for topic, msg, t in bag.read_messages(topics=[CONFIG['intervention']]):
        if (not intervening) and msg.data:
            num_interventions += 1
        intervening = msg.data
    md['interventions'] = num_interventions