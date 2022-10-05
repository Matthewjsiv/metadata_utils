import rosbag
import yaml
import subprocess
import datetime
import parsing

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
        
def main():
    
    fname = 'pre.yaml'
    
    with open(fname) as f:
        md = yaml.safe_load(f)
    
    bn = md['bagname']
    bag = rosbag.Bag(bn)
    
    info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bn], stdout=subprocess.PIPE).communicate()[0])
    md['duration'] = info_dict['duration']
    # print(info_dict)
    
    parsing.sensors(md, bag)
    parsing.interventions(md, bag)
    
    with open(fname, "w") as f:
        yaml.dump(md, f)

if __name__ == "__main__":
    main()
