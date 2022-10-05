import rosbag
import yaml
import subprocess
import datetime
import parsing
import argparse
import os
from datetime import datetime
from pathlib import Path
import post

#use yaml to launch everything and configure bagging
#open tmux

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)
    
def main(args):
    #TODO fix directories
    fname = args.name
    with open(fname) as f:
        md = yaml.safe_load(f)
    
    Path(md['folder']).mkdir(exist_ok=True)
    now = "{:%Y_%m_%d_%H_%M_%S}".format(datetime.now())
    Path(md['folder'] + '/' + now).mkdir(parents=True, exist_ok=True)

    
    bn = md['folder'] + '/' + now + '/' + now + '.bag'
    md['bagname'] = now + '.bag'
    
    #TODO: add new keywords to dict in config
    #print("NEW KEYWORDS ADDED: []")
    
    fout = md['folder'] + '/' + now + '/' + now + '.yaml'
    with open(fout, "w") as f:
        yaml.dump(md, f)
    
    # import shutil
    # shutil.copy('temp.bag', bn)
    
    post.process(fout)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('name', help='name of template')
    args = parser.parse_args()
    main(args)
