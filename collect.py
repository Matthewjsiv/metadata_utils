import rosbag
import yaml
import subprocess
import datetime
import argparse
import os
import pprint
from datetime import datetime
from pathlib import Path

import parsing
import post
from generate_tmuxp_config import generate_tmuxp_config

#use yaml to launch everything and configure bagging
#open tmux

with open('config.yaml') as f:
        CONFIG = yaml.safe_load(f)

def main(args):
    pp = pprint.PrettyPrinter(indent=4)
    fname = args.name
    with open(fname) as f:
        md = yaml.safe_load(f)

    assert os.path.exists(md['data_folder']), "{} does not exist".format(md['data_folder'])
    now = "{:%Y-%m-%d-%H-%M-%S}".format(datetime.now())


    if args.descriptor is None:
        descriptor = now
    else:
        descriptor = args.descriptor
        now = now + '_' + descriptor

    if 'no_folder' not in md.keys():


        Path(md['data_folder'] + '/' + md['experiment_name'] + '/' + now).mkdir(parents=True, exist_ok=True)

        bn = os.path.join(md['data_folder'], md['experiment_name'], now)

        #TODO: add new keywords to dict in config
        #print("NEW KEYWORDS ADDED: []")

        # fout = os.path.join(bn, os.path.basename(fname))
        fout = os.path.join(bn, "info.yaml")
        with open(fout, "w") as f:
            yaml.dump(md, f)

        fout = os.path.join(bn, 'config.yaml')
        with open(fout, "w") as f:
            yaml.dump(CONFIG, f)

        # import shutil
        # shutil.copy('temp.bag', bn)

        print('created experiment dir in {} ...'.format(bn))
    else:
        bn = './platform_launch'
        os.makedirs(bn,exist_ok=True)

    tmuxp_config = generate_tmuxp_config(CONFIG, md,now=now,descriptor=descriptor)

    
    fout = os.path.join(bn, 'tmuxp_config.yaml')
    with open(fout, "w") as f:
        yaml.dump(tmuxp_config, f)

    os.execlp('tmuxp', 'tmuxp', 'load', fout)

#    post.process(fout)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', help='name of template')
    parser.add_argument('--descriptor', help='name of template')
    args = parser.parse_args()
    main(args)
