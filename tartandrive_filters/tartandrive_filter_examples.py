import mplt
import numpy as np
import pandas as pd

import rosbag
from os import listdir
from os.path import join
from pyproj import Proj
import torch

from dataset_filters import *

import yaml

if __name__ == '__main__':
    mplt.register_api_key('AIzaSyCnxI2B01c7kxX-t8yxyJfunjfxaFfdspk')

    ###Split Using GPS bounding boxes (can generate a yaml file using map_gui.py)
    pre = '/media/matthew/Extreme Pro/dataset_test'
    with open('all.yaml', 'r') as file:
        boxlist = yaml.safe_load(file)

    out = tartandrive_bbox_filter(pre,boxlist,'odom',all_or_none=False)
    # mplt.tartandrive_dictviz(out) ##viz results instead of saving to dataset
    # dataset_from_dict(out,'test_dataset', symlink_dense=True) #save to dataset

    ###Split by properties in metadata
    # out = tartandrive_metadata_filter(pre,['pre','conditions'],['dry','wet'],'odom',clean_gps=True)
    # out = tartandrive_metadata_filter(pre,['pre','conditions'],['morning','midday','early_evening','late_afternoon','evening','afternoon'],'odom',clean_gps=True)

    ###Split by speed
    out = tartandrive_speed_filter(pre,[.5,3,5,7,9],'odom',clean_gps=True)

    mplt.tartandrive_dictviz(out) ##viz results instead of saving to dataset
    # dataset_from_dict(out,'test_dataset', symlink_dense=True) #save to dataset
