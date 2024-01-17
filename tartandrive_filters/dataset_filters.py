import warnings
from os import listdir

import numpy as np
import torch
import pandas as pd
import scipy.ndimage as ndi
from matplotlib import cm
from matplotlib import pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.patches import PathPatch
from matplotlib.patches import Polygon
from matplotlib.patches import Rectangle
from matplotlib.path import Path
from scipy.spatial import ConvexHull

from google_static_maps_api import GoogleStaticMapsAPI
from google_static_maps_api import MAPTYPE
from google_static_maps_api import MAX_SIZE
from google_static_maps_api import SCALE

import rosbag
from os import listdir
import shutil
import os
from os.path import join
from pyproj import Proj
from tqdm import tqdm
import yaml

from natsort import natsorted

MYPROJ = Proj(proj='utm',zone=17,ellps='WGS84', preserve_units=False)

FOLDER_TYPE_AIO = ['cmd','cost2','odom','tartanvo_odom','super_odometry'] #tartanvo_odom
FOLDER_TYPE_DENSE = ['depth_left', 'height_map', 'image_left', 'image_left_color', 'image_right','point_cloud_0','point_cloud_1','points_left','rgb_map', 'full_cloud']
POSSIBLE_DENSE_EXTS = ['.png','.jpg','.npy']
FOLDER_TYPE_HIGH_FREQ = {'imu':10}

def generate_intervals(input_list, resolution):
    newlist = [np.arange(number*resolution,(number+1)*resolution) for number in input_list]
    newlist = np.array(newlist).reshape(-1)
    return newlist

def get_gps_vel(dirname, odom_source):
    ###For normal
    break_signal = False
    if odom_source is not None:
        gps = np.load(dirname + '/' + odom_source + '/odometry.npy')
        vel = np.linalg.norm(gps[:,7:10],axis=1)
        if len(gps) == 0:
            break_signal = True
    ####
    #for simple
    if odom_source is None:
        gps = np.load(dirname + '/gps.npy')
        if len(gps) == 0:
            break_signal = True
            return gps, vel, break_signal
        vel = np.linalg.norm(gps[:,3:],axis=1)
    ####
    return gps, vel, break_signal

def parse_AIO(indir,outdir,ids,res=1):
    ## For types that have all samples in a single .npy file per modality (alongisde timestamps.txt)
    # e.g.
    if not os.path.exists(outdir):
        os.mkdir(outdir)
    files = os.listdir(indir)

    # print(outdir)
    if len(ids) == 0:
        return
    if res != 1:
        #id -> (id*res):(id*res)+res
        # print(len(ids))
        ids = generate_intervals(ids,res)

    for file in files:
        if file.split('.')[-1] == 'txt':
            data = np.loadtxt(indir + '/' + file)[ids]
            if len(data) == 0:
                continue
            # print(data[ids].shape)
            out_file_dir = outdir + '/' + file
            if os.path.exists(out_file_dir):
                existing_data = np.loadtxt(out_file_dir)
                existing_data = np.concatenate([existing_data,data])
                np.savetxt(out_file_dir,existing_data)
            else:
                np.savetxt(out_file_dir,data)
        else:
            data = np.load(indir + '/' + file)[ids]
            out_file_dir = outdir + '/' + file
            if len(data) == 0:
                continue
            if os.path.exists(out_file_dir):
                existing_data = np.load(out_file_dir)
                # print(existing_data.shape, data.shape)
                existing_data = np.concatenate([existing_data,data])
                # print(outdir,existing_data.shape)
                np.save(out_file_dir,existing_data)
            else:
                np.save(out_file_dir,data)
            # print(data[ids].shape)

def parse_dense(indir,outdir,ids,symlink=True):
    #notes - need to make sure the indexes start from 0 and go up
    #check for timestamps file separately

    if len(ids) == 0:
        return

    if not os.path.exists(outdir):
        os.mkdir(outdir)
    # files = os.listdir(indir)
    #
    # for file in files:
    if os.path.exists(indir + '/timestamps.txt'):
        file = 'timestamps.txt'
        data = np.loadtxt(indir + '/' + file)[ids]
        if len(data) == 0:
            return
        # print(data[ids].shape)
        out_file_dir = outdir + '/' + file
        if os.path.exists(out_file_dir):
            existing_data = np.loadtxt(out_file_dir)
            existing_data = np.concatenate([existing_data,data])
            np.savetxt(out_file_dir,existing_data)
        else:
            np.savetxt(out_file_dir,data)

    outfiles = os.listdir(outdir)
    try:
        outfiles.remove('timestamps.txt')
    except:
        ...
    if len(outfiles) == 0:
        numfiles = 0
    else:
        outfiles = natsorted(outfiles)
        # print(outfiles)
        numfiles = outfiles[-1].split('.')[0]
        # print(numfiles)
        numfiles = int(numfiles) + 1

    # print(numfiles)

    possible_exts = []
    for type in POSSIBLE_DENSE_EXTS:
        if os.path.exists(indir + '/000000' + type):
            possible_exts.append(type)

    for i in ids:
        format_in = '{0:06d}'.format(i)

        format_out = '{0:06d}'.format(numfiles)
        # print(format_out)
        for type in possible_exts:
            in_file_path = indir + '/' + format_in + type
            out_file_path = outdir + '/' + format_out + type
            # print(in_file_path, out_file_path)
            if symlink:
                os.symlink(in_file_path,out_file_path)
            else:
                shutil.copyfile(in_file_path, out_file_path)

        numfiles += 1

    # print(numfiles)

def dataset_from_dict(out,name,symlink_dense=True):
    if not os.path.exists(name):
        os.mkdir(name)
    else:
        print("ERROR: DATASET ALREADY EXISTS")

    i = 0
    for subfolder in tqdm(out.keys()):
        print('Generating data for: ' + subfolder)
        subfolder_path = os.path.join(name, subfolder)

        if not os.path.exists(subfolder_path):
            os.mkdir(subfolder_path)

        length = len(out[subfolder]['coords'])
        # print(length)
        if length == 0:
            continue


        for trajname in out[subfolder].keys():
            if trajname == 'coords':
                continue

            types = os.listdir(trajname)

            for type in types:
                type_folder = trajname + '/' + type
                out_type_folder = subfolder_path + '/' + type
                if type in FOLDER_TYPE_AIO:
                    parse_AIO(type_folder,out_type_folder,out[subfolder][trajname])
                elif type in FOLDER_TYPE_DENSE:
                    parse_dense(type_folder,out_type_folder,out[subfolder][trajname],symlink=symlink_dense)
                elif type in FOLDER_TYPE_HIGH_FREQ:
                    parse_AIO(type_folder,out_type_folder,out[subfolder][trajname],res=FOLDER_TYPE_HIGH_FREQ[type])
        # print('-----')



def tartandrive_metadata_filter(prefix,keys,subfolders,odom_source,clean_gps=False):
    dirs = listdir(prefix)
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []

    out = {}


    TOT_DUR = 0.0
    for name in subfolders:

        out[name] = {'coords':np.empty((0,2))}

        for dir in tqdm(dirs):
            dirname = join(prefix, dir)

            with open(dirname + '/info.yaml', 'r') as file:
                metadata = yaml.safe_load(file)

            TOT_DUR += metadata['duration'] / (60*60)
            print(metadata['duration'] / (60*60), ' duration')
            for key in keys:
                metadata = metadata[key]

            print(metadata)

            if name not in metadata:
                continue

            gps, vel, break_signal = get_gps_vel(dirname, odom_source)
            if break_signal:
                continue

            tlon,tlat =  MYPROJ(-gps[:,1], gps[:,0],inverse=True)

            if clean_gps:
                ids = np.where((tlat != 0.0))[0]
            else:
                ids = np.arange(tlat.shape[0])

            if len(ids) > 0:
                #if last value is included, need to remove since tartanvo_motion doesn't have
                if ids[-1] == len(tlon)-1:
                    ids = ids[:-1]

                insidelon = tlon[ids]
                insidelat = tlat[ids]

                coords = np.array([insidelon,insidelat]).T
                out[name]['coords'] = np.vstack([out[name]['coords'], coords])
                out[name][dirname] = ids.tolist()
                # print(out[name]['coords'].shape)
    return out

def tartandrive_speed_filter(prefix,bins,odom_source,clean_gps=False):
    dirs = listdir(prefix)
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []

    out = {}

    prev = bins[0]
    for name in bins[1:]:

        out[str(name)] = {'coords':np.empty((0,2))}

        for dir in tqdm(dirs):
            dirname = join(prefix, dir)


            gps, vel, break_signal = get_gps_vel(dirname, odom_source)
            if break_signal:
                continue
            # print(vel.shape)
            gps = gps[:,:2]
            tlon,tlat =  MYPROJ(-gps[:,1], gps[:,0],inverse=True)

            if clean_gps:
                # print(prev,name)
                ids = np.where((vel >= prev) & (vel < name) & (tlat != 0.0))[0]
                # ids = np.where((tlat != 0.0))[0]
                # print(ids.shape)
            else:
                ids = np.where((vel >= prev) & (vel < name))[0]



            if len(ids) > 0:
                #if last value is included, need to remove since tartanvo_motion doesn't have
                if ids[-1] == len(tlon)-1:
                    ids = ids[:-1]

                insidelon = tlon[ids]
                insidelat = tlat[ids]

                coords = np.array([insidelon,insidelat]).T
                np.save('parv_course_5ms',coords)
                out[str(name)]['coords'] = np.vstack([out[str(name)]['coords'], coords])
                out[str(name)][dirname] = ids.tolist()
                # print(out[name]['coords'].shape)

        prev = name

    return out


def tartandrive_bbox_filter(prefix,boxlist,odom_source,strict=False,savedir=None, all_or_none=False):
    dirs = listdir(prefix)
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []

    out = {}
    if all_or_none:
        out['other'] = {'coords':np.empty((0,2))}
    for name,val in boxlist.items():
        coords = val['coords']
        # minx,miny,maxx,maxy = val['coords']
        minx = np.minimum(coords[0],coords[2])
        miny = np.minimum(coords[1],coords[3])
        maxx = np.maximum(coords[0],coords[2])
        maxy = np.maximum(coords[1],coords[3])


        out[name] = {'coords':np.empty((0,2))}


        for dir in tqdm(dirs):
            dirname = join(prefix, dir)
            # print(dirname)
            gps, vel, break_signal = get_gps_vel(dirname, odom_source)
            if break_signal:
                continue
                
            tlon,tlat =  MYPROJ(-gps[:,1], gps[:,0],inverse=True)

            ids = np.where((tlat > minx) & (tlat < maxx) & (tlon > miny) & (tlon < maxy) & (tlat != 0.0))[0]


            if len(ids) > 0:
                #if last value is included, need to remove since tartanvo_motion doesn't have
                if ids[-1] == len(tlon)-1:
                    ids = ids[:-1]


                insidelon = tlon[ids]
                insidelat = tlat[ids]

                coords = np.array([insidelon,insidelat]).T
                out[name]['coords'] = np.vstack([out[name]['coords'], coords])
                out[name][dirname] = ids.tolist()
                # print(out[name]['coords'].shape)

            if all_or_none:
                others = np.where((tlat < minx) | (tlat > maxx) | (tlon < miny) | (tlon > maxy) & (tlat != 0.0))[0]

                if len(others)>0:
                    #if last value is included, need to remove since tartanvo_motion doesn't have
                    if others[-1] == len(tlon)-1:
                        others = others[:-1]

                    otherlon = tlon[others]
                    otherlat = tlat[others]
                    coords = np.array([otherlon,otherlat]).T
                    # name = 'other'
                    out['other']['coords'] = np.vstack([out['other']['coords'], coords])
                    out['other'][dirname] = others.tolist()

    return out

def torch_bbox_filter(prefix,boxlist,strict=False,savedir=None,all_or_none=False,symlink=False):
    files = listdir(prefix)
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []
    # print(files[:10])

    if savedir is not None:
        if not os.path.exists(savedir):
            os.mkdir(savedir)
            os.mkdir(savedir + '/other')

    out = {}
    for name,val in boxlist.items():
        coords = val['coords']
        # minx,miny,maxx,maxy = val['coords']
        minx = np.minimum(coords[0],coords[2])
        miny = np.minimum(coords[1],coords[3])
        maxx = np.maximum(coords[0],coords[2])
        maxy = np.maximum(coords[1],coords[3])

        out[name] = []
        for file in tqdm(files):
            # print(file)
            filepathname = join(prefix, file)
            gps = torch.load(filepathname)['gps_traj'].numpy()
            # print(gps.shape)
            # print(gps[:2,0])
            tlon,tlat =  MYPROJ(-gps[:,1], gps[:,0],inverse=True)


            if not strict:
                tlat = tlat[[0,-1]]
                tlon = tlon[[0,-1]]

            # print(tlat,tlon)
            if np.any(tlat < minx) or np.any(tlat > maxx):
                if symlink:
                    os.symlink(filepathname,savedir + '/' + 'other' + '/' + file)
                else:
                    shutil.copyfile(filepathname,savedir + '/' + 'other' + '/' + file)
                continue
            if np.any(tlon < miny) or np.any(tlon > maxy):
                if symlink:
                    os.symlink(filepathname,savedir + '/' + 'other' + '/' + file)
                else:
                    shutil.copyfile(filepathname,savedir + '/' + 'other' + '/' + file)
                continue

            out[name].append(filepathname)
            if savedir is not None:
                # print(savedir + '/' + name)
                if not os.path.exists(savedir + '/' + name):
                    os.mkdir(savedir + '/' + name)
                if symlink:
                    os.symlink(filepathname,savedir + '/' + name + '/' + file)
                else:
                    shutil.copyfile(filepathname,savedir + '/' + name + '/' + file)
    # print(out)
    return out


def torch_multi_filter(prefix,boxlist,strict=False,savedir=None):
    raise NotImplementedError
