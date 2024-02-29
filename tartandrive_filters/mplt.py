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
from os.path import join
from pyproj import Proj

from dataset_filters import *
import distinctipy as COLORS
BLANK_THRESH = 2 * 1e-3     # Value below which point in a heatmap should be blank

#UTM ZONE
ZONE = '17'
MYPROJ = Proj(proj='utm',zone=17,ellps='WGS84', preserve_units=False)

def bag2gps(filepathname, topicname='/odom'):
    bag = rosbag.Bag(filepathname, 'r')

    # myProj = Proj("+proj=utm +zone="+ZONE+", +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    # myProj = Proj(proj='utm',zone=17,ellps='WGS84', preserve_units=False)


    count = 0
    latlist = []
    longilist = []
    altilist = []
    for topic, msg, t in bag.read_messages(topics=[topicname]): # '' /target_image
        timeind = msg.header.stamp.to_nsec()%1000000000000/1000000.0
        # if starttime == -1:
        #     starttime = timeind
        #if count%skip == 0:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        lon, lat = MYPROJ(x, y,inverse=True)
        print(lat,lon)

        latlist.append(lat)
        longilist.append(lon)
        altilist.append(z)

            # print t, timeind, latitude, longitude, altitude
        count += 1

    return latlist, longilist, altilist


def register_api_key(api_key):
    """Register a Google Static Maps API key to enable queries to Google.
    Create your own Google Static Maps API key on https://console.developers.google.com.
    :param str api_key: the API key
    :return: None
    """
    GoogleStaticMapsAPI.register_api_key(api_key)


# def background_and_pixels(latitudes, longitudes, alllat, alllong, size, maptype):
#     """Queries the proper background map and translate geo coordinated into pixel locations on this map.
#     :param pandas.Series latitudes: series of sample latitudes
#     :param pandas.Series longitudes: series of sample longitudes
#     :param int size: target size of the map, in pixels
#     :param string maptype: type of maps, see GoogleStaticMapsAPI docs for more info
#     :return: map and pixels
#     :rtype: (PIL.Image, pandas.DataFrame)
#     """
#     # From lat/long to pixels, zoom and position in the tile
#     center_lat = (latitudes.max() + latitudes.min()) / 2
#     center_long = (longitudes.max() + longitudes.min()) / 2
#     zoom = GoogleStaticMapsAPI.get_zoom(latitudes, longitudes, size, SCALE)
#     pixels = GoogleStaticMapsAPI.to_tile_coordinates(latitudes, longitudes, center_lat, center_long, zoom, size, SCALE)
#     # Google Map
#     img = GoogleStaticMapsAPI.map(
#         center=(center_lat, center_long),
#         zoom=zoom,
#         scale=SCALE,
#         size=(size, size),
#         maptype=maptype,
#     )
#     return img, pixels

def background_and_pixels(latitudes, longitudes, alllat, alllong, size, maptype, zoom = None):
    """Queries the proper background map and translate geo coordinated into pixel locations on this map.
    :param pandas.Series latitudes: series of sample latitudes
    :param pandas.Series longitudes: series of sample longitudes
    :param int size: target size of the map, in pixels
    :param string maptype: type of maps, see GoogleStaticMapsAPI docs for more info
    :return: map and pixels
    :rtype: (PIL.Image, pandas.DataFrame)
    """
    # From lat/long to pixels, zoom and position in the tile
    center_lat = (alllat.max() + alllat.min()) / 2
    center_long = (alllong.max() + alllong.min()) / 2
    if zoom is None:
        zoom = GoogleStaticMapsAPI.get_zoom(alllat, alllong, size, SCALE)
    print(center_lat, center_long, zoom)
    pixels = []
    for lat, long in zip(latitudes, longitudes):
        pixels.append(GoogleStaticMapsAPI.to_tile_coordinates(pd.Series(lat), pd.Series(long), center_lat, center_long, zoom, size, SCALE))
    # Google Map
    img = GoogleStaticMapsAPI.map(
        center=(center_lat, center_long),
        zoom=zoom,
        scale=SCALE,
        size=(size, size),
        maptype=maptype,
    )
    return img, pixels


def polyline(latitudes, longitudes, closed=False, maptype=MAPTYPE, alpha=1.):
    """Plot a polyline on a map, joining (lat lon) pairs in the order defined by the input.
    :param pandas.Series latitudes: series of sample latitudes
    :param pandas.Series longitudes: series of sample longitudes
    :param bool closed: set to `True` if you want to close the line, from last (lat, lon) pair back to first one
    :param string maptype: type of maps, see GoogleStaticMapsAPI docs for more info
    :param float alpha: transparency for polyline overlay, between 0 (transparent) and 1 (opaque)
    :return: None
    """
    width = SCALE * MAX_SIZE
    img, pixels = background_and_pixels(latitudes, longitudes, MAX_SIZE, maptype)
    # Building polyline
    verts = pixels.values.tolist()
    codes = [Path.MOVETO] + [Path.LINETO for _ in range(max(pixels.shape[0] - 1, 0))]
    if closed:
        verts.append(verts[0])
        codes.append(Path.CLOSEPOLY)
    # Background map
    plt.figure(figsize=(10, 10))
    ax = plt.subplot(111)
    plt.imshow(np.array(img))
    # Polyline
    patch = PathPatch(Path(verts, codes), facecolor='none', lw=2, alpha=alpha)
    ax.add_patch(patch)
    # Axis options
    plt.gca().invert_yaxis()                                                # Origin of map is upper left
    plt.axis([0, width, width, 0])                                          # Remove margin
    plt.axis('off')
    plt.tight_layout()
    plt.show()

def multiline(latitudes, longitudes, alllat, alllong, closed=False, maptype=MAPTYPE, alpha=1., zoom = None, kids = None, cmap = None, noline=False, sname=None):
    width = SCALE * MAX_SIZE
    img, pixels = background_and_pixels(latitudes, longitudes, alllat, alllong, MAX_SIZE, maptype, zoom = zoom)
    # Building polyline
    # print(pixels)
    ##FIXXX
    # pixels = pixels[0]
    # print(pixels)
    # print(len(pixels),'PIXELS')
    print(type(pixels))
    print(type(pixels[0]))

    # Background map
    plt.figure(figsize=(10, 10))
    ax = plt.subplot(111)
    plt.imshow(np.array(img))
    # np.save('sat_map', np.array(img))
    viridis = cm.get_cmap('terrain', 5)
    # cmap = [[1,1,0], [0,0,1], [0,1,0], [1,0,0], [1,1,0], [1,0,1], [0,1,1]]
    # cmap = [[1,1,0], [0,1,0], [1,0,0], [0,0,1],[1,1,0], [1,0,1], [0,1,1]]
    # cmap = [[1,1,0], [0,1,0], [1,0,0], [0,0,1],[1,1,0], [1,0,1], [0,1,1],[1,1,1],[.5,1,.5],[1,.5,1]]
    # cmap = plt.get_cmap('tab20', 20).colors
    cmap = COLORS.get_colors(10)
    cmap  = np.array(cmap)
    # cmap = np.load('colormap.npy')

    # print(cmap.colors)
    # speedlist = [0,3,7,10,15]
    # print(len(pixels), 'PIXELS')
    for i, temp in enumerate(pixels):
        if kids is None:
            if noline:
                plt.plot(temp['x_pixel'], temp['y_pixel'], '.r')
            else:
                plt.plot(temp['x_pixel'], temp['y_pixel'], '-r', linewidth = .6)
        else:
            # print(cmap[int(kids[i])])
            # plt.plot(temp['x_pixel'], temp['y_pixel'], '-r', linewidth = .6)
            # print(kids[i])
            if noline:
                # plt.plot(temp['x_pixel'], temp['y_pixel'], '.',color = cmap[int(kids[0])],markersize=.1,linewidth=0)
                # plt.scatter(temp['x_pixel'], temp['y_pixel'],c = cmap[int(kids[i])],s=.2,linewidths=0,label= str(speedlist[i]) + '-' + str(speedlist[i+1]) + ' m/s')
                plt.scatter(temp['x_pixel'], temp['y_pixel'],c = cmap[int(kids[i])],s=.2,linewidths=0)

                # plt.scatter(temp['x_pixel'], temp['y_pixel'],c = int(kids[i]), cmap=cmap,s=4.5)
            else:
                plt.plot(temp['x_pixel'], temp['y_pixel'], linewidth = .6, color = cmap[int(kids[i])])
    # plt.legend()

    #plt.text(12,3.4,'Table Title',size=8)

    # Axis options
    plt.gca().invert_yaxis()                                                # Origin of map is upper left
    plt.axis([0, width, width, 0])                                          # Remove margin
    plt.axis('off')
    plt.tight_layout()
    # col_labels = ["◼", "◼", "◼", "◼"]
    # colors = ["yellow", "blue", "limegreen", "red"]
    # row_labels = ['Time-series', 'All Modalities']
    # # table_vals=[[.2154,.1888],[.3924,.3571],[.3552,.3085], [.3057, .2164]]
    # table_vals=[[.2154,.3924, .3552, .3057],[.1888,.3571,.3085, .2164]]
    # table = plt.table(cellText=table_vals,
    #               colWidths = [0.1]*4,
    #               rowLabels=row_labels,
    #               #colLabels=col_labels,
    #               loc='upper center')
    # for i in range(len(col_labels)):
    #     cell = table.add_cell(-1, i, width=0.1, height = .02, text=col_labels[i],
    #                           loc="center")
    #     cell.get_text().set_color(colors[i])
    # table.set_zorder(100)
    if sname is None:
        plt.show()
    else:
        plt.savefig(sname + '/cluster_viz')

def bg(latitudes, longitudes, alllat, alllong, closed=False, maptype=MAPTYPE, alpha=1., zoom = None):
    width = SCALE * MAX_SIZE
    img, pixels = background_and_pixels(latitudes, longitudes, alllat, alllong, MAX_SIZE, maptype, zoom = zoom)
    # Building polyline
    # print(pixels)
    ##FIXXX
    # pixels = pixels[0]
    # print(pixels)
    print(len(pixels))
    print(type(pixels))
    print(type(pixels[0]))

    # Background map
    plt.figure(figsize=(10, 10))
    ax = plt.subplot(111)
    plt.imshow(np.array(img))
    for temp in pixels:
        plt.plot(temp['x_pixel'], temp['y_pixel'], '-r', linewidth = .3)
    # Axis options
    plt.gca().invert_yaxis()                                                # Origin of map is upper left
    plt.axis([0, width, width, 0])                                          # Remove margin
    plt.axis('off')
    plt.tight_layout()
    plt.show()

# def filter_bb(lat, long, boxlist):

def bags2viz(files, prefix):
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []
    for file in files:
        filepathname = join(prefix, file)
        latlist, longilist, altilist = bag2gps(filepathname)

        lat_array, longi_array, alti_array = np.array(latlist), np.array(longilist), np.array(altilist)
        lat_diff, long_diff = lat_array.max() - lat_array.min(), longi_array.max() - longi_array.min()
        print
        #print(inputbagfile)
        #print(lat_array.max(), lat_array.min(), lat_diff)
        #print(longi_array.max(), longi_array.min(), long_diff)

        #lats = np.array([0,10])
        #longs = np.array([0,10])
        lat_array = lat_array[np.abs(lat_array) > 38]
        lat_array = lat_array[np.abs(lat_array) < 41]
        longi_array = longi_array[np.abs(longi_array) > 78]
        longi_array = longi_array[np.abs(longi_array) < 80]
        flatlist.append(lat_array)
        flonglist.append(longi_array)

        alllat += latlist
        alllong += longilist
    alllat = np.array(alllat)
    alllong = np.array(alllong)
    alllat = alllat[np.abs(alllat) > 38]
    alllat = alllat[np.abs(alllat) < 41]
    alllong = alllong[np.abs(alllong) > 78]
    alllong = alllong[np.abs(alllong) < 80]
    # print(alllong)
    print(np.max(alllat))
    print(np.max(alllong))
    print(np.mean(alllat))
    print(np.mean(alllong))
    multiline(flatlist,flonglist,pd.Series(alllat), pd.Series(alllong), maptype = 'satellite',zoom=15)

def np2viz(files, prefix):
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []
    for file in files:
        filepathname = join(prefix, file)
        gps = np.load(filepathname)
        latlist = gps[0,:].tolist()
        longilist = gps[1,:].tolist()
        lat_array = gps[0,:]
        longi_array = gps[1,:]

        #lats = np.array([0,10])
        #longs = np.array([0,10])
        lat_array = lat_array[np.abs(lat_array) > 37]
        lat_array = lat_array[np.abs(lat_array) < 42]
        longi_array = longi_array[np.abs(longi_array) > 77]
        longi_array = longi_array[np.abs(longi_array) < 81]
        flatlist.append(lat_array)
        flonglist.append(longi_array)

        alllat += latlist
        alllong += longilist
    alllat = np.array(alllat)
    alllong = np.array(alllong)
    alllat = alllat[np.abs(alllat) > 37]
    alllat = alllat[np.abs(alllat) < 42]
    alllong = alllong[np.abs(alllong) > 77]
    alllong = alllong[np.abs(alllong) < 81]
    # print(alllong)
    print(np.max(alllat))
    print(np.max(alllong))
    print(np.mean(alllat))
    print(np.mean(alllong))
    multiline(flatlist,flonglist,pd.Series(alllat), pd.Series(alllong), maptype = 'satellite', zoom  = 15)

def dataset2viz(prefix,bounds=None):

    flatlist = []
    flonglist = []
    alllat = []
    alllong = []

    kids = []
    if bounds is None:
        files = listdir(prefix)
        files = [join(prefix,file) for file in files]
    else:
        zone_dict = torch_bbox_filter(prefix,bounds)
        print(zone_dict)
        files = []
        i = 0
        for zone,f in zone_dict.items():
            # print(zone,f)
            files += f
            kids += [i] * len(f)
            i += 1
    # print(files)
    for file in files:
        # print(file)
        filepathname = file
        # print(filepathname)
        gps = torch.load(filepathname)['gps_traj'].numpy()
        # print(gps[:2,0])
        tlon,tlat =  MYPROJ(-gps[:,1], gps[:,0],inverse=True)
        # print(tlon[:5],tlat[:5])
        latlist = tlat.tolist()
        longilist = tlon.tolist()
        lat_array = tlat
        longi_array = tlon

        #lats = np.array([0,10])
        #longs = np.array([0,10])
        lat_array = lat_array[np.abs(lat_array) > 40.4]
        lat_array = lat_array[np.abs(lat_array) < 41]
        longi_array = longi_array[np.abs(longi_array) > 79.7]
        longi_array = longi_array[np.abs(longi_array) < 79.8]
        flatlist.append(lat_array)
        flonglist.append(longi_array)

        alllat += latlist
        alllong += longilist
    alllat = np.array(alllat)
    alllong = np.array(alllong)
    alllat = alllat[np.abs(alllat) > 40.4]
    alllat = alllat[np.abs(alllat) < 41]
    alllong = alllong[np.abs(alllong) > 79.7]
    alllong = alllong[np.abs(alllong) < 79.8]
    # print(alllong)
    print(np.max(alllat))
    print(np.max(alllong))
    print(np.mean(alllat))
    print(np.mean(alllong))
    multiline(flatlist,flonglist,pd.Series(alllat), pd.Series(alllong), maptype = 'satellite', zoom  = 15, kids = kids)

def clusterviz(files, prefix, kmeans_dict, kmeans_centers):
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []
    kids = np.array([])
    for file in files:
        filepathname = join(prefix, file)
        keyname = filepathname.split('/')[-1].split('.')[0]
        gps = np.load(filepathname)
        latlist = gps[0,:].tolist()
        longilist = gps[1,:].tolist()
        lat_array = gps[0,:]
        longi_array = gps[1,:]

        #lats = np.array([0,10])
        #longs = np.array([0,10])
        lat_array = lat_array[np.abs(lat_array) > 40.4]
        lat_array = lat_array[np.abs(lat_array) < 41]
        longi_array = longi_array[np.abs(longi_array) > 79.7]
        longi_array = longi_array[np.abs(longi_array) < 79.8]


        if keyname in kmeans_dict.keys():
            flatlist.append(lat_array)
            flonglist.append(longi_array)
            alllat += latlist
            alllong += longilist
            kids = np.hstack([kids, kmeans_dict[keyname]])

    alllat = np.array(alllat)
    alllong = np.array(alllong)
    alllat = alllat[np.abs(alllat) > 40.4]
    alllat = alllat[np.abs(alllat) < 41]
    alllong = alllong[np.abs(alllong) > 79.7]
    alllong = alllong[np.abs(alllong) < 79.8]
    # print(alllong)
    print(np.max(alllat))
    print(np.max(alllong))
    print(np.mean(alllat))
    print(np.mean(alllong))
    multiline(flatlist,flonglist,pd.Series(alllat), pd.Series(alllong), maptype = 'satellite', zoom  = 15, kids = kids, cmap = kmeans_centers)

def newclusterviz(kmeans_dict,sname=None):
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []
    kids = np.array([])
    for file in kmeans_dict.keys():
        filepathname = file
        # keyname = filepathname.split('/')[-1].split('.')[0]
        id = kmeans_dict[file]
        # print(filepathname)

        gps = torch.load(filepathname)['gps_traj'].numpy()
        # print(gps[:2,0])
        tlon,tlat =  MYPROJ(-gps[:,1], gps[:,0],inverse=True)
        # print(tlon[:5],tlat[:5])
        latlist = tlat.tolist()
        longilist = tlon.tolist()
        lat_array = tlat
        longi_array = tlon

        #lats = np.array([0,10])
        #longs = np.array([0,10])
        lat_array = lat_array[np.abs(lat_array) > 40.4]
        lat_array = lat_array[np.abs(lat_array) < 41]
        longi_array = longi_array[np.abs(longi_array) > 79.7]
        longi_array = longi_array[np.abs(longi_array) < 79.8]
        flatlist.append(lat_array)
        flonglist.append(longi_array)


        # if keyname in kmeans_dict.keys():
        alllat += latlist
        alllong += longilist
        kids = np.hstack([kids, id])
        # print(id)

    alllat = np.array(alllat)
    alllong = np.array(alllong)
    alllat = alllat[np.abs(alllat) > 40.4]
    alllat = alllat[np.abs(alllat) < 41]
    alllong = alllong[np.abs(alllong) > 79.7]
    alllong = alllong[np.abs(alllong) < 79.8]
    # print(alllong)
    print(np.max(alllat))
    print(np.max(alllong))
    print(np.mean(alllat))
    print(np.mean(alllong))
    print('lengs')
    print(len(flatlist))
    # print(len(kids))
    multiline(flatlist,flonglist,pd.Series(alllat), pd.Series(alllong), maptype = 'satellite', zoom  = 15, kids = kids,sname=sname)

def tartandrive_clusterviz(kmeans_dict,sname=None):
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []
    kids = np.array([])
    for file in kmeans_dict.keys():
        filepathname = file
        # keyname = filepathname.split('/')[-1].split('.')[0]
        id = kmeans_dict[file][0]
        # print(filepathname)

        gps = kmeans_dict[file][1].reshape(1,-1)
        # print(gps[:2,0])
        tlon,tlat =  MYPROJ(-gps[:,1], gps[:,0],inverse=True)
        # print(tlon[:5],tlat[:5])
        latlist = tlat.tolist()
        longilist = tlon.tolist()
        lat_array = tlat
        longi_array = tlon

        #lats = np.array([0,10])
        #longs = np.array([0,10])
        lat_array = lat_array[np.abs(lat_array) > 40.4]
        lat_array = lat_array[np.abs(lat_array) < 41]
        longi_array = longi_array[np.abs(longi_array) > 79.7]
        longi_array = longi_array[np.abs(longi_array) < 79.8]
        flatlist.append(lat_array)
        flonglist.append(longi_array)


        # if keyname in kmeans_dict.keys():
        alllat += latlist
        alllong += longilist
        kids = np.hstack([kids, id])
        # print(id)

    alllat = np.array(alllat)
    alllong = np.array(alllong)
    alllat = alllat[np.abs(alllat) > 40.4]
    alllat = alllat[np.abs(alllat) < 41]
    alllong = alllong[np.abs(alllong) > 79.7]
    alllong = alllong[np.abs(alllong) < 79.8]
    # print(alllong)
    print(np.max(alllat))
    print(np.max(alllong))
    print(np.mean(alllat))
    print(np.mean(alllong))
    print('lengs')
    print(len(flatlist))
    # print(len(kids))
    multiline(flatlist,flonglist,pd.Series(alllat), pd.Series(alllong), maptype = 'satellite', zoom  = 15, kids = kids, noline=True,sname=sname)

def tartandrive_dictviz(boxlist,sname=None):
    flatlist = []
    flonglist = []
    alllat = []
    alllong = []
    kids = np.array([])
    for i,key in enumerate(boxlist.keys()):
        coords = boxlist[key]['coords']
        tlon,tlat = coords[:,0],coords[:,1]
        latlist = tlat.tolist()
        longilist = tlon.tolist()
        lat_array = tlat
        longi_array = tlon

        #lats = np.array([0,10])
        #longs = np.array([0,10])
        lat_array = lat_array[np.abs(lat_array) > 40.4]
        lat_array = lat_array[np.abs(lat_array) < 41]
        longi_array = longi_array[np.abs(longi_array) > 79.7]
        longi_array = longi_array[np.abs(longi_array) < 79.8]
        flatlist.append(lat_array)
        flonglist.append(longi_array)


        # if keyname in kmeans_dict.keys():
        alllat += latlist
        alllong += longilist
        kids = np.hstack([kids, i])
        # print(id)

    alllat = np.array(alllat)
    alllong = np.array(alllong)
    alllat = alllat[np.abs(alllat) > 40.4]
    alllat = alllat[np.abs(alllat) < 41]
    alllong = alllong[np.abs(alllong) > 79.7]
    alllong = alllong[np.abs(alllong) < 79.8]
    # print(alllong)
    print(np.max(alllat))
    print(np.max(alllong))
    print(np.mean(alllat))
    print(np.mean(alllong))
    print('lengs')
    print(len(flatlist))
    # print(len(kids))
    multiline(flatlist,flonglist,pd.Series(alllat), pd.Series(alllong), maptype = 'satellite', zoom  = 15, kids = kids, noline=True,sname=sname)
