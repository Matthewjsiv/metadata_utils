import os
import argparse

import rasterio
import numpy as np
import matplotlib.pyplot as plt

from pathlib import Path
from rosbags.highlevel import AnyReader
from matplotlib.colors import Normalize

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag_dir', type=str, required=True, help='run dir ot postproc')
    parser.add_argument('--tif_fp', help='path to site TIF (leave empty for gascola)')
    parser.add_argument('--cvar_topic', type=str, required=False, default='/cost_cvar')
    args = parser.parse_args()

    DEFAULT_TIFF = os.path.join(os.environ['TARTANDRIVER_HOME'], 'src/core/mission_manager/gps_maps/gascola.tif')
    
    tif_path = DEFAULT_TIFF if args.tif_fp is None else args.tif_fp
    tif = rasterio.open(tif_path)
    
    bag_data = np.load(os.path.join(args.bag_dir, 'run_data.npz'))

    cvar_vals = []
    cvar_times = []

    with AnyReader([Path(args.bag_dir)]) as reader:
            connections = [c for c in reader.connections]
            cvar_conn = [c for c in reader.connections if c.topic == args.cvar_topic]
            assert len(cvar_conn) == 1
            for connection, timestamp, rawdata in reader.messages(connections=cvar_conn):
                msg = reader.deserialize(rawdata, connection.msgtype)
                cvar_vals.append(msg.data)
                cvar_times.append(timestamp * 1e-9)

    cvar_vals = np.array(cvar_vals)
    cvar_times = np.array(cvar_times)

    ##rescale vals to gps times
    cvar_rescaled = []
    for t in bag_data['times']:
         dts = np.abs(t - cvar_times)
         cvar_rescaled.append(cvar_vals[dts.argmin()])

    cvar_rescaled = np.array(cvar_rescaled)

    gps = bag_data['gps']

    rgb_map = tif.read([1,2,3])
    rgb_map = np.transpose(rgb_map, [1,2,0])

    pts = np.stack([tif.index(-x[1], x[0]) for x in gps], axis=0)
    rows = pts[:, 0]
    cols = pts[:, 1]


    plt.imshow(rgb_map)
    # plt.plot(cols, rows, '-r')
    aaa = plt.scatter(cols, rows, c=cvar_rescaled, cmap='magma_r', s=1.)
    plt.scatter(cols[0], rows[0], marker='s', c='y', label='start')
    plt.scatter(cols[-1], rows[-1], marker='>', c='y', label='end')

    margin = 50
    plt.xlim(cols.min() - margin, cols.max() + margin)
    plt.ylim(rows.max() + margin, rows.min() - margin)

    plt.title('CVaR Plot')
    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')

    cbar = plt.colorbar(aaa)
    cbar.set_label('CVaR (lower=more agggressive)')

    # plt.show()
    plt.savefig(os.path.join(args.bag_dir, 'cvar_traj.png'), dpi=300, bbox_inches='tight')
    plt.clf()
    plt.close('all')