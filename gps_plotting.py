import os
import numpy as np
import matplotlib.pyplot as plt

def traj_plot(bagdata, dir, tif=None, legend=True):
    traj = bagdata['super_odometry']
    plt.plot(traj[:, 0], traj[:, 1], c='r')
    plt.gca().set_aspect(1.)
    
    plt.title('Traj')
    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')
    plt.savefig(os.path.join(dir, 'odometry.png'), dpi=300, bbox_inches='tight')
    plt.clf()
    plt.close('all')

def basic_gps_plot(bagdata, dir, tif, legend=True, time_plot=True):
    """
    Args:
        bagdata: data to plot from
        dir: dir to save plot
        tif: tif to get gpsinfo from
    """
    gps = bagdata['gps']

    rgb_map = tif.read([1,2,3])
    rgb_map = np.transpose(rgb_map, [1,2,0])

    pts = np.stack([tif.index(-x[1], x[0]) for x in gps], axis=0)
    rows = pts[:, 0]
    cols = pts[:, 1]

    plt.imshow(rgb_map)
    
    if time_plot:
        z = bagdata['times']
        z = (z - z[0]) / (z[-1] - z[0])
        aaa = plt.scatter(cols, rows, c=z, cmap='jet', s=1.)
        cbar = plt.colorbar(aaa)
        cbar.set_label('Elapsed Time (Normalized)')
    else:
        plt.scatter(cols, rows, c='r', s=1.)

    plt.scatter(cols[0], rows[0], marker='s', c='y', label='start')
    plt.scatter(cols[-1], rows[-1], marker='>', c='y', label='end')

    margin = 50
    plt.xlim(cols.min() - margin, cols.max() + margin)
    plt.ylim(rows.max() + margin, rows.min() - margin)

    plt.title('GPS Plot')
    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')

    if legend:
        plt.legend()

    # plt.show()
    plt.savefig(os.path.join(dir, 'traj.png'), dpi=300, bbox_inches='tight')
    plt.clf()
    plt.close('all')

def intervention_gps_plot(bagdata, dir, tif, legend=True):
    """
    color the trajectory by teleop/auto

    Args:
        bagdata: data to plot from
        dir: dir to save plot
        tif: tif to get gpsinfo from
    """
    gps = bagdata['gps']
    mask = bagdata['intervention'] > 0.5

    iidxs = np.argwhere(mask[1:] & ~mask[:-1]).flatten()

    rgb_map = tif.read([1,2,3])
    rgb_map = np.transpose(rgb_map, [1,2,0])

    pts = np.stack([tif.index(-x[1], x[0]) for x in gps], axis=0)
    rows = pts[:, 0]
    cols = pts[:, 1]

    plt.imshow(rgb_map)
    # plt.plot(cols, rows, '-r')
    plt.scatter(cols[~mask], rows[~mask], c='b', label='is autonomy', s=1.)
    plt.scatter(cols[mask], rows[mask], c='r', label='is ntervention', s=1.)
    plt.scatter(cols[iidxs], rows[iidxs], c='r', marker='x', label='intervention pt')
    plt.scatter(cols[0], rows[0], marker='s', c='y', label='start')
    plt.scatter(cols[-1], rows[-1], marker='>', c='y', label='end')

    margin = 50
    plt.xlim(cols.min() - margin, cols.max() + margin)
    plt.ylim(rows.max() + margin, rows.min() - margin)

    plt.title(f'Intervention Plot ({iidxs.shape[0]} total)')
    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')

    if legend:
        plt.legend()

    # plt.show()
    plt.savefig(os.path.join(dir, 'intervention_traj.png'), dpi=300, bbox_inches='tight')
    plt.clf()
    plt.close('all')

def speed_gps_plot(bagdata, dir, tif, legend=True):
    """
    color the trajectory by teleop/auto

    Args:
        bagdata: data to plot from
        dir: dir to save plot
        tif: tif to get gpsinfo from
    """
    gps = bagdata['gps']
    speed = bagdata['speed']

    rgb_map = tif.read([1,2,3])
    rgb_map = np.transpose(rgb_map, [1,2,0])

    pts = np.stack([tif.index(-x[1], x[0]) for x in gps], axis=0)
    rows = pts[:, 0]
    cols = pts[:, 1]

    plt.imshow(rgb_map)
    # plt.plot(cols, rows, '-r')
    aaa = plt.scatter(cols, rows, c=speed, cmap='jet', s=1., vmax=10.)
    plt.scatter(cols[0], rows[0], marker='s', c='y', label='start')
    plt.scatter(cols[-1], rows[-1], marker='>', c='y', label='end')

    margin = 50
    plt.xlim(cols.min() - margin, cols.max() + margin)
    plt.ylim(rows.max() + margin, rows.min() - margin)

    plt.title('Speed Plot')
    plt.xlabel('X(m)')
    plt.ylabel('Y(m)')

    cbar = plt.colorbar(aaa)
    cbar.set_label('Speed (m/s)')

    if legend:
        plt.legend()

    # plt.show()
    plt.savefig(os.path.join(dir, 'speed_traj.png'), dpi=300, bbox_inches='tight')
    plt.clf()
    plt.close('all')

def speed_histogram(bagdata, dir, tif, legend=True):
    """
    Args:
        bagdata: data to plot from
        dir: dir to save plot
        tif: tif to get gpsinfo from
    """
    speed = bagdata['speed']
    mask = bagdata['intervention'] < 0.5

    hist_kwargs = {
        'bins': 14,
        'range': (0, 15),
        'histtype': 'step'
    }

    plt.hist(speed, label='all speed', color='r', **hist_kwargs)

    if mask.any():
        plt.hist(speed[mask], label='auto speed', color='b', **hist_kwargs)

    plt.title('Speed histogram')
    plt.xlabel('Speed (m/s)')
    plt.ylabel('Count')

    if legend:
        plt.legend()

    plt.savefig(os.path.join(dir, 'vels.png'), dpi=300, bbox_inches='tight')
    plt.clf()
    plt.close('all')