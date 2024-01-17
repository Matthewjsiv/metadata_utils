import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
import matplotlib.image as mpimg
from google_static_maps_api import GoogleStaticMapsAPI

import yaml

from pyproj import Proj
MYPROJ = Proj(proj='utm',zone=17,ellps='WGS84', preserve_units=False)

def onpick(event):

    try: # use try/except in case we are not using Qt backend
        zooming_panning = ( fig.canvas.cursor().shape() != 0 ) # 0 is the arrow, which means we are not zooming or panning.
    except:
        zooming_panning = False

    if zooming_panning:
        print('zoom mode, ignoring click')
        return

    global COUNT, POINTS
    COUNT += 1

    x = event.xdata
    y = event.ydata
    y /= bg.shape[0]*.97
    y *= YLIM[1]-YLIM[0]
    y += YLIM[0]-10

    x /= bg.shape[1]*1.02
    x *= XLIM[1]-XLIM[0]
    x += XLIM[0]+15

    utm_x = -y
    utm_y = x
    # print(utm_x,utm_y)

    lon,lat = MYPROJ(utm_y, -utm_x,inverse=True)
    print(lon,lat)
    POINTS.append(lat)
    POINTS.append(lon)

    if COUNT == 2:
        plt.close(fig)



BRLL = [40.45153990414565,-79.78015626907073]
TLLL = [40.46687443404498,-79.79493624443535]

# UTM 17
BR_UTM = np.array([603437.68,4478590.17])
TL_UTM = np.array([602161.13,4480275.08])
# scope = np.abs(TL_UTM-BR_UTM)
XLIM = [602161.13,603437.68]
YLIM = [4478590.17,4480275.08]

bg = mpimg.imread('gascola_earth.png')
bg = np.flipud(bg)


data = {}

# Get the filename from the user
filename = input("Enter the filename for the YAML file (e.g. 'zones_split'): ")

while True:
    COUNT = 0
    POINTS = []
    # Ask the user to enter a word
    word = input("Enter a zone name (or 'q' to quit): ")

    # Check if the user wants to quit
    if word.lower() == 'q':
        break

    fig, ax = plt.subplots()
    ax.imshow(bg,origin='lower')

    fig.canvas.mpl_connect('button_press_event', onpick)
    plt.show()

    # Add the word as a key to the dictionary
    data[word] = {}
    data[word]['type'] = 'bbox'
    data[word]['coords'] = POINTS

# Save the dictionary to the YAML file
with open(filename + '.yaml', 'w') as yaml_file:
    yaml.dump(data, yaml_file)
