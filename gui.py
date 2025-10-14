import streamlit as st
import yaml
import pandas as pd
import os
import operator
from datetime import datetime
import pydeck as pdk
import numpy as np
from pyproj import Transformer
import rasterio
import matplotlib.pyplot as plt

DATA_DIR = '/media/striest/offroad/rosbags/20251009/teleop'
GASCOLA2LATLON = Transformer.from_crs("EPSG:32617", "EPSG:4326", always_xy=True)
DEFAULT_TIFF = '/home/tartandriver/tartandriver_ws/src/core/mission_manager/gps_maps/gascola.tif'

@st.cache_resource
def load_tiff(tif_path):
    tif = rasterio.open(tif_path)
    rgb_map = tif.read([1,2,3])
    rgb_map = np.transpose(rgb_map, [1,2,0])

    return rgb_map, tif

# --- Load dataset metadata ---
records = []
for folder in os.listdir(DATA_DIR):
    meta_path = os.path.join(DATA_DIR, folder, "info.yaml")
    if not os.path.isfile(meta_path):
        continue
    with open(meta_path) as f:
        meta = yaml.safe_load(f)
    meta["dataset"] = folder
    meta["full_path"] = os.path.join(DATA_DIR, folder)
    try:
        meta["date"] = datetime.strptime(meta["date"], "%Y-%m-%d_%H-%M-%S")
    except Exception:
        pass

    records.append(meta)

df = pd.json_normalize(records)

st.title("Dataset Query Builder")


def infer_type(series):
    if pd.api.types.is_numeric_dtype(series):
        return "numeric"
    elif pd.api.types.is_datetime64_any_dtype(series):
        return "date"
    elif isinstance(series.iloc[0], list):
        return "list"
    elif isinstance(series.iloc[0], str):
        # heuristic: try parse date
        try:
            pd.to_datetime(series.iloc[0])
            return "date"
        except Exception:
            return "string"
    else:
        return "string"

types = {col: infer_type(df[col]) for col in df.columns if col != "dataset"}

# --- Operator sets by type ---
ops = {
    "string": {
        "equals": operator.eq,
        "not equals": operator.ne,
        "contains": lambda a, b: b.lower() in str(a).lower(),
    },
    "numeric": {
        "equals": operator.eq,
        "greater than": operator.gt,
        "less than": operator.lt,
        "between": lambda a, b: b[0] <= a <= b[1],
    },
    "date": {
        "before": operator.lt,
        "after": operator.gt,
    },
    "list": {
        "contains": lambda a, b: b in a,
        "does not contain": lambda a, b: b not in a,
    },
}

# --- Build UI for filters ---
st.sidebar.header("Filters")
num_filters = st.sidebar.number_input("Number of filters", 1, 5, 1)

filters = []
for i in range(int(num_filters)):
    with st.sidebar.expander(f"Filter {i+1}", expanded=True):
        field = st.selectbox("Field", list(types.keys()), key=f"field{i}")
        ftype = types[field]
        op_label = st.selectbox("Operator", list(ops[ftype].keys()), key=f"op{i}")

        # Value input adapts to type
        if ftype == "numeric":
            if op_label == "between":
                low = st.number_input("Min", key=f"low{i}")
                high = st.number_input("Max", key=f"high{i}")
                val = (low, high)
            else:
                val = st.number_input("Value", key=f"val{i}")
        elif ftype == "date":
            dates = pd.to_datetime(df[field], errors='coerce')
            min_date, max_date = dates.min(), dates.max()
            date_val = st.date_input("Date", key=f"date{i}", min_value=min_date.date(), max_value=max_date.date())
            val = pd.Timestamp(date_val)
        elif ftype == "list":
            # Auto-suggest from unique flattened items
            unique_items = sorted({x for sublist in df[field] for x in (sublist if isinstance(sublist, list) else [])})
            val = st.selectbox("Value", unique_items, key=f"val{i}")
        elif ftype == "string":
            unique_vals = sorted(df[field].dropna().unique().tolist())
            val = st.selectbox("Value", unique_vals, key=f"val{i}")
        else:
            val = st.text_input("Value", key=f"val{i}")

        negate = st.checkbox("NOT", key=f"not{i}")
        filters.append((field, ops[ftype][op_label], val, negate))

logic = st.sidebar.radio("Combine filters using", ["AND", "OR"])

# --- Apply filters ---
mask = pd.Series(True, index=df.index) if logic == "AND" else pd.Series(False, index=df.index)
for field, fn, val, negate in filters:
    col = df[field]
    result = col.apply(lambda x: fn(x, val))
    if negate:
        result = ~result
    if logic == "AND":
        mask &= result
    else:
        mask |= result

subset = df[mask]

# --- Display results ---
st.subheader(f"Matching datasets: {len(subset)}")
st.dataframe(subset)

if st.checkbox("Show summary"):
    st.write(subset.describe(include='all'))

if not subset.empty:
    tif_path = DEFAULT_TIFF 
    rgb_map, tif = load_tiff(tif_path)
    fig, ax = plt.subplots()
    ax.imshow(rgb_map)
    minr = np.inf
    minc = np.inf
    maxr = -np.inf
    maxc = -np.inf
    for _, row in subset.iterrows():
        gps_file = os.path.join(row["full_path"], "gps.npy")
        if not os.path.exists(gps_file):
            continue
        gps = np.load(gps_file)

        rows, cols = tif.index(-gps[:,1], gps[:,0])
        rows = np.array(rows)
        cols = np.array(cols)

        minr = min(minr, rows.min())
        minc = min(minc, cols.min())
        maxr = max(maxr, rows.max())
        maxc = max(maxc, cols.max())

        ax.plot(cols, rows, '-')

    margin = 50
    ax.set_xlim(minc - margin, maxc + margin)
    ax.set_ylim(maxr + margin, minr - margin) 

    st.pyplot(fig)
    
else:
    st.info("No datasets match your filters.")