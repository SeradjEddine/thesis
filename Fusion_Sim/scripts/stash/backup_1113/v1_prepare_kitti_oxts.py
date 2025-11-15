import os
import glob
import pandas as pd
import numpy as np

def load_oxts_file(file_path):
    """Parse a single OXTS line into a dict of useful values."""
    with open(file_path, 'r') as f:
        line = f.readline().strip()
    values = [float(x) for x in line.split()]

    return {
        "lat": values[0],
        "lon": values[1],
        "alt": values[2],
        "roll": values[3],
        "pitch": values[4],
        "yaw": values[5],
        "vn": values[6],
        "ve": values[7],
        "vf": values[8],
        "vl": values[9],
        "vu": values[10],
        "ax": values[11],
        "ay": values[12],
        "az": values[13],
        "af": values[14],
        "al": values[15],
        "au": values[16],
        "wx": values[17],
        "wy": values[18],
        "wz": values[19],
        "wf": values[20],
        "wl": values[21],
        "wu": values[22],
    }

def parse_timestamp_ns(ts: str) -> int:
    """Parse KITTI timestamp string into nanoseconds since epoch (int64)."""
    base, frac = ts.split(".")
    frac = (frac + "000000000")[:9]
    ns = int(frac)
    base_epoch = pd.Timestamp(base).to_datetime64().astype("datetime64[s]").astype(np.int64)
    return base_epoch * 1_000_000_000 + ns

def prepare_kitti_sequence(oxts_base_dir, output_csv):
    """Convert OXTS txt files + timestamps into a clean CSV with relative nanosecond time."""

    timestamps_file = os.path.join(oxts_base_dir, "timestamps.txt")
    oxts_data_dir = os.path.join(oxts_base_dir, "data")

    with open(timestamps_file, 'r') as f:
        timestamps = [line.strip() for line in f.readlines()]

    files = sorted(glob.glob(os.path.join(oxts_data_dir, "*.txt")))
    if len(files) != len(timestamps):
        raise ValueError("Number of OXTS files and timestamps differ!")

    rows = []
    for ts, fpath in zip(timestamps, files):
        data = load_oxts_file(fpath)
        data["timestamp"] = ts
        rows.append(data)

    df = pd.DataFrame(rows)
    ns_times = np.array([parse_timestamp_ns(ts) for ts in df["timestamp"]], dtype=np.int64)
    rel_ns = ns_times - ns_times[0]
    df["rel_time"] = [f"{ns/1e9:.9f}" for ns in rel_ns]

    df.to_csv(output_csv, index=False)
    print(f"Saved {len(df)} entries to {output_csv}")

if __name__ == "__main__":
    base_dir = os.path.dirname(__file__)
    oxts_dir = os.path.join(base_dir, "../Data/oxts")
    output_dir = os.path.join(base_dir, "../Data/oxts_csv")

    os.makedirs(output_dir, exist_ok=True)
    output_csv = os.path.join(output_dir, "oxts.csv")

    prepare_kitti_sequence(oxts_dir, output_csv)

