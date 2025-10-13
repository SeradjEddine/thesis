#!/usr/bin/env python3
"""
vo_flow_stereo.py

Sparse optical-flow + stereo-depth visual odometry preprocessor.

Input folder layout (cwd):
./image_00/data/      -> left images (png)
./image_01/data/      -> right images (png)
./image_00/timestamps.txt
./image_01/timestamps.txt

Outputs:
 - visual_samples.csv: timestamp (s), tx,ty,tz, qw,qx,qy,qz, n_inliers, mean_residual
 - optional debug images saved to debug/ (feature tracks & inliers)

Requires: opencv-python, numpy
pip install opencv-python numpy
"""

import os
import cv2
import numpy as np
import argparse
import math
import csv
import random

# -----------------------
# Parameters (tune these)
# -----------------------
MAX_CORNERS = 800            # number of features to detect
QUALITY_LEVEL = 0.01
MIN_DISTANCE = 7
LK_WIN = (21,21)
LK_MAX_LEVEL = 3
LK_CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)

DOWNSAMPLE = 1               # integer factor (1 = original, 2 = half size)
VISUAL_FPS = 20              # target visual processing frequency (Hz). Use 100 to process every frame
RANSAC_ITERS = 150
RANSAC_THRESH_M = 0.20       # in meters (inlier distance threshold)
MIN_INLIERS = 30

# StereoBM parameters (for disparity)
STEREO_NUM_DISPARITIES = 64  # must be divisible by 16
STEREO_BLOCK_SIZE = 15       # odd

# camera intrinsics - override with your KITTI rectified intrinsics
# If you have a calibration file, replace these with exact values.
FX = None   # placeholder -> will be read from calib file if provided
FY = None
CX = None
CY = None
BASELINE = None  # meters, distance between left & right cameras

# -----------------------
# Helpers
# -----------------------
def read_image_list(img_dir):
    """Return sorted list of file paths inside img_dir/data/"""
    data_dir = os.path.join(img_dir, "data")
    files = sorted([os.path.join(data_dir, f) for f in os.listdir(data_dir) if f.endswith(".png") or f.endswith(".jpg")])
    return files

from datetime import datetime

from datetime import datetime

from datetime import datetime

from datetime import datetime

def read_timestamps(ts_file):
    """
    Read KITTI-style timestamps and convert to relative seconds (float).

    Handles subsecond precision up to nanoseconds by truncating/padding
    to microseconds (6 digits), since datetime.strptime supports only that.
    """
    fmt = "%Y-%m-%d %H:%M:%S.%f"
    with open(ts_file, "r") as f:
        lines = [l.strip() for l in f if l.strip()]

    times = []
    t0 = None
    for line in lines:
        try:
            date_part, time_part = line.split()
            if '.' in time_part:
                sec, frac = time_part.split('.')
                # truncate or pad fractional part to 6 digits (microseconds)
                frac = (frac + "000000")[:6]
                time_part = f"{sec}.{frac}"
            else:
                time_part = f"{time_part}.000000"
            line_fmt = f"{date_part} {time_part}"
            t = datetime.strptime(line_fmt, fmt)
        except Exception as e:
            raise ValueError(f"Malformed timestamp line: {line}") from e

        if t0 is None:
            t0 = t
        times.append((t - t0).total_seconds())

    return times




def load_camera_params_from_kitti(path_calib_txt=None):
    """
    Optional helper: if you have calibration txt with fx,fy,cx,cy and baseline info,
    parse it here. For now, if not provided, we will attempt to derive fx,cx from image width heuristics.
    """
    # Minimal implementation: returns None and we fall back to reasonable defaults
    return None

def disparity_sgbm(left_gray, right_gray):
    """Compute disparity with StereoBM or StereoSGBM. Return float32 disparity in pixels (valid: >0)"""
    # Use StereoBM for speed
    # Convert to 8-bit if needed
    left = left_gray
    right = right_gray
    if left.dtype != np.uint8:
        left = cv2.normalize(left, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        right = cv2.normalize(right, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    sbm = cv2.StereoBM_create(numDisparities=STEREO_NUM_DISPARITIES, blockSize=STEREO_BLOCK_SIZE)
    disp = sbm.compute(left, right).astype(np.float32) / 16.0  # OpenCV returns fixed-point
    return disp

def disparity_at_point(disp, pt):
    """Safe read disparity with bilinear interpolation; return None if outside or invalid"""
    x, y = pt
    h, w = disp.shape
    if x < 0 or y < 0 or x >= w-1 or y >= h-1:
        return None
    # bilinear
    x0 = int(np.floor(x)); y0 = int(np.floor(y))
    dx = x - x0; dy = y - y0
    v00 = disp[y0, x0]; v01 = disp[y0, x0+1]; v10 = disp[y0+1, x0]; v11 = disp[y0+1, x0+1]
    val = (1-dx)*(1-dy)*v00 + dx*(1-dy)*v01 + (1-dx)*dy*v10 + dx*dy*v11
    if val <= 0.1:  # invalid
        return None
    return float(val)

def triangulate_point(u, v, disp, fx, fy, cx, cy, baseline):
    """Given pixel (u,v) in left image and disparity, return 3D point in camera coords (x,y,z)"""
    if disp is None or disp <= 0.1: 
        return None
    z = (fx * baseline) / disp
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z], dtype=np.float64)

def svd_rigid_transform(A, B):
    """
    Compute rigid transform (R, t) that best maps A->B (A,B: Nx3)
    using SVD (Umeyama without scale)
    Returns R (3x3), t (3,)
    """
    assert A.shape == B.shape and A.shape[1] == 3
    N = A.shape[0]
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    Am = A - centroid_A
    Bm = B - centroid_B
    H = Am.T.dot(Bm)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T.dot(U.T)
    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = Vt.T.dot(U.T)
    t = centroid_B - R.dot(centroid_A)
    return R, t

def rotmat_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion (w,x,y,z)"""
    # Using standard method
    m = R
    tr = m[0,0] + m[1,1] + m[2,2]
    if tr > 0:
        S = math.sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (m[2,1] - m[1,2]) / S
        qy = (m[0,2] - m[2,0]) / S
        qz = (m[1,0] - m[0,1]) / S
    elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
        S = math.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
        qw = (m[2,1] - m[1,2]) / S
        qx = 0.25 * S
        qy = (m[0,1] + m[1,0]) / S
        qz = (m[0,2] + m[2,0]) / S
    elif m[1,1] > m[2,2]:
        S = math.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
        qw = (m[0,2] - m[2,0]) / S
        qx = (m[0,1] + m[1,0]) / S
        qy = 0.25 * S
        qz = (m[1,2] + m[2,1]) / S
    else:
        S = math.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
        qw = (m[1,0] - m[0,1]) / S
        qx = (m[0,2] + m[2,0]) / S
        qy = (m[1,2] + m[2,1]) / S
        qz = 0.25 * S
    return np.array([qw, qx, qy, qz], dtype=np.float64)

# -----------------------
# Main processing
# -----------------------
def process_stereo_folder(left_folder, right_folder, out_csv="visual_samples.csv",
                          debug_dir="debug", visual_fps=VISUAL_FPS, downsample=DOWNSAMPLE,
                          fx=None, fy=None, cx=None, cy=None, baseline=None):
    # Read lists
    left_list = read_image_list(left_folder)
    right_list = read_image_list(right_folder)
    ts_left = read_timestamps(os.path.join(left_folder, "timestamps.txt"))
    ts_right = read_timestamps(os.path.join(right_folder, "timestamps.txt"))

    # Basic sanity
    assert len(left_list) == len(right_list) == len(ts_left) == len(ts_right), "Left/right counts mismatch"

    N = len(left_list)
    print("Found %d frames" % N)

    # If camera params not passed, try heuristics
    img0 = cv2.imread(left_list[0], cv2.IMREAD_GRAYSCALE)
    h, w = img0.shape[:2]
    if fx is None:
        fx = 0.7 * w   # heuristic focal length (pixels)
        fy = fx
    if cx is None:
        cx = w / 2.0
        cy = h / 2.0
    if baseline is None:
        baseline = 0.54  # KITTI stereo baseline ~0.54 m default; override if you have exact

    print("Camera params: fx=%.2f cx=%.2f cy=%.2f baseline=%.3f" % (fx, cx, cy, baseline))

    # Prepare stereo matcher (computed per-frame)
    if not os.path.exists(debug_dir):
        os.makedirs(debug_dir, exist_ok=True)

    # Skip factor based on dataset rate (assumed dataset rate = 100 Hz)
    dataset_rate = 100.0
    skip = max(1, int(round(dataset_rate / visual_fps)))

    samples = []
    max_corners = MAX_CORNERS

    # Preload images? We'll stream to avoid memory blow
    prev_left = None
    prev_right = None
    prev_gray = None
    prev_disp = None
    prev_kps = None
    prev_xyz = None

    idx = 0
    processed = 0
    for i in range(0, N, skip):
        # load current frame
        left_path = left_list[i]; right_path = right_list[i]
        left = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
        tstamp = ts_left[i]

        if downsample != 1:
            left = cv2.resize(left, (left.shape[1]//downsample, left.shape[0]//downsample))
            right = cv2.resize(right, (right.shape[1]//downsample, right.shape[0]//downsample))
            # adjust intrinsics
            fx_ = fx / downsample; fy_ = fy / downsample; cx_ = cx / downsample; cy_ = cy / downsample
            baseline_ = baseline / downsample  # baseline should remain in meters (do not scale); so keep baseline same -> adjust fx instead
            fx_cur, fy_cur, cx_cur, cy_cur, baseline_cur = fx_, fy_, cx_, cy_, baseline
        else:
            fx_cur, fy_cur, cx_cur, cy_cur, baseline_cur = fx, fy, cx, cy, baseline

        # compute disparity map for left frame
        disp = disparity_sgbm(left, right)  # float disparity in px

        if prev_left is None:
            # detect initial features in left
            corners = cv2.goodFeaturesToTrack(left, maxCorners=max_corners, qualityLevel=QUALITY_LEVEL,
                                              minDistance=MIN_DISTANCE)
            if corners is None:
                print("No corners found in first frame")
                return
            pts = corners.reshape(-1,2)
            # triangulate each corner
            xyz = []
            for (u,v) in pts:
                d = disparity_at_point(disp, (u,v))
                p3 = triangulate_point(u, v, d, fx_cur, fy_cur, cx_cur, cy_cur, baseline_cur)
                xyz.append(p3)
            xyz = np.array([p for p in xyz if p is not None], dtype=np.float64)
            prev_left = left; prev_right = right; prev_gray = left; prev_disp = disp
            prev_kps = pts[:len(xyz)]
            prev_xyz = xyz
            prev_t = tstamp
            continue

        # Track prev_kps from prev_left to current left using LK
        p0 = prev_kps.reshape(-1,1,2).astype(np.float32)
        p1, st, err = cv2.calcOpticalFlowPyrLK(prev_left, left, p0, None, winSize=LK_WIN,
                                               maxLevel=LK_MAX_LEVEL, criteria=LK_CRIT)
        p1 = p1.reshape(-1,2)
        st = st.reshape(-1)
        good0 = prev_kps[st==1]
        good1 = p1[st==1]
        xyz0 = prev_xyz[st==1]

        if len(good0) < MIN_INLIERS:
            # Not enough tracked points; re-detect features and continue
            corners = cv2.goodFeaturesToTrack(prev_left, maxCorners=max_corners, qualityLevel=QUALITY_LEVEL,
                                              minDistance=MIN_DISTANCE)
            if corners is None:
                prev_left = left; prev_right = right; prev_disp = disp; prev_t = tstamp
                continue
            pts = corners.reshape(-1,2)
            xyz = []
            for (u,v) in pts:
                d = disparity_at_point(disp, (u,v))
                p3 = triangulate_point(u, v, d, fx_cur, fy_cur, cx_cur, cy_cur, baseline_cur)
                xyz.append(p3)
            xyz = np.array([p for p in xyz if p is not None], dtype=np.float64)
            prev_kps = pts[:len(xyz)]
            prev_xyz = xyz
            prev_left = left; prev_right = right; prev_disp = disp; prev_t = tstamp
            continue

        # For current matched points, obtain 3D positions at time t (use disparity of current frame)
        xyz1_list = []
        good1_list = []
        good0_list = []
        xyz0_list = []
        for (pt0, pt1, p3_0) in zip(good0, good1, xyz0):
            u1, v1 = pt1
            d1 = disparity_at_point(disp, (u1, v1))
            p3_1 = triangulate_point(u1, v1, d1, fx_cur, fy_cur, cx_cur, cy_cur, baseline_cur)
            if p3_1 is None:
                continue
            xyz0_list.append(p3_0)
            xyz1_list.append(p3_1)
            good0_list.append(pt0)
            good1_list.append(pt1)

        if len(xyz0_list) < MIN_INLIERS:
            # not enough valid 3D points
            prev_left = left; prev_right = right; prev_disp = disp; prev_t = tstamp
            prev_kps = good1[:len(good1)] if len(good1)>0 else None
            prev_xyz = np.array(xyz1_list) if len(xyz1_list)>0 else None
            continue

        A = np.vstack(xyz0_list)
        B = np.vstack(xyz1_list)

        # RANSAC on 3D-3D transform
        best_inliers = []
        best_R = None; best_t = None
        for it in range(RANSAC_ITERS):
            # draw 3 random distinct indices
            if A.shape[0] < 3: break
            ids = random.sample(range(A.shape[0]), 3)
            try:
                R_cand, t_cand = svd_rigid_transform(A[ids], B[ids])
            except Exception:
                continue
            # compute residuals
            B_pred = (R_cand.dot(A.T)).T + t_cand
            residuals = np.linalg.norm(B - B_pred, axis=1)
            inliers = np.where(residuals < RANSAC_THRESH_M)[0]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_R = R_cand; best_t = t_cand
                if len(best_inliers) > 0.9 * A.shape[0]:
                    break

        if best_R is None or len(best_inliers) < MIN_INLIERS:
            # skip update
            prev_left = left; prev_right = right; prev_disp = disp; prev_t = tstamp
            prev_kps = np.array(good1_list) if len(good1_list)>0 else None
            prev_xyz = np.array(xyz1_list) if len(xyz1_list)>0 else None
            continue

        inlier_A = A[best_inliers]
        inlier_B = B[best_inliers]
        # refine on all inliers
        R_ref, t_ref = svd_rigid_transform(inlier_A, inlier_B)
        # residuals
        B_pred = (R_ref.dot(inlier_A.T)).T + t_ref
        residuals = np.linalg.norm(inlier_B - B_pred, axis=1)
        mean_res = float(np.mean(residuals))

        # Convert rotation to quaternion (camera frame)
        q = rotmat_to_quat(R_ref)  # w,x,y,z

        # Store sample for timestamp = current frame t (we treat delta from prev to current)
        sample = {
            't': float(tstamp),
            'tx': float(t_ref[0]),
            'ty': float(t_ref[1]),
            'tz': float(t_ref[2]),
            'qw': float(q[0]), 'qx': float(q[1]), 'qy': float(q[2]), 'qz': float(q[3]),
            'n_inliers': int(len(best_inliers)),
            'mean_residual': float(mean_res)
        }
        samples.append(sample)
        processed += 1

        # Debug overlay and save
        # draw matches for inliers
        vis = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
        for idx_in in best_inliers:
            u0, v0 = A[idx_in][:2]; u1, v1 = B[idx_in][:2]
            cv2.circle(vis, (int(u1), int(v1)), 2, (0,255,0), -1)
            cv2.line(vis, (int(u1), int(v1)), (int(u0), int(v0)), (0,128,255), 1)
        debug_name = os.path.join(debug_dir, "vis_%06d.png" % processed)
        cv2.imwrite(debug_name, vis)

        # shift: set current as prev for next iter
        prev_left = left; prev_right = right; prev_disp = disp; prev_t = tstamp
        prev_kps = np.array(good1_list) if len(good1_list)>0 else None
        prev_xyz = np.array(xyz1_list) if len(xyz1_list)>0 else None

    # Write CSV
    print("Processed %d visual samples" % len(samples))
    with open(out_csv, "w", newline='') as csvf:
        writer = csv.writer(csvf)
        writer.writerow(["t","tx","ty","tz","qw","qx","qy","qz","n_inliers","mean_residual"])
        for s in samples:
            writer.writerow([s['t'], s['tx'], s['ty'], s['tz'],
                             s['qw'], s['qx'], s['qy'], s['qz'],
                             s['n_inliers'], s['mean_residual']])
    print("Wrote", out_csv)
    return samples

# -----------------------
# Command-line entry
# -----------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sparse stereo optical-flow VO preprocessor")
    parser.add_argument("--left", default="image_00", help="left image folder (contains data/ and timestamps.txt)")
    parser.add_argument("--right", default="image_01", help="right image folder")
    parser.add_argument("--out", default="visual_samples.csv", help="output CSV path")
    parser.add_argument("--fps", type=float, default=VISUAL_FPS, help="visual processing fps (target)")
    parser.add_argument("--down", type=int, default=DOWNSAMPLE, help="downsample factor")
    parser.add_argument("--debug", action="store_true", help="save debug images")
    args = parser.parse_args()

    left_folder = args.left
    right_folder = args.right
    if not os.path.isdir(left_folder) or not os.path.isdir(right_folder):
        print("Error: folders not found:", left_folder, right_folder)
        exit(1)

    samples = process_stereo_folder(left_folder, right_folder,
                                    out_csv=args.out, debug_dir="debug" if args.debug else "debug_skip",
                                    visual_fps=args.fps, downsample=args.down)
