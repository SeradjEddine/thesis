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
RANSAC_THRESH_M = 0.4       # in meters (inlier distance threshold)
MIN_INLIERS = 6

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

def process_stereo_folder_debug(left_folder, right_folder, out_csv="visual_samples.csv",
                                debug_dir="debug", visual_fps=VISUAL_FPS, downsample=DOWNSAMPLE,
                                fx=None, fy=None, cx=None, cy=None, baseline=None):
    left_list = read_image_list(left_folder)
    right_list = read_image_list(right_folder)
    ts_left = read_timestamps(os.path.join(left_folder, "timestamps.txt"))
    ts_right = read_timestamps(os.path.join(right_folder, "timestamps.txt"))

    assert len(left_list) == len(right_list) == len(ts_left) == len(ts_right), "Left/right counts mismatch"
    N = len(left_list)
    print(f"[DEBUG] Found {N} frames")

    img0 = cv2.imread(left_list[0], cv2.IMREAD_GRAYSCALE)
    h, w = img0.shape[:2]
    fx = fx if fx is not None else 0.7 * w
    fy = fy if fy is not None else fx
    cx = cx if cx is not None else w / 2.0
    cy = cy if cy is not None else h / 2.0
    baseline = baseline if baseline is not None else 0.54
    print(f"[DEBUG] Camera params: fx={fx:.2f} cx={cx:.2f} cy={cy:.2f} baseline={baseline:.3f}")

    if not os.path.exists(debug_dir):
        os.makedirs(debug_dir, exist_ok=True)

    skip = max(1, int(round(100.0 / visual_fps)))
    samples = []

    prev_left = prev_right = prev_disp = prev_kps = prev_xyz = prev_t = None

    for i in range(0, N, skip):
        left = cv2.imread(left_list[i], cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(right_list[i], cv2.IMREAD_GRAYSCALE)
        tstamp = ts_left[i]

        if downsample != 1:
            left = cv2.resize(left, (left.shape[1]//downsample, left.shape[0]//downsample))
            right = cv2.resize(right, (right.shape[1]//downsample, right.shape[0]//downsample))
            fx_cur, fy_cur, cx_cur, cy_cur = fx/downsample, fy/downsample, cx/downsample, cy/downsample
        else:
            fx_cur, fy_cur, cx_cur, cy_cur = fx, fy, cx, cy

        disp = disparity_sgbm(left, right)

        if prev_left is None:
            # Detect initial corners and triangulate
            corners = cv2.goodFeaturesToTrack(left, maxCorners=MAX_CORNERS,
                                              qualityLevel=QUALITY_LEVEL, minDistance=MIN_DISTANCE)
            if corners is None:
                print("[DEBUG] No corners found in first frame")
                return
            pts = corners.reshape(-1, 2)
            xyz = []
            for u, v in pts:
                d = disparity_at_point(disp, (u, v))
                p3 = triangulate_point(u, v, d, fx_cur, fy_cur, cx_cur, cy_cur, baseline)
                if p3 is not None:
                    xyz.append(p3)
            xyz = np.array(xyz, dtype=np.float64)
            print(f"[DEBUG] First frame: {len(xyz)} 3D points")

            prev_left, prev_right, prev_disp, prev_kps, prev_xyz, prev_t = left, right, disp, pts[:len(xyz)], xyz, tstamp
            continue

        # Track points with LK
        p0 = prev_kps.reshape(-1, 1, 2).astype(np.float32)
        p1, st, _ = cv2.calcOpticalFlowPyrLK(prev_left, left, p0, None, winSize=LK_WIN,
                                             maxLevel=LK_MAX_LEVEL, criteria=LK_CRIT)
        st = st.reshape(-1)
        good0, good1 = prev_kps[st==1], p1.reshape(-1,2)[st==1]
        xyz0 = prev_xyz[st==1]

        print(f"[DEBUG] Frame {i}: tracked {len(prev_kps)} points -> {len(good0)} valid")

        if len(good0) < MIN_INLIERS:
            # Re-detect if not enough points
            corners = cv2.goodFeaturesToTrack(left, maxCorners=MAX_CORNERS,
                                              qualityLevel=QUALITY_LEVEL, minDistance=MIN_DISTANCE)
            if corners is None:
                prev_left, prev_right, prev_disp, prev_kps, prev_xyz, prev_t = left, right, disp, prev_kps, prev_xyz, tstamp
                continue
            pts = corners.reshape(-1,2)
            xyz = []
            for u,v in pts:
                d = disparity_at_point(disp, (u,v))
                p3 = triangulate_point(u,v,d, fx_cur, fy_cur, cx_cur, cy_cur, baseline)
                if p3 is not None:
                    xyz.append(p3)
            xyz = np.array(xyz, dtype=np.float64)
            prev_kps, prev_xyz = pts[:len(xyz)], xyz
            prev_left, prev_right, prev_disp, prev_t = left, right, disp, tstamp
            continue

        # Triangulate matched points
        xyz0_list, xyz1_list, good0_list, good1_list = [], [], [], []
        for pt0, pt1, p3_0 in zip(good0, good1, xyz0):
            d1 = disparity_at_point(disp, pt1)
            p3_1 = triangulate_point(pt1[0], pt1[1], d1, fx_cur, fy_cur, cx_cur, cy_cur, baseline)
            if p3_1 is not None:
                xyz0_list.append(p3_0)
                xyz1_list.append(p3_1)
                good0_list.append(pt0)
                good1_list.append(pt1)

        print(f"[DEBUG] Frame {i}: {len(xyz0_list)} valid 3D points after triangulation")

        if len(xyz0_list) < MIN_INLIERS:
            prev_left, prev_right, prev_disp, prev_kps, prev_xyz, prev_t = left, right, disp, np.array(good1_list), np.array(xyz1_list), tstamp
            continue

        A = np.vstack(xyz0_list)
        B = np.vstack(xyz1_list)

        # RANSAC
        best_inliers, best_R, best_t = [], None, None
        for it in range(RANSAC_ITERS):
            if A.shape[0] < 3: break
            ids = np.random.choice(A.shape[0], 3, replace=False)
            try:
                R_cand, t_cand = svd_rigid_transform(A[ids], B[ids])
            except Exception:
                continue
            residuals = np.linalg.norm(B - (R_cand.dot(A.T)).T - t_cand, axis=1)
            inliers = np.where(residuals < RANSAC_THRESH_M)[0]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_R, best_t = R_cand, t_cand
                if len(best_inliers) > 0.9 * A.shape[0]:
                    break

        print(f"[DEBUG] Frame {i}: {len(best_inliers)} RANSAC inliers")
        if best_R is None or len(best_inliers) < MIN_INLIERS:
            prev_left, prev_right, prev_disp, prev_kps, prev_xyz, prev_t = left, right, disp, np.array(good1_list), np.array(xyz1_list), tstamp
            continue

        inlier_A, inlier_B = A[best_inliers], B[best_inliers]
        R_ref, t_ref = svd_rigid_transform(inlier_A, inlier_B)
        q = rotmat_to_quat(R_ref)

        sample = {
            't': float(tstamp),
            'tx': float(t_ref[0]), 'ty': float(t_ref[1]), 'tz': float(t_ref[2]),
            'qw': float(q[0]), 'qx': float(q[1]), 'qy': float(q[2]), 'qz': float(q[3]),
            'n_inliers': int(len(best_inliers)),
            'mean_residual': float(np.mean(np.linalg.norm(inlier_B - (R_ref.dot(inlier_A.T)).T - t_ref, axis=1)))
        }
        samples.append(sample)

        # Debug overlay
        vis = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
        for idx_local in range(len(best_inliers)):
            u0, v0 = inlier_A[idx_local][:2]
            u1, v1 = inlier_B[idx_local][:2]
            cv2.circle(vis, (int(u1), int(v1)), 2, (0,255,0), -1)
            cv2.line(vis, (int(u0), int(v0)), (int(u1), int(v1)), (0,128,255), 1)
        cv2.imwrite(os.path.join(debug_dir, f"vis_{i:06d}.png"), vis)

        # Update previous frame
        prev_left, prev_right, prev_disp, prev_kps, prev_xyz, prev_t = left, right, disp, np.array(good1_list), np.array(xyz1_list), tstamp

    # Write CSV
    print(f"[DEBUG] Processed {len(samples)} visual samples, writing {out_csv}")
    with open(out_csv, "w", newline='') as csvf:
        writer = csv.writer(csvf)
        writer.writerow(["t","tx","ty","tz","qw","qx","qy","qz","n_inliers","mean_residual"])
        for s in samples:
            writer.writerow([s['t'], s['tx'], s['ty'], s['tz'],
                             s['qw'], s['qx'], s['qy'], s['qz'],
                             s['n_inliers'], s['mean_residual']])
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

    samples = process_stereo_folder_debug(left_folder, right_folder,
                                    out_csv=args.out, debug_dir="debug" if args.debug else "debug_skip",
                                    visual_fps=args.fps, downsample=args.down)
