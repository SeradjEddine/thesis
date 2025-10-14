#!/usr/bin/env python3
"""
vo_flow_stereo.py
Sparse optical-flow stereo visual-odometry preprocessor.

Produces a CSV of visual motion samples:
t, tx, ty, tz, qw, qx, qy, qz, n_inliers, mean_residual

Assumptions:
 - Input stereo images are rectified and synchronized.
 - You provide left/right image directories and a timestamps file with one timestamp (sec) per frame.
 - Camera intrinsics and baseline must be provided (either via KITTI calib or manually).
 - Uses OpenCV (cv2), numpy.

Usage example:
  python3 vo_flow_stereo.py \
    --left_dir data/image_02/data --right_dir data/image_03/data \
    --timestamps data/timestamps.txt \
    --out visual_samples.csv \
    --fx 721.5377 --fy 721.5377 --cx 609.5593 --cy 172.8540 --baseline 0.54 \
    --visual_fps 10 --max_corners 800

"""

import os
import argparse
import cv2
import numpy as np
import math
import csv
from tqdm import tqdm

# ----------------------------
# Utility math helpers
# ----------------------------
def se3_from_rotation_translation(R, t):
    """Return quaternion (w,x,y,z) for rotation R and translation t (not used for t)"""
    # Convert rotation matrix to quaternion (w,x,y,z)
    # Using method from https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    m = R
    trace = m[0,0] + m[1,1] + m[2,2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2,1] - m[1,2]) * s
        y = (m[0,2] - m[2,0]) * s
        z = (m[1,0] - m[0,1]) * s
    else:
        if (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
            s = 2.0 * math.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2])
            w = (m[2,1] - m[1,2]) / s
            x = 0.25 * s
            y = (m[0,1] + m[1,0]) / s
            z = (m[0,2] + m[2,0]) / s
        elif m[1,1] > m[2,2]:
            s = 2.0 * math.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2])
            w = (m[0,2] - m[2,0]) / s
            x = (m[0,1] + m[1,0]) / s
            y = 0.25 * s
            z = (m[1,2] + m[2,1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1])
            w = (m[1,0] - m[0,1]) / s
            x = (m[0,2] + m[2,0]) / s
            y = (m[1,2] + m[2,1]) / s
            z = 0.25 * s
    return np.array([w, x, y, z], dtype=float)

def rigid_transform_3D(A, B):
    """
    Umeyama / SVD-based rigid transform from A->B for corresponding 3xN matrices (columns are points).
    Returns R (3x3), t (3x1), mean_residual, inlier_indices (here all points assumed inliers).
    """
    assert A.shape == B.shape
    n = A.shape[1]
    if n < 3:
        return None, None, float('inf')
    centroid_A = np.mean(A, axis=1, keepdims=True)
    centroid_B = np.mean(B, axis=1, keepdims=True)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA @ BB.T
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        # reflection fix
        Vt[2,:] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    # residuals
    A_trans = R @ A + t
    res = np.linalg.norm(A_trans - B, axis=0)
    mean_res = float(np.mean(res))
    return R, t.flatten(), mean_res

# ----------------------------
# Stereo and optical flow helpers
# ----------------------------
def compute_disparity_sparse(left, right, pts, half_window=5, max_disp=256):
    """
    For each point in pts (Nx2), search along same row in 'right' image using template matching
    returns disparities (u_left - u_right), or np.nan if not found/invalid.
    """
    h, w = left.shape
    disparities = np.full((pts.shape[0],), np.nan, dtype=float)
    tpl_w = half_window
    tpl_h = half_window
    for i, (u, v) in enumerate(pts):
        u_i = int(round(u)); v_i = int(round(v))
        if v_i - tpl_h < 0 or v_i + tpl_h >= h or u_i - tpl_w < 0 or u_i + tpl_w >= w:
            continue
        tpl = left[v_i - tpl_h:v_i + tpl_h + 1, u_i - tpl_w:u_i + tpl_w + 1]
        # search area in right: limit disparity range
        min_u = max(u_i - max_disp, tpl_w)
        max_u = min(u_i + tpl_w, w - tpl_w -1)
        if min_u >= max_u: 
            continue
        search = right[v_i - tpl_h:v_i + tpl_h + 1, min_u - tpl_w:max_u + tpl_w + 1]
        # use matchTemplate with TM_SQDIFF_NORMED
        res = cv2.matchTemplate(search, tpl, method=cv2.TM_SQDIFF_NORMED)
        min_val, _, min_loc, _ = cv2.minMaxLoc(res)
        # match x in search coordinates:
        rel_x = min_loc[0]
        found_u = min_u + rel_x
        disp = float(u_i - found_u)
        if disp <= 0.0:
            continue
        disparities[i] = disp
    return disparities

def pts_to_3d(pts_uv, disparities, fx, fy, cx, cy, baseline):
    """
    pts_uv: Nx2 array of pixel coords (u,v) in left image
    disparities: N-array of disparity (u_left - u_right)
    returns: 3xM array of 3D points (X,Y,Z) for valid disparities
    """
    valid = np.isfinite(disparities) & (disparities > 0.0)
    if np.sum(valid) == 0:
        return np.empty((3,0)), np.array([], dtype=int)
    u = pts_uv[valid,0].astype(float)
    v = pts_uv[valid,1].astype(float)
    d = disparities[valid].astype(float)
    Z = (fx * baseline) / d
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    pts3 = np.vstack((X, Y, Z))
    idxs = np.nonzero(valid)[0]
    return pts3, idxs

# ----------------------------
# Main processing
# ----------------------------
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--left_dir", required=True, help="Left images directory (ordered filenames)")
    p.add_argument("--right_dir", required=True, help="Right images directory (ordered filenames)")
    p.add_argument("--timestamps", required=True, help="Text file with one timestamp (seconds) per frame, same order as images")
    p.add_argument("--out", default="visual_samples.csv", help="Output CSV path")
    p.add_argument("--fx", type=float, required=True)
    p.add_argument("--fy", type=float, required=True)
    p.add_argument("--cx", type=float, required=True)
    p.add_argument("--cy", type=float, required=True)
    p.add_argument("--baseline", type=float, required=True, help="Stereo baseline in meters")
    p.add_argument("--visual_fps", type=float, default=10.0, help="Processing frequency (Hz) to generate outputs")
    p.add_argument("--max_corners", type=int, default=800)
    p.add_argument("--quality_level", type=float, default=0.01)
    p.add_argument("--min_distance", type=float, default=7.0)
    p.add_argument("--lk_win", type=int, default=21)
    p.add_argument("--lk_max_level", type=int, default=3)
    p.add_argument("--half_window_disp", type=int, default=5)
    p.add_argument("--min_inliers", type=int, default=20)
    p.add_argument("--downscale", type=float, default=1.0, help="Downscale factor for images (1.0 = full size)")
    return p.parse_args()

def load_filenames(left_dir, right_dir, timestamps_file):
    left_files = sorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if f.endswith(('.png','.jpg'))])
    right_files = sorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if f.endswith(('.png','.jpg'))])
    with open(timestamps_file,'r') as fh:
        timestamps = [float(line.strip()) for line in fh if line.strip()]
    if not (len(left_files) == len(right_files) == len(timestamps)):
        raise RuntimeError("Mismatch lengths: left %d right %d timestamps %d" % (len(left_files), len(right_files), len(timestamps)))
    return left_files, right_files, timestamps

def main():
    args = parse_args()

    # camera params
    fx, fy, cx, cy, baseline = args.fx, args.fy, args.cx, args.cy, args.baseline

    # load file lists
    left_files, right_files, timestamps = load_filenames(args.left_dir, args.right_dir, args.timestamps)
    n_frames = len(left_files)
    print("Found %d frames" % n_frames)

    # compute skip step from dataset_fps (assume dataset 100 Hz) -> target visual_fps
    dataset_fps = 100.0
    step = max(1, int(round(dataset_fps / args.visual_fps)))
    print("Processing every %d frame(s) -> approx %.2f Hz" % (step, dataset_fps/step))

    # LK params
    lk_params = dict(winSize=(args.lk_win, args.lk_win),
                     maxLevel=args.lk_max_level,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    # feature detection params
    feature_params = dict(maxCorners=args.max_corners,
                          qualityLevel=args.quality_level,
                          minDistance=args.min_distance,
                          blockSize=7)

    # prepare output CSV
    out_f = open(args.out, 'w', newline='')
    writer = csv.writer(out_f)
    writer.writerow(['t','tx','ty','tz','qw','qx','qy','qz','n_inliers','mean_residual'])

    # Read first frame
    prev_idx = 0
    prev_img_l = cv2.imread(left_files[prev_idx], cv2.IMREAD_GRAYSCALE)
    prev_img_r = cv2.imread(right_files[prev_idx], cv2.IMREAD_GRAYSCALE)
    if prev_img_l is None or prev_img_r is None:
        raise RuntimeError("Failed reading first images")
    if args.downscale != 1.0:
        prev_img_l = cv2.resize(prev_img_l, (0,0), fx=1.0/args.downscale, fy=1.0/args.downscale, interpolation=cv2.INTER_AREA)
        prev_img_r = cv2.resize(prev_img_r, (0,0), fx=1.0/args.downscale, fy=1.0/args.downscale, interpolation=cv2.INTER_AREA)
        # adjust intrinsics
        fx /= args.downscale; fy /= args.downscale; cx /= args.downscale; cy /= args.downscale; baseline /= args.downscale

    # detect initial features in prev left
    prev_pts = cv2.goodFeaturesToTrack(prev_img_l, mask=None, **feature_params)
    if prev_pts is None:
        prev_pts = np.empty((0,1,2), dtype=np.float32)
    prev_pts = prev_pts.reshape(-1,2) if prev_pts.size else np.empty((0,2),dtype=float)

    # iterate frames
    for k in tqdm(range(step, n_frames, step)):
        # load current pair
        cur_img_l = cv2.imread(left_files[k], cv2.IMREAD_GRAYSCALE)
        cur_img_r = cv2.imread(right_files[k], cv2.IMREAD_GRAYSCALE)
        if cur_img_l is None or cur_img_r is None:
            print("Warning: failed reading frame", k)
            continue
        if args.downscale != 1.0:
            cur_img_l = cv2.resize(cur_img_l, (0,0), fx=1.0/args.downscale, fy=1.0/args.downscale, interpolation=cv2.INTER_AREA)
            cur_img_r = cv2.resize(cur_img_r, (0,0), fx=1.0/args.downscale, fy=1.0/args.downscale, interpolation=cv2.INTER_AREA)

        # Track prev_pts -> cur_pts using sparse LK
        if prev_pts.shape[0] == 0:
            prev_pts = cv2.goodFeaturesToTrack(prev_img_l, mask=None, **feature_params)
            prev_pts = prev_pts.reshape(-1,2) if prev_pts is not None else np.empty((0,2))
        prev_pts_float = prev_pts.astype(np.float32)
        cur_pts, st, err = cv2.calcOpticalFlowPyrLK(prev_img_l, cur_img_l, prev_pts_float, None, **lk_params)
        if cur_pts is None:
            # nothing tracked; re-detect and continue
            prev_img_l = cur_img_l.copy(); prev_img_r = cur_img_r.copy()
            prev_pts = cv2.goodFeaturesToTrack(prev_img_l, mask=None, **feature_params)
            prev_pts = prev_pts.reshape(-1,2) if prev_pts is not None else np.empty((0,2))
            continue
        st = st.reshape(-1)
        cur_pts = cur_pts.reshape(-1,2)
        tracked_prev = prev_pts[st==1]
        tracked_cur = cur_pts[st==1]
        if tracked_prev.shape[0] < 10:
            # too few tracks; redetect features
            prev_img_l = cur_img_l.copy(); prev_img_r = cur_img_r.copy()
            prev_pts = cv2.goodFeaturesToTrack(prev_img_l, mask=None, **feature_params)
            prev_pts = prev_pts.reshape(-1,2) if prev_pts is not None else np.empty((0,2))
            continue

        # For tracked_prev, compute stereo disparity in prev frame, and optionally in current frame
        disparities_prev = compute_disparity_sparse(prev_img_l, prev_img_r, tracked_prev, half_window=args.half_window_disp)
        disparities_cur  = compute_disparity_sparse(cur_img_l, cur_img_r, tracked_cur, half_window=args.half_window_disp)

        # Build 3D points
        A3, idxsA = pts_to_3d(tracked_prev, disparities_prev, fx, fy, cx, cy, baseline)
        B3, idxsB = pts_to_3d(tracked_cur, disparities_cur, fx, fy, cx, cy, baseline)

        # Need correspondences: keep only pts that have valid disparities in both frames and correspond by index
        # idxsA/B are indices into tracked_prev/cur arrays
        setA = set(idxsA.tolist())
        setB = set(idxsB.tolist())
        common = sorted(list(setA & setB))
        if len(common) < args.min_inliers:
            # not enough valid stereo-depth correspondences
            # prepare next iteration: detect new features if necessary
            prev_img_l = cur_img_l.copy(); prev_img_r = cur_img_r.copy()
            prev_pts = cv2.goodFeaturesToTrack(prev_img_l, mask=None, **feature_params)
            prev_pts = prev_pts.reshape(-1,2) if prev_pts is not None else np.empty((0,2))
            continue

        PA = np.zeros((3, len(common)))
        PB = np.zeros((3, len(common)))
        for i, idx in enumerate(common):
            # find positions in A3/B3 arrays
            ia = np.where(idxsA == idx)[0][0]
            ib = np.where(idxsB == idx)[0][0]
            PA[:,i] = A3[:,ia]
            PB[:,i] = B3[:,ib]

        # robustly estimate R,t using simple RANSAC on subsets
        best_R, best_t, best_res, best_inliers = None, None, float('inf'), None
        # RANSAC iterations: sample minimal sets of 3 points repeatedly
        n_corr = PA.shape[1]
        n_iters = min(200, max(50, int(500 * 1.0/(n_corr/100.0+1))))  # heuristic
        for it in range(n_iters):
            # choose 3 random indices
            s = np.random.choice(n_corr, 3, replace=False)
            R_est, t_est, _ = rigid_transform_3D(PA[:,s], PB[:,s])
            if R_est is None: continue
            PA_t = R_est @ PA + t_est.reshape(3,1)
            residuals = np.linalg.norm(PA_t - PB, axis=0)
            inliers = np.where(residuals < 0.1)[0]  # 10cm threshold, tunable
            if inliers.size < args.min_inliers: continue
            mean_res = float(np.mean(residuals[inliers]))
            # prefer more inliers then lower residual
            score = (len(inliers), -mean_res)
            # pick best by number of inliers then residual
            if best_inliers is None or (len(inliers) > len(best_inliers)) or (len(inliers) == len(best_inliers) and mean_res < best_res):
                best_R = R_est; best_t = t_est; best_res = mean_res; best_inliers = inliers

        if best_R is None or best_inliers is None or len(best_inliers) < args.min_inliers:
            # failed robust fit
            prev_img_l = cur_img_l.copy(); prev_img_r = cur_img_r.copy()
            prev_pts = cv2.goodFeaturesToTrack(prev_img_l, mask=None, **feature_params)
            prev_pts = prev_pts.reshape(-1,2) if prev_pts is not None else np.empty((0,2))
            continue

        # Compose quaternion from R and translation vector t
        q = se3_from_rotation_translation(best_R, best_t)
        tvec = best_t  # translation in left camera frame (meters)

        # Timestamp for this measurement: use timestamps[k]
        t_stamp = timestamps[k]

        # write CSV row: t, tx,ty,tz, qw,qx,qy,qz, n_inliers, mean_residual
        writer.writerow([t_stamp, float(tvec[0]), float(tvec[1]), float(tvec[2]),
                         float(q[0]), float(q[1]), float(q[2]), float(q[3]),
                         int(len(best_inliers)), float(best_res)])
        out_f.flush()

        # prepare next iteration
        prev_img_l = cur_img_l.copy(); prev_img_r = cur_img_r.copy()
        # track keep-inlier pts to maintain continuity
        # build prev_pts from tracked_cur[common[best_inliers]] to continue
        try:
            inlier_indices = [common[i] for i in best_inliers]
            prev_pts = tracked_cur[inlier_indices, :].reshape(-1,2)
        except Exception:
            prev_pts = cv2.goodFeaturesToTrack(prev_img_l, mask=None, **feature_params)
            prev_pts = prev_pts.reshape(-1,2) if prev_pts is not None else np.empty((0,2))

    out_f.close()
    print("Done. Visual CSV written to", args.out)

if __name__ == "__main__":
    main()
