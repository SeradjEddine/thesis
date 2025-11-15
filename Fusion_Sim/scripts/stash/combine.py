#!/usr/bin/env python3
"""
Robust combiner for results_metrics.csv files.

- Looks under ../Data/sim_results_csv/{city,residential,road}/oxts*/results_metrics.csv
- Attempts robust parsing (auto-detect delimiter, strip BOM, normalize column names)
- Groups rows by n_corrupt (normalized name) and writes combined CSVs into ../Data/combined_results/
- If n_corrupt cannot be found, attempts to recover from nearby *_meta.json or records row in combined_unknown_n_corrupt.csv
- Prints diagnostics for files that couldn't be parsed normally.

Run from scripts/:
    python3 combine_all_logs_robust.py
"""
from pathlib import Path
import os
import glob
import pandas as pd
import json
import re
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = SCRIPT_DIR.parent / "Data" / "sim_results_csv"
OUT_DIR = SCRIPT_DIR.parent / "Data" / "combined_results"
OUT_DIR.mkdir(parents=True, exist_ok=True)

CATEGORIES = ["city", "residential", "road"]

def normalize_colname(c: str) -> str:
    if c is None:
        return c
    # remove BOM
    c = c.replace("\ufeff", "").replace("\ufbff", "")
    # strip whitespace and lower
    c = c.strip().lower()
    # replace non-alnum with underscore
    c = re.sub(r"[^\w]+", "_", c)
    # collapse multiple underscores
    c = re.sub(r"_+", "_", c)
    # trim underscores
    c = c.strip("_")
    return c

def try_read_csv(path: Path):
    """Try reading CSV robustly; return (df, used_sep) or raise."""
    text = path.read_bytes()
    # try with pandas auto-detect
    try:
        df = pd.read_csv(path, sep=None, engine="python", encoding="utf-8-sig")
        return df, "auto"
    except Exception:
        # fall back to common separators
        for sep in [",", "\t", ";", "|"]:
            try:
                df = pd.read_csv(path, sep=sep, engine="python", encoding="utf-8-sig")
                return df, sep
            except Exception:
                continue
    # last resort: read with whitespace split
    try:
        df = pd.read_csv(path, delim_whitespace=True, engine="python", encoding="utf-8-sig")
        return df, "whitespace"
    except Exception as e:
        raise e

def find_meta_n_corrupt(seq_folder: Path):
    """Look for *_meta.json files in the same folder and try to extract n_corrupt."""
    for jf in seq_folder.glob("*_meta.json"):
        try:
            j = json.loads(jf.read_text())
            if "n_corrupt" in j:
                return int(j["n_corrupt"])
        except Exception:
            continue
    return None

def inspect_and_report(path: Path, raw_first_bytes: bytes, df=None, used_sep=None):
    print("\n[DIAG] Problem file:", str(path))
    print("  raw first 256 bytes:", raw_first_bytes[:256])
    if used_sep is not None:
        print("  attempted sep:", used_sep)
    if df is not None:
        print("  parsed columns:", list(df.columns))
        # normalized
        print("  normalized columns:", [normalize_colname(c) for c in df.columns])
        print("  sample rows (first 3):")
        try:
            print(df.head(3).to_string(index=False))
        except Exception as e:
            print("   (failed to show sample rows):", e)
    else:
        print("  could not parse CSV into dataframe")

def main():
    all_rows_by_corrupt = {}  # n_corrupt_value -> list of dataframes
    unknown_rows = []

    files_found = 0
    for cat in CATEGORIES:
        cat_path = DATA_DIR / cat
        if not cat_path.exists():
            continue
        # find all results_metrics.csv under each sequence folder
        seq_paths = sorted(cat_path.glob("oxts*"))
        for seq in seq_paths:
            metrics = seq / "results_metrics.csv"
            if not metrics.exists():
                # skip silently
                continue
            files_found += 1
            # read raw first line bytes for diagnostics
            raw = b""
            try:
                with metrics.open("rb") as f:
                    raw = f.readline()
            except Exception as e:
                print(f"[WARN] Could not read raw bytes from {metrics}: {e}")
            # try to parse
            try:
                df, used_sep = try_read_csv(metrics)
            except Exception as e:
                print(f"[WARN] Failed to parse {metrics}: {e}")
                inspect_and_report(metrics, raw, df=None, used_sep=None)
                unknown_rows.append({"_source_file": str(metrics), "_error": "parse_failed"})
                continue

            # normalize columns
            orig_cols = list(df.columns)
            df.columns = [normalize_colname(c) for c in orig_cols]

            # If n_corrupt present after normalization, good
            if "n_corrupt" in df.columns:
                # ensure numeric type
                try:
                    df["n_corrupt"] = pd.to_numeric(df["n_corrupt"], errors="coerce")
                except Exception:
                    pass
                # attach metadata columns if missing
                if "dataset" not in df.columns:
                    df["dataset"] = seq.name
                if "dataset_group" not in df.columns:
                    df["dataset_group"] = cat
                # save grouped
                for val in df["n_corrupt"].dropna().unique():
                    key = int(val)
                    sub = df[df["n_corrupt"] == val].copy()
                    all_rows_by_corrupt.setdefault(key, []).append(sub)
                continue

            # attempt recovery: check for meta json
            recovered = False
            meta_nc = find_meta_n_corrupt(seq)
            if meta_nc is not None:
                # tag df with this n_corrupt
                df["n_corrupt"] = int(meta_nc)
                if "dataset" not in df.columns:
                    df["dataset"] = seq.name
                if "dataset_group" not in df.columns:
                    df["dataset_group"] = cat
                all_rows_by_corrupt.setdefault(meta_nc, []).append(df)
                recovered = True
                continue

            # attempt recovery heuristics: look for likely column names
            # common possibilities: ncorrupt, n-corrupt, corrupt_count, num_corrupt, corrupted
            heur_candidates = [c for c in df.columns if any(x in c for x in ("n_corrupt","ncorrupt","num_corrupt","corrupt","n_corrupts","n_corrupted"))]
            if heur_candidates:
                col = heur_candidates[0]
                try:
                    df["n_corrupt"] = pd.to_numeric(df[col], errors="coerce")
                    if df["n_corrupt"].notna().any():
                        for val in df["n_corrupt"].dropna().unique():
                            key = int(val)
                            sub = df[df["n_corrupt"] == val].copy()
                            all_rows_by_corrupt.setdefault(key, []).append(sub)
                        recovered = True
                except Exception:
                    recovered = False
            if recovered:
                continue

            # last resort: if the file contains only a single row, maybe this row is summary and n_corrupt might be in a field named 'n_file' or 'n' — try a few numeric columns that look like counts
            numeric_cols = [c for c in df.columns if pd.api.types.is_numeric_dtype(df[c])]
            possible = None
            for c in numeric_cols:
                # skip obvious metric columns
                if c in ("pos_accept_rate","vel_accept_rate","error_ratio","seed"):
                    continue
                # plausible count column names
                if any(k in c for k in ("n_file","n_files","n","total","count")):
                    possible = c
                    break
            if possible:
                # assume this column is n_corrupt only if values are small (<=10)
                vals = pd.to_numeric(df[possible], errors="coerce").dropna()
                if not vals.empty and vals.max() <= 10:
                    df["n_corrupt"] = vals.astype(int)
                    for val in df["n_corrupt"].dropna().unique():
                        key = int(val)
                        sub = df[df["n_corrupt"] == val].copy()
                        all_rows_by_corrupt.setdefault(key, []).append(sub)
                    continue

            # If still not recovered, report diagnostics and store in unknown
            inspect_and_report(metrics, raw, df=df, used_sep=used_sep)
            unknown_rows.append({"_source_file": str(metrics), "_columns": list(df.columns)})

    # end for seq

    # continue to next category

    # end category loop

    # after scanning all files
    if files_found == 0:
        print(f"[ERROR] No results_metrics.csv files found under {DATA_DIR}")
        sys.exit(1)

    # write combined outputs per n_corrupt
    print(f"[INFO] Writing combined outputs to {OUT_DIR}")
    for nc, df_list in sorted(all_rows_by_corrupt.items()):
        combined = pd.concat(df_list, ignore_index=True)
        outp = OUT_DIR / f"combined_n_corrupt_{nc}.csv"
        combined.to_csv(outp, index=False)
        print(f"[OK] Saved {outp} ({len(combined)} rows)")

    # unknowns
    if unknown_rows:
        print(f"[WARN] {len(unknown_rows)} files could not be assigned an n_corrupt. Saving a debug file.")
        dbg = pd.DataFrame(unknown_rows)
        dbg_path = OUT_DIR / "combined_unknown_n_corrupt_debug.csv"
        dbg.to_csv(dbg_path, index=False)
        print(f"[OK] Saved debug info -> {dbg_path}")

if __name__ == "__main__":
    main()

