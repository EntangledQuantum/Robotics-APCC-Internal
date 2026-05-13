"""Convert self-collision XLSX logger files to CSV."""

from __future__ import annotations

import glob
import os
import sys

import pandas as pd


def convert(src_dir: str, dst_dir: str) -> None:
    os.makedirs(dst_dir, exist_ok=True)
    files = sorted(glob.glob(os.path.join(src_dir, "self_collision_traj_*.xlsx")))
    if not files:
        print(f"No xlsx found in {src_dir}", file=sys.stderr)
        return
    for f in files:
        df = pd.read_excel(f)
        out = os.path.join(
            dst_dir, os.path.splitext(os.path.basename(f))[0] + ".csv"
        )
        df.to_csv(out, index=False)
        cols = list(df.columns)
        print(f"{out}  rows={len(df)}  cols[0:6]={cols[:6]}  cols[-3:]={cols[-3:]}")


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "."
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(src, "csv")
    convert(src, dst)
