#!/usr/bin/env python3
"""Analyze SW01 UDP 6100/6101 pcap captures for calibration.

This script is a helper for docs/REAL_MACHINE_CALIBRATION.md. It does not
replace the physical measurements; it prints the raw protocol numbers you need
to fill in B1-B13 of the calibration matrix.

Examples
--------
# Analyze a static capture (robot stationary):
python3 tools/calibration_analyze.py --odom /tmp/odom_static.pcap \\
    --cloud /tmp/lidar_static.pcap

# Analyze a motion capture (e.g. push forward 1 m):
python3 tools/calibration_analyze.py --odom /tmp/odom_forward_1m.pcap

The script expects pcap files written by:
    sudo tcpdump -i eno1 -w /tmp/odom.pcap    udp port 6101
    sudo tcpdump -i eno1 -w /tmp/lidar.pcap   udp port 6100
"""

from __future__ import annotations

import argparse
import collections
import math
import struct
import sys
from typing import Iterator, Optional, Tuple


def iter_pcap_frames(
    path: str, max_packets: Optional[int] = None
) -> Iterator[Tuple[float, bytes]]:
    """Yield (timestamp_seconds, raw_link_frame) from a classic pcap file."""
    with open(path, "rb") as f:
        header = f.read(24)
        if len(header) < 24:
            return
        magic = header[:4]
        if magic not in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1"):
            raise ValueError("unsupported pcap magic; expected little-endian pcap")
        offset = 24
        count = 0
        while True:
            rec = f.read(16)
            if len(rec) < 16:
                break
            ts_sec, ts_usec, incl_len, orig_len = struct.unpack("<IIII", rec)
            frame = f.read(incl_len)
            if len(frame) < incl_len:
                break
            yield ts_sec + ts_usec * 1e-6, frame
            count += 1
            if max_packets is not None and count >= max_packets:
                break


def udp_payload(frame: bytes) -> Optional[bytes]:
    """Extract UDP payload from an Ethernet/IPv4/UDP frame."""
    if len(frame) < 14:
        return None
    ip_off = 14
    ihl = (frame[ip_off] & 0x0F) * 4
    if len(frame) < ip_off + ihl + 8:
        return None
    if frame[ip_off + 9] != 17:  # UDP
        return None
    udp_off = ip_off + ihl
    udp_len = struct.unpack_from(">H", frame, udp_off + 4)[0]
    if udp_len < 8 or udp_off + udp_len > len(frame):
        return None
    return frame[udp_off + 8 : udp_off + udp_len]


def analyze_odom(path: str, max_packets: int) -> None:
    print(f"=== UDP 6101 odometry: {path} ===")
    samples = []
    lengths = collections.Counter()
    for _, frame in iter_pcap_frames(path, max_packets):
        pl = udp_payload(frame)
        if pl is None:
            continue
        lengths[len(pl)] += 1
        sample = None
        if len(pl) == 68 and pl[:2] == b"\x55\xaa" and pl[-2:] == b"\x00\xff":
            ts = struct.unpack_from("<Q", pl, 2)[0]
            x, y, z = struct.unpack_from("<qqq", pl, 10)
            qx, qy, qz, qw = struct.unpack_from("<qqqq", pl, 34)
            sample = (ts, x, y, z, qx, qy, qz, qw)
        elif len(pl) == 80 and pl[:2] == b"\x55\xaa" and pl[-2:] == b"\x00\xff":
            ts = struct.unpack_from("<Q", pl, 8)[0]
            x, y, z = struct.unpack_from("<qqq", pl, 16)
            qx, qy, qz, qw = struct.unpack_from("<qqqq", pl, 40)
            sample = (ts, x, y, z, qx, qy, qz, qw)
        if sample is not None:
            samples.append(sample)

    print("packet length distribution:", dict(lengths))
    print("parsed odometry packets:", len(samples))
    if not samples:
        return

    first, last = samples[0], samples[-1]
    print("first raw:", first)
    print("last  raw:", last)
    print("raw delta x,y,z:", last[1] - first[1], last[2] - first[2], last[3] - first[3])

    xs = [s[1] for s in samples]
    ys = [s[2] for s in samples]
    zs = [s[3] for s in samples]
    print(
        "raw range x/y/z:",
        f"x [{min(xs)},{max(xs)}] delta {max(xs)-min(xs)}",
        f"y [{min(ys)},{max(ys)}] delta {max(ys)-min(ys)}",
        f"z [{min(zs)},{max(zs)}] delta {max(zs)-min(zs)}",
    )

    for label, s in (("first", first), ("last", last)):
        qs = s[4:8]
        norm = math.sqrt(sum(v * v for v in qs))
        print(
            f"{label} quaternion raw (qx,qy,qz,qw): {qs} norm={norm:.3f} "
            f"normalized=({qs[0]/norm:.4f},{qs[1]/norm:.4f},{qs[2]/norm:.4f},{qs[3]/norm:.4f})"
        )

    # If the capture is static, the normalized quaternion should be nearly
    # constant; report the spread as a sanity check.
    qnorms = []
    for s in samples:
        qs = s[4:8]
        norm = math.sqrt(sum(v * v for v in qs))
        if norm > 0:
            qnorms.append(tuple(v / norm for v in qs))
    if len(qnorms) > 1:
        q0 = qnorms[0]
        diffs = []
        for q in qnorms:
            diff = max(abs(a - b) for a, b in zip(q0, q))
            diffs.append(diff)
        print(
            "quaternion max component change vs first sample:",
            f"{max(diffs):.6f} (0=perfectly static)",
        )


def analyze_cloud(path: str, max_packets: int) -> None:
    print(f"=== UDP 6100 point cloud: {path} ===")
    lengths = collections.Counter()
    frames: dict[int, dict] = {}
    first_points = []
    for _, frame in iter_pcap_frames(path, max_packets):
        pl = udp_payload(frame)
        if pl is None:
            continue
        lengths[len(pl)] += 1
        if len(pl) != 1422 or pl[:2] != b"\x55\xaa" or pl[-2:] != b"\x00\xff":
            continue
        ts = struct.unpack_from("<Q", pl, 2)[0]
        total = struct.unpack_from("<I", pl, 10)[0]
        idx = struct.unpack_from("<I", pl, 14)[0]
        pos = struct.unpack_from("<H", pl, 18)[0]
        x, y, z = struct.unpack_from("<iii", pl, 20)
        if len(first_points) < 20:
            first_points.append((ts, total, idx, pos, x, y, z))
        frame = frames.setdefault(
            ts,
            {
                "total": total,
                "indexes": [],
                "poslist": [],
                "points": 0,
                "last_idx": 0,
                "last_pos": 0,
            },
        )
        if frame["total"] != total:
            frame["total_conflict"] = True
        frame["indexes"].append(idx)
        frame["poslist"].append(pos)
        frame["points"] += pos
        frame["last_idx"] = idx
        frame["last_pos"] = pos

    print("packet length distribution:", dict(lengths))
    print("distinct frame timestamps observed:", len(frames))
    if not frames:
        return

    # Print one complete frame's point-range coverage.
    sample_ts = next(iter(frames))
    f = frames[sample_ts]
    # Build the sorted (start, count) list from the observed indexes and the
    # per-packet valid point counts.
    pos_by_index = dict(zip(f["indexes"], f["poslist"]))
    ranges = sorted(pos_by_index.items())
    expected = 0
    gaps = []
    for start, count in ranges:
        if start != expected:
            gaps.append((expected, start))
        expected = start + count
    if expected != f["total"]:
        gaps.append((expected, f["total"]))
    print(
        f"sample frame ts={sample_ts} total={f['total']} "
        f"packets={len(ranges)} points_sum={f['points']} "
        f"index_min={ranges[0][0]} index_max={ranges[-1][0]} "
        f"last_range=[{f['last_idx']},{f['last_idx']+f['last_pos']})"
    )
    if gaps:
        print("coverage gaps in sample frame:", gaps[:20])
    else:
        print("coverage gaps: none (frame fully covers [0,total))")
    print("first 10 packets (ts,total,idx,pos,first point x,y,z):")
    for row in first_points[:10]:
        print("  ", row)

    totals = {f["total"] for f in frames.values()}
    print("unique totals:", len(totals), "sample:", sorted(totals)[:5])
    if any(f.get("total_conflict") for f in frames.values()):
        print("WARNING: total field changed within the same timestamp")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odom", help="path to UDP 6101 pcap file")
    parser.add_argument("--cloud", help="path to UDP 6100 pcap file")
    parser.add_argument(
        "--max-packets",
        type=int,
        default=20000,
        help="maximum packets to read from each pcap (default 20000)",
    )
    args = parser.parse_args()

    if not args.odom and not args.cloud:
        parser.error("provide at least one of --odom or --cloud")

    if args.odom:
        analyze_odom(args.odom, args.max_packets)
        print()
    if args.cloud:
        analyze_cloud(args.cloud, args.max_packets)

    return 0


if __name__ == "__main__":
    sys.exit(main())