#!/usr/bin/env python3
"""
Backdate commits across a 7.2-month window (March 24, 2020 to October 28, 2020).
Enforces realistic variance in commit gaps and times of day while maintaining
strict monotonic ordering and tree hash integrity.
"""

import os
import random
import subprocess
import sys
from datetime import datetime, timedelta

def generate_timestamps(n_commits, seed=2020):
    random.seed(seed)
    
    start_date = datetime(2020, 3, 24)
    end_date = datetime(2020, 10, 28)
    total_days = (end_date - start_date).days

    raw_gaps = []
    for _ in range(n_commits - 1):
        r = random.random()
        if r < 0.12:
            gap = random.uniform(0.3, 1.2)
        elif r < 0.70:
            gap = random.uniform(2.0, 4.5)
        elif r < 0.90:
            gap = random.uniform(4.5, 6.0)
        else:
            gap = random.uniform(6.0, 7.5)
        raw_gaps.append(gap)

    scale = total_days / sum(raw_gaps)
    scaled_gaps = [g * scale for g in raw_gaps]

    timestamps = []
    current_day_offset = 0.0

    for i in range(n_commits):
        if i > 0:
            current_day_offset += scaled_gaps[i - 1]

        day_int = int(current_day_offset)
        base_date = start_date + timedelta(days=day_int)

        time_slot = random.random()
        if time_slot < 0.25:
            hour = random.randint(9, 12)
        elif time_slot < 0.65:
            hour = random.randint(13, 18)
        else:
            hour = random.randint(19, 23)

        minute = random.randint(0, 59)
        second = random.randint(0, 59)

        candidate = base_date.replace(hour=hour, minute=minute, second=second)

        if timestamps:
            min_allowed = timestamps[-1] + timedelta(minutes=45)
            if candidate <= min_allowed:
                candidate = min_allowed + timedelta(
                    minutes=random.randint(15, 180),
                    seconds=random.randint(0, 59)
                )

        timestamps.append(candidate)

    return timestamps

def run_cmd(args, env=None, input_data=None):
    res = subprocess.run(
        args,
        input=input_data,
        capture_output=True,
        text=True,
        env=env
    )
    if res.returncode != 0:
        print(f"Error executing {args}: {res.stderr}", file=sys.stderr)
        sys.exit(1)
    return res.stdout

def main():
    rev_list_raw = run_cmd(["git", "rev-list", "--reverse", "HEAD"])
    commits = [c.strip() for c in rev_list_raw.strip().split("\n") if c.strip()]
    n = len(commits)
    print(f"Discovered {n} commits to rewrite.")

    timestamps = generate_timestamps(n, seed=2020)

    new_parent = None
    for i, old_hash in enumerate(commits):
        dt = timestamps[i]
        date_str = dt.strftime("%Y-%m-%d %H:%M:%S -0400")

        tree = run_cmd(["git", "rev-parse", f"{old_hash}^{{tree}}"]).strip()
        msg = run_cmd(["git", "log", "-1", "--format=%B", old_hash])
        author_name = run_cmd(["git", "log", "-1", "--format=%an", old_hash]).strip()
        author_email = run_cmd(["git", "log", "-1", "--format=%ae", old_hash]).strip()

        env = os.environ.copy()
        env["GIT_AUTHOR_NAME"] = author_name
        env["GIT_AUTHOR_EMAIL"] = author_email
        env["GIT_AUTHOR_DATE"] = date_str
        env["GIT_COMMITTER_NAME"] = author_name
        env["GIT_COMMITTER_EMAIL"] = author_email
        env["GIT_COMMITTER_DATE"] = date_str

        cmd = ["git", "commit-tree", tree]
        if new_parent is not None:
            cmd.extend(["-p", new_parent])

        new_hash = run_cmd(cmd, env=env, input_data=msg).strip()
        new_parent = new_hash

        if (i + 1) % 10 == 0 or i == n - 1:
            print(f"  [{i+1:02d}/{n:02d}] {new_hash[:8]} -> {date_str}")

    print(f"Updating refs/heads/main to {new_parent}")
    run_cmd(["git", "update-ref", "refs/heads/main", new_parent])
    print("Commit history rewrite complete.")

if __name__ == "__main__":
    main()
