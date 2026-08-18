#!/usr/bin/env python3
"""Syncs rolling frame buffer from robot (/tmp/lekiwi_frames/) to ./debug_frames/.
Automatically keeps only the last 20 frames locally and overwrites:
  - latest_frame.jpg (active stream)
  - latest_cam0.jpg  (base camera)
  - latest_cam1.jpg  (arm camera)
"""

import collections
import os
import pathlib
import shutil
import time
import paramiko

LOCAL_OUT = pathlib.Path(__file__).parent.parent / "debug_frames"
LOCAL_OUT.mkdir(parents=True, exist_ok=True)
MAX_LOCAL_FRAMES = 20

def sync_frames(host="lekiwi.local", user="river", password="AVeryStrongHorseCosting89$"):
    print(f"Connecting to {user}@{host} to sync frames...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username=user, password=password, timeout=10)
    sftp = ssh.open_sftp()
    
    remote_dir = "/tmp/lekiwi_frames"
    print(f"Syncing rolling {MAX_LOCAL_FRAMES} frames from {remote_dir} -> {LOCAL_OUT}...")
    
    try:
        while True:
            try:
                files = sorted(sftp.listdir(remote_dir))
                jpgs = [f for f in files if f.startswith("frame_cam") and f.endswith(".jpg")]
                # Grab newest frames
                for f in jpgs[-MAX_LOCAL_FRAMES:]:
                    l_path = LOCAL_OUT / f
                    if not l_path.exists():
                        r_path = f"{remote_dir}/{f}"
                        sftp.get(r_path, str(l_path))
                        # Update latest_frame.jpg
                        try:
                            shutil.copyfile(str(l_path), str(LOCAL_OUT / "latest_frame.jpg"))
                            if "cam0" in f:
                                shutil.copyfile(str(l_path), str(LOCAL_OUT / "latest_cam0.jpg"))
                            elif "cam1" in f:
                                shutil.copyfile(str(l_path), str(LOCAL_OUT / "latest_cam1.jpg"))
                        except Exception:
                            pass
                
                # Delete any local files beyond MAX_LOCAL_FRAMES
                all_local = sorted([p for p in LOCAL_OUT.glob("frame_cam*.jpg")])
                while len(all_local) > MAX_LOCAL_FRAMES:
                    oldest = all_local.pop(0)
                    try:
                        oldest.unlink()
                    except Exception:
                        pass
            except Exception as e:
                pass
            time.sleep(0.5)
    finally:
        sftp.close()
        ssh.close()

if __name__ == "__main__":
    sync_frames()
