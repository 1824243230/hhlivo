#!/usr/bin/env python3
"""Validate a completed runtime result, optionally against the HBS conversion counts."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag

def verify(directory,conversion=None,require_all=False):
    root=Path(directory);summary=json.loads((root/'summary.json').read_text())
    assert summary['updates']>0 and summary['map_points']>0,'missing odometry/map output'
    assert summary['minimum_covariance_eigenvalue']>=-1e-9
    for path in root.glob('trajectory*.txt'):
        if not path.stat().st_size:continue
        trajectory=np.atleast_2d(np.loadtxt(path));assert np.isfinite(trajectory).all()
        assert np.all(np.diff(trajectory[:,0])>0),'non-monotonic trajectory segment'
        assert np.allclose(np.linalg.norm(trajectory[:,4:8],axis=1),1,atol=1e-6)
    outputs=0
    for path in root.glob('estimates*.bag'):
        with rosbag.Bag(str(path)) as bag:
            for _,msg,_ in bag.read_messages(topics=['/rig/odometry']):
                outputs+=1
                for covariance in (msg.pose.covariance,msg.twist.covariance):
                    matrix=np.asarray(covariance).reshape(6,6);assert np.isfinite(matrix).all()
                    assert np.linalg.eigvalsh(matrix).min()>-1e-9
    assert outputs==summary['updates'],'output bag/summary counts disagree'
    if conversion:
        converted=json.loads(Path(conversion).read_text())
        for key,count in converted['output_counts'].items():assert summary['sensors'][key]['received']==count,(key,count)
    if require_all:
        assert all(v['matched']>0 and v['inserted']>0 for v in summary['lidars'].values())
        assert all(v['observations']>0 for v in summary['cameras'].values())
        assert all(v['propagations']>0 for v in summary['imus'].values())
    frames=np.genfromtxt(root/'frames.csv',delimiter=',',names=True)
    report={'status':'passed','sensor_instances':len(summary['sensors']),'odometry_updates':summary['updates'],'map_points':summary['map_points'],
        'peak_rss_mb':summary['peak_rss_mb'],'mean_compute_ms':summary['mean_compute_ms'],'p95_compute_ms':float(np.percentile(np.atleast_1d(frames['compute_ms']),95)),
        'all_sensor_counts_match_converter':bool(conversion),'all_sensors_contributed':require_all,'tracking_at_end':summary['tracking']}
    (root/'validation.json').write_text(json.dumps(report,indent=2)+'\n');return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory');p.add_argument('--conversion-report');p.add_argument('--require-all',action='store_true');a=p.parse_args()
    print(json.dumps(verify(a.directory,a.conversion_report,a.require_all),indent=2))
