#!/usr/bin/env python3
"""End-to-end assertions against the real runtime, not a mock filter."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
import rosbag

def run(binary,config,bag,output):
    bags=[str(p) for p in bag] if isinstance(bag,list) else [str(bag)]
    with open(str(output)+'.log','w') as fd:subprocess.run([binary,'--config',str(config),'--bag',*bags,'--output',str(output)],stdout=fd,stderr=subprocess.STDOUT,check=True,timeout=180)
    summary=json.loads((output/'summary.json').read_text());trajectory=np.atleast_2d(np.loadtxt(output/'trajectory.txt'))
    assert len(trajectory)>5 and np.isfinite(trajectory).all()
    assert np.all(np.diff(trajectory[:,0])>0),'non-monotonic output'
    assert summary['map_points']>0,'empty map'
    with rosbag.Bag(str(output/'estimates.bag')) as bag:
        for _,m,_ in bag.read_messages(topics=['/rig/odometry']):
            p=np.asarray(m.pose.covariance).reshape(6,6);assert np.isfinite(p).all();assert np.linalg.eigvalsh(p).min()>-1e-8
            twist=np.asarray(m.twist.covariance).reshape(6,6);assert np.isfinite(twist).all();assert np.linalg.eigvalsh(twist).min()>-1e-8
    return summary,trajectory

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',required=True);p.add_argument('--work',required=True);a=p.parse_args()
    root=Path(a.work);root.mkdir(parents=True,exist_ok=True);script=Path(__file__).with_name('generate_rig.py')
    for name,flags in [('normal',[]),('faults',['--faults']),('both',['--fail-both'])]:subprocess.run([sys.executable,str(script),'--output',str(root/name),'--duration','3',*flags],check=True)
    baseline,t=run(a.binary,root/'normal/config.yaml',root/'normal/synthetic.bag',root/'normal/run')
    assert baseline['state_dimension']==41 and baseline['tracking']
    assert baseline['geometry_updates']>5 and baseline['visual_updates']>5
    assert all(v['observations']>0 for v in baseline['cameras'].values()),'all eight cameras must contribute'
    assert all(v['matched']>0 and v['inserted']>0 for v in baseline['lidars'].values()),'all four LiDARs must contribute'
    assert all(v['propagations']>0 for v in baseline['imus'].values()),'both IMUs must propagate'
    assert baseline['spline_queries']>0 and baseline['cross_camera_observations']>0
    truth=np.loadtxt(root/'normal/truth.txt');gt=np.stack([np.interp(t[:,0],truth[:,0],truth[:,i]) for i in (1,2,3)],axis=1)
    error=float(np.sqrt(np.mean(np.sum((t[:,1:4]-gt)**2,axis=1))));assert error<0.15,('synthetic RMSE',error)
    faults,ft=run(a.binary,root/'faults/config.yaml',root/'faults/synthetic.bag',root/'faults/run')
    frames=np.genfromtxt(root/'faults/run/frames.csv',delimiter=',',names=True)
    assert 26 in frames['state_dimension'],'missing single-IMU marginalization'
    assert faults['state_dimension']==41 and faults['tracking'],'IMU did not recover'
    assert any(v['recoveries']>0 for k,v in faults['sensors'].items() if k.startswith('imu/'))
    assert ft[-1,0]>t[0,0]+2.5,'fault blocked subsequent processing'
    both,bt=run(a.binary,root/'both/config.yaml',root/'both/synthetic.bag',root/'both/run')
    assert not both['tracking'],'all-IMU outage must be explicit'
    assert bt[-1,0]<1700000001.2,'stale IMU integrated after complete outage'
    repeat,rt=run(a.binary,root/'normal/config.yaml',root/'normal/synthetic.bag',root/'normal/repeat')
    assert np.allclose(t,rt,rtol=0,atol=1e-9),'non-deterministic trajectory'
    config=yaml.safe_load((root/'normal/config.yaml').read_text());config['processing']['visual']=False;(root/'normal/no_visual.yaml').write_text(yaml.safe_dump(config))
    no_visual,_=run(a.binary,root/'normal/no_visual.yaml',root/'normal/synthetic.bag',root/'normal/no_visual')
    assert no_visual['visual_rows']==0 and no_visual['geometry_rows']>0,'visual ablation ineffective'
    config=yaml.safe_load((root/'normal/config.yaml').read_text());config['sensors']['imus'][1]['enabled']=False;(root/'normal/single_imu.yaml').write_text(yaml.safe_dump(config))
    single,_=run(a.binary,root/'normal/single_imu.yaml',root/'normal/synthetic.bag',root/'normal/single_imu')
    assert single['state_dimension']==26 and single['imus']['1']['propagations']==0,'IMU ablation ineffective'
    reset,_=run(a.binary,root/'normal/config.yaml',[root/'normal/synthetic.bag',root/'normal/synthetic.bag'],root/'normal/reset')
    assert reset['resets']==1 and (root/'normal/reset/trajectory_1.txt').exists(),'clock rollback did not create a new segment'
    seg=np.loadtxt(root/'normal/reset/trajectory_1.txt');assert np.all(np.diff(seg[:,0])>0)
    report={'status':'passed','synthetic_position_rmse_m':error,'normal_updates':baseline['updates'],'fault_updates':faults['updates'],'all_cameras_contribute':True,'all_lidars_contribute':True,'both_imus_contribute':True,'repeatable':True,'fault_recovery':True,'ablation':True,'clock_reset':True}
    (root/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
