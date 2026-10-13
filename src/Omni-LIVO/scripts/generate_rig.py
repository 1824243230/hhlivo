#!/usr/bin/env python3
"""Generate deterministic room measurements for all 8 cameras, 4 LiDARs and 2 IMUs."""
import argparse
import math
from pathlib import Path
import numpy as np
import yaml
import rosbag
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header
from hbs_multi import normalized_cloud, ros_time

EPOCH=1700000000000000000
def rz(a):return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1.]])
def motion(t):
    s=max(0,t-0.4);return rz(0.025*s*s),np.array([0.04*s*s,0.02*math.sin(s)**2,0.01*math.sin(s)**2])

def configuration(synthetic=True):
    optical=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
    sensors={'imus':[],'lidars':[],'cameras':[]}
    for id in range(2):
        R=np.eye(3) if id==0 else np.diag([1.,-1.,-1.]);p=np.zeros(3) if not synthetic or id==0 else np.array([0.2,0.05,0.03])
        sensors['imus'].append(dict(id=id,topic=f'/rig/imu/{id}/data_raw',rotation=R.flatten().tolist(),translation=p.tolist(),gyro_noise=0.02,accel_noise=0.2,gyro_walk=0.0001,accel_walk=0.001))
    for id in range(4):
        R=rz(id*math.pi/2) if synthetic else np.eye(3);p=np.array([0.1*math.cos(id*math.pi/2),0.1*math.sin(id*math.pi/2),0]) if synthetic else np.zeros(3)
        sensors['lidars'].append(dict(id=id,topic=f'/rig/lidar/{id}/points',rotation=R.flatten().tolist(),translation=p.tolist(),max_points=5000,blind=0.5,range_noise=0.03,beam_noise=0.001))
    for id in range(8):
        width,height=(160,120) if synthetic else (1920,1080);R=rz(id*math.pi/4)@optical
        sensors['cameras'].append(dict(id=id,topic=f'/rig/camera/{id}/image_raw',rotation=R.flatten().tolist(),translation=[0.,0.,0.],model='pinhole',width=width,height=height,scale=1. if synthetic else 0.5,fx=width/2,fy=width/2,cx=width/2,cy=height/2,distortion=[0.,0.,0.,0.]))
    return dict(calibration_mode='validated' if synthetic else 'placeholder',sensors=sensors,processing=dict(threads=4,max_match_points=20000,visual_points=800,camera_min=50,camera_max=150,patch_size=8,iterations=3,max_voxels=40000,max_visual_points=3000,update_period=0.1,wait_budget=0.15,sensor_timeout=0.3,init_duration=0.2,voxel_size=1.,map_radius=50.,photo_variance=100.,photo_gate=40.,rigid_rotation_noise=0.01,rigid_position_noise=0.02,geometry=True,visual=True,uncertainty=True,cross_camera=True))

def intersect(origin,rays):
    limits=np.array([12.,10.,4.]);rays=np.asarray(rays)
    distances=[]
    for k in range(3):
        d=rays[:,k];target=np.where(d>=0,limits[k],-limits[k]);distances.append(np.where(abs(d)>1e-9,(target-origin[k])/np.where(abs(d)>1e-9,d,1),np.inf))
    stack=np.stack(distances,axis=1);axis=np.argmin(stack,axis=1);distance=np.min(stack,axis=1)
    return origin+distance[:,None]*rays,axis

def texture(points,axis):
    # Continuous surface texture provides valid direct-photometric constraints.
    value=125+32*np.sin(2.1*points[:,0]+1.3*points[:,1]+0.9*points[:,2])+25*np.cos(1.7*points[:,0]-1.1*points[:,1]+2.3*points[:,2])+8*axis
    return np.clip(value,0,255).astype(np.uint8)

def render(t,c):
    w,h=c['width'],c['height'];xx,yy=np.meshgrid(np.arange(w),np.arange(h));rays=np.stack([(xx-c['cx'])/c['fx'],(yy-c['cy'])/c['fy'],np.ones_like(xx)],axis=-1).reshape(-1,3)
    R,p=motion(t);E=np.array(c['rotation']).reshape(3,3);ep=np.array(c['translation']);wc=R@E;origin=R@ep+p
    points,axis=intersect(origin,rays@wc.T);image=texture(points,axis).reshape(h,w)
    # Separate exposures are known and estimated independently.
    image=np.clip(image.astype(float)*(1+0.01*c['id']),0,255).astype(np.uint8)
    return np.repeat(image[:,:,None],3,axis=2)

def generate(args):
    dest=Path(args.output);dest.mkdir(parents=True,exist_ok=True);bag_path=dest/'synthetic.bag'
    if bag_path.exists():raise FileExistsError(bag_path)
    config=configuration();(dest/'config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    entries=[];truth=[]
    for k in range(int(args.duration*100)+1):
        t=k/100;stamp=EPOCH+round(t*1e9)
        for c in config['sensors']['imus']:
            id=c['id']
            if args.faults and id==0 and 1<=t<1.5:continue
            if args.fail_both and 1<=t:continue
            eps=1e-4;E=np.array(c['rotation']).reshape(3,3);lever=np.array(c['translation'])
            R,p=motion(t);Rp,pp=motion(t+eps);Rm,pm=motion(max(0,t-eps));Ri=R@E
            if t<0.001:acc=Ri.T@np.array([0.,0.,9.81]);gyro=np.zeros(3)
            else:
                acc=Ri.T@((pp+Rp@lever-2*(p+R@lever)+pm+Rm@lever)/eps**2+np.array([0.,0.,9.81]))
                gyro=E.T@np.array([0.,0.,0.05*max(0,t-0.4)])
            msg=Imu(header=Header(seq=k,stamp=ros_time(stamp),frame_id=f'imu_{id}'));msg.orientation.w=1;msg.orientation_covariance[0]=-1
            msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=acc;msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=gyro
            if args.faults and id==1 and 2<=t<2.05:msg.angular_velocity.x=1000
            entries.append((stamp,0,f'/rig/imu/{id}/data_raw',msg))
    for k in range(int(args.duration*10)+1):
        t=k/10;stamp=EPOCH+round(t*1e9);R,p=motion(t);truth.append([stamp/1e9,*p,0,0,math.sin(0.025*max(0,t-0.4)**2/2),math.cos(0.025*max(0,t-0.4)**2/2)])
        for c in config['sensors']['cameras']:
            id=c['id']
            if args.faults and id==3 and 1<=t<1.7:continue
            img=render(t,c);msg=Image(header=Header(seq=k,stamp=ros_time(stamp),frame_id=f'camera_{id}'),height=img.shape[0],width=img.shape[1],encoding='bgr8',is_bigendian=False,step=img.shape[1]*3,data=img.tobytes())
            entries.append((stamp,2,f'/rig/camera/{id}/image_raw',msg))
        if k==0:continue
        for c in config['sensors']['lidars']:
            id=c['id']
            if args.faults and id==2 and 1<=t<1.6:continue
            E=np.array(c['rotation']).reshape(3,3);lever=np.array(c['translation']);count=1200
            az=np.arange(count)*2*math.pi/count;el=0.65*np.sin(np.arange(count)*0.73)
            rays=np.stack([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)],axis=1)
            xyz=[];offset=np.arange(count,dtype=np.int64)*99900000//(count-1);begin=stamp-100000000
            for j in range(count):
                at=t-0.1+offset[j]*1e-9;Rt,pt=motion(at);origin=Rt@lever+pt;world,axis=intersect(origin,np.array([Rt@E@rays[j]]));xyz.append(E.T@Rt.T@(world[0]-origin))
            msg=normalized_cloud(np.array(xyz),np.full(count,100),offset,np.arange(count)%32,begin,id,k)
            entries.append((stamp,1,f'/rig/lidar/{id}/points',msg))
    with rosbag.Bag(str(bag_path),'w',compression='lz4') as bag:
        for stamp,order,topic,msg in sorted(entries,key=lambda e:(e[0],e[1],e[2])):bag.write(topic,msg,ros_time(stamp))
    np.savetxt(dest/'truth.txt',np.array(truth),fmt='%.9f');print(bag_path)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--duration',type=float,default=3);p.add_argument('--faults',action='store_true');p.add_argument('--fail-both',action='store_true');args=p.parse_args()
    if args.duration<0.5:p.error('duration must be at least 0.5 seconds')
    generate(args)
