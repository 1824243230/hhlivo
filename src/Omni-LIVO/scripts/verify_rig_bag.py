#!/usr/bin/env python3
"""Read back normalized sensor bags, independently checking wire fields and times."""
import argparse
from collections import Counter
import json
from pathlib import Path
import rosbag
import numpy as np

def verify(paths):
    counts=Counter();last={};previous=-1;dimensions={};points=Counter()
    for path in paths:
        with rosbag.Bag(path) as bag:
            for topic,msg,record in bag.read_messages():
                if not topic.startswith('/rig/'):continue
                assert record.to_nsec()>=previous,'bag recording time reversed';previous=record.to_nsec()
                stamp=msg.header.stamp.to_nsec();assert stamp>last.get(topic,-1),'duplicate/reversed per-sensor stamp';last[topic]=stamp;counts[topic]+=1
                if '/camera/' in topic:
                    assert msg._type=='sensor_msgs/Image' and msg.encoding=='bgr8'
                    assert msg.step==msg.width*3 and len(msg.data)==msg.step*msg.height
                    dimensions[topic]=[msg.width,msg.height]
                elif '/imu/' in topic:
                    assert msg._type=='sensor_msgs/Imu'
                    assert np.isfinite([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]).all()
                    assert msg.orientation_covariance[0]==-1
                elif '/lidar/' in topic:
                    assert msg._type=='sensor_msgs/PointCloud2' and not msg.is_bigendian and msg.point_step==24
                    assert msg.row_step==msg.width*24 and len(msg.data)==msg.row_step*msg.height
                    expected={'x':(0,7),'y':(4,7),'z':(8,7),'intensity':(12,7),'offset_time':(16,6),'ring':(20,4)}
                    assert {f.name:(f.offset,f.datatype) for f in msg.fields}==expected
                    offsets=np.ndarray((msg.width,),dtype='<u4',buffer=msg.data,offset=16,strides=(24,))
                    assert not len(offsets) or np.all(offsets[1:]>=offsets[:-1])
                    assert not len(offsets) or stamp+int(offsets[-1])<=record.to_nsec()
                    for position in (0,4,8):assert np.isfinite(np.ndarray((msg.width,),dtype='<f4',buffer=msg.data,offset=position,strides=(24,))).all()
                    points[topic]+=msg.width
    expected=[f'/rig/camera/{i}/image_raw' for i in range(8)]+[f'/rig/lidar/{i}/points' for i in range(4)]+[f'/rig/imu/{i}/data_raw' for i in range(2)]
    assert all(counts[t]>0 for t in expected),'not all 14 sensors present'
    return {'status':'passed','counts':dict(counts),'lidar_points':dict(points),'image_dimensions':dimensions}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bags',nargs='+');p.add_argument('--output',required=True);a=p.parse_args()
    result=verify(a.bags);Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result['counts'],indent=2))
