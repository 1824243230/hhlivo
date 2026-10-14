#!/usr/bin/env python3
"""Stream all HBS sensor instances to one normalized, availability-ordered ROS1 bag."""
import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
from fractions import Fraction
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header
from google.protobuf import descriptor_pb2, descriptor_pool, message_factory

CAM_HEAD = struct.Struct('<BBHIIIIQQ')
LIDAR_HEAD = struct.Struct('<Q8I4B')
POINT = np.dtype([('x','<i2'),('y','<i2'),('z','<i2'),('distance','<u2'),
                  ('hi','<u2'),('lo','<u2'),('ref','u1'),('ring','u1'),('resv','<u2')])
NORMALIZED = np.dtype({'names':['x','y','z','intensity','offset_time','ring'],
    'formats':['<f4','<f4','<f4','<f4','<u4','<u2'],'offsets':[0,4,8,12,16,20],'itemsize':24})

class FormatError(ValueError):
    pass

def read_exact(fd, size, context):
    offset=fd.tell();data=fd.read(size)
    if len(data)!=size:raise FormatError(f'{context}: truncated at {offset}, expected {size}, got {len(data)}')
    return data

def records(fd):
    magic,count=struct.unpack('<II',read_exact(fd,8,'header'))
    if magic!=0xC7C7C7C7 or not 0<count<=10000:raise FormatError('invalid HBS header')
    read_exact(fd,80,'reserved')
    for _ in range(count):
        total,n,name,length=struct.unpack('<QI256sI',read_exact(fd,272,'topic header'))
        if not 0<n<=256 or total!=16+n+length:raise FormatError('invalid topic metadata')
        read_exact(fd,length,'topic payload')
    if read_exact(fd,4,'data magic')!=b'\xb6'*4:raise FormatError('invalid data magic')
    size=os.fstat(fd.fileno()).st_size
    while fd.tell()<size:
        start=fd.tell();length=struct.unpack('<I',read_exact(fd,4,'record size'))[0]
        if length<36 or length>size-fd.tell():raise FormatError(f'invalid record length at {start}')
        data=read_exact(fd,length,'record');channel_len=struct.unpack_from('<H',data,28)[0]
        pos=30+channel_len
        if channel_len<1 or pos+2>length:raise FormatError(f'invalid channel at {start}')
        class_len=struct.unpack_from('<H',data,pos)[0];end=pos+2+class_len
        if class_len<1 or end+4>length:raise FormatError(f'invalid class at {start}')
        payload_len=struct.unpack_from('<I',data,end)[0];payload_pos=end+4
        if payload_len>length-payload_pos:raise FormatError(f'invalid payload at {start}')
        yield data[30:pos].rstrip(b'\0').decode(),data[pos+2:end].rstrip(b'\0').decode(),start+4+payload_pos,data[payload_pos:payload_pos+payload_len]

def imu_type():
    desc=descriptor_pb2.FileDescriptorProto(name='service_itf_imu.proto',package='svc.itf.imu',syntax='proto2')
    msg=desc.message_type.add(name='ItfImuData')
    names=['timestampUS','instId','vendorId','uuid','accX','accY','accZ','gyrX','gyrY','gyrZ',
           'roll','pitch','yaw','q0','q1','q2','q3','angle_type','magX','magY','magZ','temperature','ErrorCode','sampleTime','resv']
    for i,name in enumerate(names,1):
        typ=4 if i in (1,24,25) else 13 if i in (2,3,4,18,23) else 2
        msg.field.add(name=name,number=i,type=typ,label=2 if i<=18 else 1)
    pool=descriptor_pool.DescriptorPool();pool.Add(desc);spec=pool.FindMessageTypeByName('svc.itf.imu.ItfImuData')
    return message_factory.GetMessageClass(spec) if hasattr(message_factory,'GetMessageClass') else message_factory.MessageFactory(pool).GetPrototype(spec)

ImuProto=imu_type()

def ros_time(ns):
    sec,nsec=divmod(int(ns),1000000000)
    if not 0<=sec<=0xFFFFFFFF:raise FormatError('timestamp outside ROS1 range')
    return rospy.Time(sec,nsec)

def normalized_cloud(xyz,intensity,offsets,ring,stamp,id,seq=0):
    if len(offsets) and (int(np.min(offsets))<0 or int(np.max(offsets))>0xFFFFFFFF):raise FormatError('point offset outside uint32 nanosecond range')
    order=np.argsort(offsets,kind='stable');wire=np.zeros(len(offsets),dtype=NORMALIZED)
    for k,name in enumerate(('x','y','z')):wire[name]=xyz[order,k]
    wire['intensity']=np.asarray(intensity)[order];wire['offset_time']=np.asarray(offsets)[order];wire['ring']=np.asarray(ring)[order]
    fields=[PointField(name=name,offset=NORMALIZED.fields[name][1],datatype=PointField.FLOAT32 if name in ('x','y','z','intensity') else PointField.UINT32 if name=='offset_time' else PointField.UINT16,count=1) for name in NORMALIZED.names]
    return PointCloud2(header=Header(seq=seq,stamp=ros_time(stamp),frame_id=f'lidar_{id}'),height=1,width=len(wire),fields=fields,is_bigendian=False,point_step=24,row_step=24*len(wire),data=wire.tobytes(),is_dense=True)

def lidar_payload(payload,id):
    if len(payload)<LIDAR_HEAD.size:raise FormatError('truncated LiDAR header')
    h=LIDAR_HEAD.unpack_from(payload);count=h[8]
    if h[9] not in (0,2) or count==0 or len(payload)!=44+count*16:raise FormatError('invalid LiDAR coordinates/count')
    p=np.frombuffer(payload,POINT,count,44)
    offsets=((p['hi'].astype(np.uint32)<<16)|p['lo']).view(np.int32).astype(np.int64)
    unit=1 if id==0 else 1000
    absolute=((h[2]<<32)|h[4])*unit+offsets*unit
    return h,p,absolute

@dataclass
class Entry:
    kind:str
    id:int
    stamp:int
    available:int
    offset:int
    length:int
    width:int=0
    height:int=0
    spool:str=''

def index(path,report):
    entries=[];cameras=defaultdict(lambda:defaultdict(list));stats=defaultdict(Counter)
    with open(path,'rb') as fd:
        for channel,name,offset,payload in records(fd):
            report['records']+=1
            try:
                if name=='raw.itf_camera_h265':
                    h=CAM_HEAD.unpack_from(payload);id=h[2]
                    if id not in range(8):continue
                    if h[0]!=0 or h[6]!=len(payload)-CAM_HEAD.size or not 0<h[4]<=16384 or not 0<h[5]<=16384:raise FormatError('invalid camera packet')
                    stamp=h[7]*1000;ros_time(stamp)
                    cameras[id][stamp].append(Entry('camera',id,stamp,stamp,offset+CAM_HEAD.size,h[6],h[4],h[5]));stats[f'camera/{id}']['packets']+=1
                elif name=='raw.itf_lidar_point_mix':
                    id=LIDAR_HEAD.unpack_from(payload)[1]
                    if id not in range(4):continue
                    h,p,absolute=lidar_payload(payload,id);stamp=int(absolute.min());end=int(absolute.max());ros_time(stamp)
                    entries.append(Entry('lidar',id,stamp,end,offset,len(payload)));stats[f'lidar/{id}']['frames']+=1;stats[f'lidar/{id}']['points']+=len(p)
                elif name=='svc.itf.imu.ItfImuData':
                    m=ImuProto.FromString(payload)
                    if not m.IsInitialized():raise FormatError('IMU missing required fields')
                    if m.instId not in (0,1):continue
                    stamp=m.timestampUS*(1000 if m.instId==0 else 1);ros_time(stamp)
                    entries.append(Entry('imu',m.instId,stamp,stamp,offset,len(payload)));stats[f'imu/{m.instId}']['samples']+=1
                else:report['ignored_records']+=1
            except Exception as exc:raise FormatError(f'{name} at file offset {offset}: {exc}') from exc
    report['input']={key:dict(value) for key,value in stats.items()}
    for id,groups in cameras.items():report['input'][f'camera/{id}']['capture_stamps']=len(groups)
    missing=[f'{kind}/{id}' for kind,count in (('camera',8),('lidar',4),('imu',2)) for id in range(count) if f'{kind}/{id}' not in stats]
    report['missing_instances']=missing
    if missing:raise FormatError(f'missing expected sensors: {missing}')
    return entries,cameras

def decode_camera(path,id,groups,spool_dir,begin,end):
    try:import av
    except ImportError:
        # Optional reuse of the already installed local converter dependency.
        dep=Path.home()/'hubs2rosbag'/'.deps'
        if dep.is_dir():sys.path.append(str(dep))
        import av
    decoder=av.CodecContext.create('hevc','r');decoder.thread_count=2
    output=[];seen=set();errors=[];spool=Path(spool_dir)/f'camera_{id}.bgr'
    with open(path,'rb') as source,open(spool,'wb') as fd:
        def save(frames):
            for frame in frames:
                if frame.pts is None or frame.time_base is None:raise FormatError('decoded frame lost source PTS')
                ns=frame.pts*frame.time_base*1000000000
                if ns.denominator!=1:raise FormatError('non-integer decoded time')
                stamp=int(ns)
                if stamp not in groups or stamp in seen:raise FormatError('unknown/duplicate decoded PTS')
                seen.add(stamp)
                expected=groups[stamp][0]
                if (frame.width,frame.height)!=(expected.width,expected.height):raise FormatError('decoded image dimension mismatch')
                if stamp<begin or (end is not None and stamp>=end):continue
                data=frame.to_ndarray(format='bgr24').tobytes();pos=fd.tell();fd.write(data)
                output.append(Entry('camera',id,stamp,stamp,pos,len(data),frame.width,frame.height,str(spool)))
        for stamp,refs in sorted(groups.items()):
            if end is not None and stamp>=end:break
            chunks=[]
            for ref in refs:source.seek(ref.offset);chunks.append(read_exact(source,ref.length,'H.265'))
            packet=av.Packet(b''.join(chunks));packet.pts=stamp;packet.time_base=Fraction(1,1000000000)
            try:save(decoder.decode(packet))
            except av.error.FFmpegError as exc:errors.append({'stamp':stamp,'reason':str(exc)})
        save(decoder.decode(None))
    undecoded=[s for s in groups if s>=begin and (end is None or s<end) and s not in seen]
    return output,{'decoded':len(output),'errors':errors,'undecoded_stamps':undecoded}

def imu_message(payload,stamp,id,seq,gyro_units):
    d=ImuProto.FromString(payload);a=[d.accX,d.accY,d.accZ];w=[d.gyrX,d.gyrY,d.gyrZ]
    if not all(map(math.isfinite,a+w)):raise FormatError('non-finite IMU sample')
    msg=Imu(header=Header(seq=seq,stamp=ros_time(stamp),frame_id=f'imu_{id}'));msg.orientation.w=1;msg.orientation_covariance[0]=-1
    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=a
    scale=math.pi/180 if gyro_units[id]=='deg/s' else 1
    msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=[v*scale for v in w]
    return msg

def convert(args):
    output=Path(args.output);report={'input_file':str(Path(args.input).resolve()),'status':'indexing','records':0,'ignored_records':0,
        'units':{'camera_timestamp':'us','lidar_0_timestamp_and_offset':'ns','lidar_1_2_3_timestamp_and_offset':'us','imu_0_timestamp':'us','imu_1_timestamp':'ns','coordinates':'cm -> m','output_time':'ns','output_gyro':'rad/s'},
        'gyro_source_units':[args.imu0_gyro_unit,args.imu1_gyro_unit], 'calibration':'not applied'}
    report_path=output.with_suffix('.report.json');output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists() or report_path.exists():raise FileExistsError('choose a new output path; existing data is never overwritten')
    partial=Path(str(output)+'.partial');bag=None
    if partial.exists():raise FileExistsError(f'unfinished output already exists: {partial}; choose a new output path')
    try:
        entries,cameras=index(args.input,report);origin=min([e.stamp for e in entries]+[s for g in cameras.values() for s in g]);begin=origin+int(args.start*1e9);end=None if args.duration is None else begin+int(args.duration*1e9)
        entries=[e for e in entries if e.stamp>=begin and (end is None or e.stamp<end)]
        print('Indexed all 14 sensors; decoding cameras',flush=True)
        with tempfile.TemporaryDirectory(prefix='rig_decode_',dir=args.temp_dir) as temp:
            with ThreadPoolExecutor(max_workers=args.workers) as pool:
                futures={id:pool.submit(decode_camera,args.input,id,groups,temp,begin,end) for id,groups in cameras.items()}
                report['cameras']={}
                for id,future in futures.items():
                    decoded,stats=future.result();entries.extend(decoded);report['cameras'][str(id)]=stats;print(f'Camera {id}: {len(decoded)} frames',flush=True)
            counts=Counter();ranges={}
            if not entries:raise FormatError('no sensor messages in the selected interval')
            bag=rosbag.Bag(str(partial),'w',compression=args.compression)
            with open(args.input,'rb') as source:
                spools={id:open(Path(temp)/f'camera_{id}.bgr','rb') for id in cameras}
                try:
                    for e in sorted(entries,key=lambda e:(e.available,0 if e.kind=='imu' else 1 if e.kind=='lidar' else 2,e.id,e.offset)):
                        fd=spools[e.id] if e.kind=='camera' else source;fd.seek(e.offset);p=read_exact(fd,e.length,'indexed payload');key=f'{e.kind}/{e.id}';seq=counts[key]
                        if e.kind=='camera':msg=Image(header=Header(seq=seq,stamp=ros_time(e.stamp),frame_id=f'camera_{e.id}'),height=e.height,width=e.width,encoding='bgr8',is_bigendian=False,step=e.width*3,data=p);topic=f'/rig/camera/{e.id}/image_raw'
                        elif e.kind=='imu':msg=imu_message(p,e.stamp,e.id,seq,report['gyro_source_units']);topic=f'/rig/imu/{e.id}/data_raw'
                        else:
                            h,points,times=lidar_payload(p,e.id);xyz=np.column_stack([points[k] for k in ('x','y','z')]).astype(np.float32)*0.01
                            msg=normalized_cloud(xyz,points['ref'],times-e.stamp,points['ring'],e.stamp,e.id,seq);topic=f'/rig/lidar/{e.id}/points'
                        bag.write(topic,msg,ros_time(e.available));counts[key]+=1
                        if key not in ranges:ranges[key]=[e.stamp,e.stamp]
                        ranges[key][1]=e.stamp
                finally:
                    for fd in spools.values():fd.close()
            bag.close();bag=None;partial.rename(output)
            report.update(status='complete',output_bags=[str(output.resolve())],output_counts=dict(counts),output_ranges_ns=ranges)
    except Exception as exc:
        report.update(status='failed',error=str(exc));raise
    finally:
        if bag is not None:bag.close()
        report_path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['output_counts'],indent=2),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('input');p.add_argument('-o','--output',required=True)
    p.add_argument('--start',type=float,default=0);p.add_argument('--duration',type=float);p.add_argument('--workers',type=int,default=2)
    p.add_argument('--temp-dir');p.add_argument('--compression',choices=['lz4','none','bz2'],default='lz4')
    for id in (0,1):p.add_argument(f'--imu{id}-gyro-unit',choices=['deg/s','rad/s'],default='deg/s')
    args=p.parse_args()
    if args.start<0 or (args.duration is not None and args.duration<=0) or not 1<=args.workers<=8:p.error('invalid interval/workers')
    convert(args)
if __name__=='__main__':main()
