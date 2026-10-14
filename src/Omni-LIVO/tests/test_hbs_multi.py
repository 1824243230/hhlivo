import io
from contextlib import redirect_stdout
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import hbs_multi
from hbs_multi import Entry,FormatError,records,lidar_payload,normalized_cloud,imu_message,ImuProto
from verify_rig_bag import verify

class ConverterTests(unittest.TestCase):
    def make_hbs(self,path):
        stamp=1700000000000000000
        payloads=[]
        for id in range(8):
            payload=hbs_multi.CAM_HEAD.pack(0,0,id,0,1,1,1,stamp//1000,0)+b'\0'
            payloads.append(('raw.itf_camera_h265',payload))
        for id in range(4):
            physical=stamp//(1 if id==0 else 1000)
            payload=hbs_multi.LIDAR_HEAD.pack(0,id,physical>>32,0,physical&0xffffffff,0,0,0,1,0,0,0,0)
            payload+=struct.pack('<hhhHHHBBH',100,200,300,0,0,100,50,1,0)
            payloads.append(('raw.itf_lidar_point_mix',payload))
        for id in range(2):
            msg=ImuProto()
            for field in msg.DESCRIPTOR.fields:
                if field.label==field.LABEL_REQUIRED:setattr(msg,field.name,0)
            msg.instId=id;msg.timestampUS=stamp//(1000 if id==0 else 1);msg.accZ=9.81
            payloads.append(('svc.itf.imu.ItfImuData',msg.SerializeToString()))
        with path.open('wb') as fd:
            fd.write(struct.pack('<II',0xC7C7C7C7,1)+bytes(80))
            fd.write(struct.pack('<QI256sI',17,1,b'x',0)+b'\xb6'*4)
            for name,payload in payloads:
                name=name.encode()
                record=bytes(28)+struct.pack('<H',1)+b'x'+struct.pack('<H',len(name))+name+struct.pack('<I',len(payload))+payload
                fd.write(struct.pack('<I',len(record))+record)

    @staticmethod
    def decoded_camera(path,id,groups,spool_dir,begin,end):
        spool=Path(spool_dir)/f'camera_{id}.bgr';spool.write_bytes(b'\x10\x20\x30')
        stamp=next(iter(groups))
        entries=[] if stamp<begin or (end is not None and stamp>=end) else [Entry('camera',id,stamp,stamp,0,3,1,1,str(spool))]
        return entries,{'decoded':len(entries),'errors':[],'undecoded_stamps':[]}

    def run_converter(self,source,output,extra_args=()):
        # Exercise the exact CLI without decoding the fixture's placeholder H.265.
        with patch.object(sys,'argv',['hbs_multi.py',str(source),'--output',str(output),*extra_args]), \
             patch.object(hbs_multi,'decode_camera',side_effect=self.decoded_camera),redirect_stdout(io.StringIO()):
            hbs_multi.main()

    def test_cli_writes_one_readable_bag_with_all_sensors(self):
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'input.hbs';output=Path(temp)/'rig.bag';self.make_hbs(source)
            self.run_converter(source,output)
            self.assertEqual(list(Path(temp).glob('*.bag')),[output])
            self.assertFalse(Path(str(output)+'.partial').exists())
            report=json.loads(output.with_suffix('.report.json').read_text())
            self.assertEqual(report['status'],'complete')
            self.assertEqual(report['output_bags'],[str(output.resolve())])
            validation=verify([str(output)])
            self.assertEqual(validation['status'],'passed')
            self.assertEqual(len(validation['counts']),14)
            self.assertTrue(all(count==1 for count in validation['counts'].values()))

    def test_output_stays_single_beyond_old_split_threshold(self):
        class LargePayload:
            def __len__(self):return 3*1024**3
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'input.hbs';output=Path(temp)/'rig.bag';self.make_hbs(source)
            bag=Mock()
            def open_bag(path,*args,**kwargs):
                Path(path).touch();return bag
            with patch.object(hbs_multi.rosbag,'Bag',side_effect=open_bag) as writer, \
                 patch.object(hbs_multi,'Image',return_value=Mock(data=LargePayload())):
                self.run_converter(source,output)
            self.assertEqual(writer.call_count,1)
            self.assertEqual(bag.write.call_count,14)
            self.assertEqual(bag.close.call_count,1)
            self.assertEqual(list(Path(temp).glob('*.bag')),[output])

    def test_existing_output_artifacts_are_preserved(self):
        for suffix in ('.bag','.report.json','.bag.partial'):
            with self.subTest(suffix=suffix),tempfile.TemporaryDirectory() as temp:
                output=Path(temp)/'rig.bag';existing=Path(temp)/('rig'+suffix);existing.write_bytes(b'keep')
                with self.assertRaises(FileExistsError):self.run_converter(Path(temp)/'input.hbs',output)
                self.assertEqual(existing.read_bytes(),b'keep')

    def test_failed_conversion_does_not_publish_final_bag(self):
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'input.hbs';output=Path(temp)/'rig.bag';self.make_hbs(source)
            with patch.object(hbs_multi,'imu_message',side_effect=RuntimeError('conversion interrupted')):
                with self.assertRaisesRegex(RuntimeError,'conversion interrupted'):self.run_converter(source,output)
            self.assertFalse(output.exists())
            self.assertTrue(Path(str(output)+'.partial').exists())
            report=json.loads(output.with_suffix('.report.json').read_text())
            self.assertEqual(report['status'],'failed')
            self.assertEqual(report['error'],'conversion interrupted')

    def test_empty_interval_is_reported_without_a_bag(self):
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'input.hbs';output=Path(temp)/'rig.bag';self.make_hbs(source)
            with self.assertRaisesRegex(FormatError,'no sensor messages'):
                self.run_converter(source,output,['--start','3600'])
            self.assertFalse(output.exists())
            self.assertFalse(Path(str(output)+'.partial').exists())
            report=json.loads(output.with_suffix('.report.json').read_text())
            self.assertEqual(report['status'],'failed')

    def test_truncated_and_invalid_input(self):
        with tempfile.NamedTemporaryFile() as f:
            f.write(b'bad');f.flush()
            with open(f.name,'rb') as source:
                with self.assertRaises(FormatError):list(records(source))
    def test_lidar_units_and_signed_offsets(self):
        for id,factor in [(0,1),(1,1000),(2,1000),(3,1000)]:
            physical=1700000000000000000//factor
            header=struct.pack('<Q8I4B',0,id,physical>>32,0,physical&0xffffffff,0,0,0,2,0,0,0,0)
            chunks=[]
            for offset in (-100,100):
                raw=offset&0xffffffff;chunks.append(struct.pack('<hhhHHHBBH',100,200,300,0,raw>>16,raw&0xffff,50,1,0))
            h,p,t=lidar_payload(header+b''.join(chunks),id)
            self.assertEqual(int(t[1]-t[0]),200*factor)
            stamp=int(t.min());msg=normalized_cloud(np.column_stack([p[k] for k in ('x','y','z')])*0.01,p['ref'],t-stamp,p['ring'],stamp,id)
            offsets=np.ndarray((2,),dtype='<u4',buffer=msg.data,offset=16,strides=(24,))
            self.assertEqual(list(offsets),[0,200*factor]);self.assertEqual(msg.header.stamp.to_nsec(),stamp)
    def test_imu_instance_and_unit_are_separate(self):
        for id,unit in [(0,'deg/s'),(1,'rad/s')]:
            m=ImuProto()
            for field in m.DESCRIPTOR.fields:
                if field.label==field.LABEL_REQUIRED:setattr(m,field.name,0)
            m.instId=id;m.gyrX=180;m.accZ=9.81
            out=imu_message(m.SerializeToString(),1700000000000000000,id,0,['deg/s','rad/s'])
            self.assertAlmostEqual(out.angular_velocity.x,np.pi if id==0 else 180)
            self.assertEqual(out.header.frame_id,f'imu_{id}')
    def test_offset_overflow_rejected(self):
        with self.assertRaises(FormatError):normalized_cloud(np.zeros((1,3)),[0],[2**32],[0],1700000000000000000,0)
if __name__=='__main__':unittest.main()
