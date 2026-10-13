import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from hbs_multi import FormatError,records,lidar_payload,normalized_cloud,imu_message,ImuProto

class ConverterTests(unittest.TestCase):
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
