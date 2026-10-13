#!/usr/bin/env python3
"""Exercise the actual ROS subscriptions/publications on an isolated local master."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',required=True);p.add_argument('--config',required=True);p.add_argument('--bag',required=True);p.add_argument('--output',required=True);p.add_argument('--port',type=int,default=11317);a=p.parse_args()
    root=Path(a.output).resolve();root.mkdir(parents=True,exist_ok=True);env=os.environ.copy()
    env.update(ROS_MASTER_URI=f'http://127.0.0.1:{a.port}',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_HOME=str(root/'ros_home'),ROS_LOG_DIR=str(root/'ros_logs'),ROS_PACKAGE_PATH='/opt/ros/noetic/share')
    os.environ.update({k:v for k,v in env.items() if k.startswith('ROS_')});processes=[];logs=[]
    socket.setdefaulttimeout(0.5)
    try:
        def spawn(command,name):
            log=open(root/(name+'.log'),'w');logs.append(log);process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(process);return process
        master=spawn(['/usr/bin/python3','/opt/ros/noetic/bin/roscore','-p',str(a.port)],'master')
        proxy=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']);deadline=time.monotonic()+15
        while True:
            if master.poll() is not None:raise RuntimeError('local ROS master failed; see master.log')
            try:
                if proxy.getPid('/rig_transport_test')[0]==1:break
            except (OSError,xmlrpc.client.Error):pass
            if time.monotonic()>deadline:raise RuntimeError('ROS master startup timeout')
            time.sleep(0.05)
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        rospy.init_node('rig_transport_test',anonymous=True,disable_signals=True)
        rospy.set_param('/use_sim_time',True);counts={'odometry':0,'nonempty_maps':0}
        def odometry(msg):
            counts['odometry']+=1
        def cloud(msg):
            if msg.width:counts['nonempty_maps']+=1
        subscriptions=[rospy.Subscriber('/rig/odometry',Odometry,odometry),rospy.Subscriber('/rig/local_map',PointCloud2,cloud)]
        node=spawn([str(Path(a.binary).resolve()),'--config',str(Path(a.config).resolve()),'--output',str(root/'results')],'node')
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if node.poll() is not None:raise RuntimeError('runtime failed to start; see node.log')
            code,_,state=proxy.getSystemState('/rig_transport_test')
            if code==1 and sum(topic.startswith('/rig/') for topic,_ in state[1])>=14:break
            time.sleep(0.1)
        else:raise RuntimeError('not all 14 ROS subscriptions connected')
        play=spawn(['/usr/bin/python3','/opt/ros/noetic/bin/rosbag','play','--clock','--rate','0.2',str(Path(a.bag).resolve())],'play')
        play.wait(timeout=40);time.sleep(2)
        if node.poll() is not None:raise RuntimeError('runtime exited during ROS playback')
        assert counts['odometry']>=10 and counts['nonempty_maps']>0,counts
        os.killpg(node.pid,signal.SIGINT);node.wait(timeout=10)
        assert node.returncode==0,'runtime failed during shutdown'
        import numpy as np
        import rosbag
        import yaml
        saved=json.loads((root/'results/summary.json').read_text())
        trajectory=np.atleast_2d(np.loadtxt(root/'results/trajectory.txt'))
        with rosbag.Bag(a.bag) as source:
            last_imu=max(m.header.stamp.to_sec() for topic,m,_ in source.read_messages() if '/imu/' in topic)
        period=yaml.safe_load(Path(a.config).read_text()).get('processing',{}).get('update_period',0.1)
        assert last_imu-trajectory[-1,0]<period-1e-5,'final received batch not saved'
        report={'status':'passed','subscriptions':14,'saved_odometry':saved['updates'],'final_batch_saved':True,**counts};(root/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        for log in logs:log.close()
if __name__=='__main__':main()
