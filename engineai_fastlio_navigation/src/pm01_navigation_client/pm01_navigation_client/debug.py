"""Bounded sampling, no bag recording or background daemon."""
import argparse
import json
import time
import rclpy
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from .navigator import Navigator

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=10.);args,rosargs=p.parse_known_args()
    if not 0<args.seconds<=120:p.error('--seconds must be in (0,120]')
    rclpy.init(args=rosargs);n=Navigator('pm01_bounded_debug');samples={};statuses={}
    start=time.monotonic()
    def observe(topic,msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        s=samples.setdefault(topic,{'count':0,'first':now,'last':now,'age_max_sec':None})
        s['count']+=1;s['last']=now
        ros_now=n.get_clock().now().nanoseconds*1e-9
        s['age_sec']=None if n.get_parameter('use_sim_time').value and ros_now==0 else ros_now-stamp
        if s['age_sec'] is not None:
            s['age_max_sec']=s['age_sec'] if s['age_max_sec'] is None else max(s['age_max_sec'],s['age_sec'])
    for topic,kind in [('/livox/lidar',None),('/livox/points',PointCloud2),('/livox/imu',Imu),('/Odometry',Odometry),('/odom',Odometry),('/navigation_map',OccupancyGrid)]:
        if kind:n.create_subscription(kind,topic,lambda m,t=topic:observe(t,m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if kind==OccupancyGrid else qos_profile_sensor_data)
    def diag(msg):
        for s in msg.status:statuses[s.name]={'message':s.message,'values':{v.key:v.value for v in s.values}}
    for t in ('/lio/status','/pm01/map_update_status','/pm01/localization_status','/pm01/navigation/motion_status'):
        n.create_subscription(DiagnosticArray,t,diag,10)
    try:
        while rclpy.ok() and time.monotonic()-start<args.seconds:n.poll(.05)
        for s in samples.values():
            s['hz']=(s['count']-1)/max(.001,s['last']-s['first']);s['receive_age_sec']=time.monotonic()-s['last']
        print(json.dumps({'duration_sec':time.monotonic()-start,'topics':samples,'diagnostics':statuses,'navigation':n.get_status()},ensure_ascii=False,indent=2))
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
