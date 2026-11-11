"""Run only the simulated robot in a dedicated ROS domain and Gazebo master."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET


def run(workspace, output, probe_binary, *, mode='fresh', map_yaml=None):
    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import Twist
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from pm01_navigation_client import Navigator
    profile=ET.parse(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'])
    ns={'f':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    addresses=[e.text for e in profile.findall('.//f:interfaceWhiteList/f:address',ns)]
    if addresses!=['127.0.0.1'] or int(os.environ.get('ROS_DOMAIN_ID','0')) == 0:
        raise RuntimeError('Use a dedicated non-default domain and loopback-only Fast DDS profile')
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    result={};process=probe=node=None
    log=(output/'launch.log').open('w')
    try:
        args=['ros2','launch','pm01_navigation_bringup','navigation.launch.py',
              'profile:=sim','mode:='+mode,'gui:=false','rviz:=false',
              'gazebo_master_uri:='+os.environ['GAZEBO_MASTER_URI'],
              'save_root:='+str(output/'maps')]
        if map_yaml:args.append('map:='+str(map_yaml))
        process=subprocess.Popen(args,cwd=workspace,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        owner_file=output/'tf-owners.txt'
        probe=subprocess.Popen([str(probe_binary),str(owner_file)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=Navigator('pm01_sim_workflow_test')
        counts={'map':0,'navigation_map':0,'scan':0}
        commands={}
        result['commands']=commands
        result['policy_targets']={'count':0}
        def policy_targets(msg):
            result['policy_targets']['count']+=1
            result['policy_targets']['last']=list(msg.data)
        node.create_subscription(Float64MultiArray,'/pm01_sdk_pd_controller/commands',policy_targets,10)
        def command(topic, msg):
            values=[msg.linear.x,msg.linear.y,msg.angular.z]
            entry=commands.setdefault(topic,dict(count=0,max_abs=[0.,0.,0.]))
            entry['count']+=1;entry['last']=values
            entry['max_abs']=[max(a,abs(b)) for a,b in zip(entry['max_abs'],values)]
        for topic in ('/cmd_vel','/pm01/walking_cmd_vel'):
            node.create_subscription(Twist,topic,lambda msg,t=topic:command(t,msg),10)
        def motion_gate(msg):
            result['motion_gate']=[dict(name=s.name,message=s.message,values={v.key:v.value for v in s.values}) for s in msg.status]
        node.create_subscription(DiagnosticArray,'/pm01/navigation_motion_status',motion_gate,10)
        def seen(name):
            def callback(msg):counts[name]+=1
            return callback
        for name in ('map','navigation_map'):
            node.create_subscription(OccupancyGrid,'/'+name,seen(name),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(LaserScan,'/scan',seen('scan'),qos_profile_sensor_data)
        clients={name:node.create_client(GetState,'/'+name+'/get_state')
                 for name in ('bt_navigator','controller_server','planner_server')}
        states={};pending={};sent={};initialized=False
        def poll():
            nonlocal initialized
            node.poll(.1)
            if process.poll() is not None:raise RuntimeError('Simulation launch exited')
            if mode=='saved' and not initialized and counts['scan']>3 and node.initial_pub.get_subscription_count():
                node.set_initial_pose(0.,0.,0.);initialized=True
            for name,client in clients.items():
                if name in pending and pending[name].done():
                    states[name]=pending.pop(name).result().current_state.id
                if name in pending and time.monotonic()-sent[name]>2.:
                    client.remove_pending_request(pending.pop(name))
                if name not in pending and client.service_is_ready() and time.monotonic()-sent.get(name,0)>.5:
                    pending[name]=client.call_async(GetState.Request());sent[name]=time.monotonic()
        deadline=time.monotonic()+140
        while time.monotonic()<deadline:
            poll()
            if (counts['navigation_map']>=10 and node.motion and node.localization.get('matched') is True
                    and all(states.get(n)==3 for n in clients)):break
        result.update(counts=counts,lifecycle_states=states,initial_status=node.get_status())
        assert counts['navigation_map']>=10 and counts['scan']>=3
        assert all(states.get(n)==3 for n in clients),states
        assert node.motion['state']=='disabled' and node.count_publishers('/pm01/walking_cmd_vel')==0
        assert node.localization.get('matched') is True
        def response(future):
            deadline=time.monotonic()+15
            while not future.done() and time.monotonic()<deadline:poll()
            value=future.result();assert value and value.success,value
            return value
        result['saved_map']=response(node.save_map()).message
        assert Path(result['saved_map']).is_file()
        response(node.enable_motion())
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:poll()
        result['enabled_publishers']=node.count_publishers('/pm01/walking_cmd_vel')
        assert result['enabled_publishers']==1
        pose=node.get_status()['tf']['map->base_footprint'];x,y,_=pose['xyz_m'];yaw=pose['yaw_rad']
        task=node.navigate_to_pose(x+.6*math.cos(yaw),y+.6*math.sin(yaw),yaw)
        node.wait(task,70.);result['goal_state']=task.state
        result['after_goal']=node.get_status()['tf']['map->base_footprint']
        result['after_goal_status']=node.get_status()
        assert task.state=='succeeded',task.error or task.state
        next_pose=node.get_status()['tf']['map->base_footprint']
        nx,ny,_=next_pose['xyz_m'];next_yaw=next_pose['yaw_rad']
        task=node.navigate_to_pose(nx+.4*math.cos(next_yaw),ny+.4*math.sin(next_yaw),next_yaw)
        node.wait(task,60.);result['second_goal_state']=task.state
        assert task.state=='succeeded',task.error or task.state
        task=node.navigate_to_pose(x+2*math.cos(yaw),y+2*math.sin(yaw),yaw)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:poll()
        node.cancel(task);node.wait(task,15.);result['cancel_state']=task.state
        assert task.state=='canceled'
        response(node.release_motion())
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:poll()
        result['released_publishers']=node.count_publishers('/pm01/walking_cmd_vel')
        assert result['released_publishers']==0
        result['tf_publishers']=len(set(owner_file.read_text().splitlines()))
        assert result['tf_publishers']==1
        result['passed']=True
        return result
    except Exception as exc:
        result.update(passed=False,error=repr(exc));raise
    finally:
        # Only terminate process groups created by this scenario.
        if process:
            for sig,timeout in ((signal.SIGINT,30),(signal.SIGTERM,10),(signal.SIGKILL,5)):
                if process.poll() is not None:break
                try:os.killpg(process.pid,sig)
                except ProcessLookupError:pass
                try:
                    process.wait(timeout)
                    break
                except subprocess.TimeoutExpired:pass
        if probe:probe.terminate();probe.wait(5)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        log.close();(output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
