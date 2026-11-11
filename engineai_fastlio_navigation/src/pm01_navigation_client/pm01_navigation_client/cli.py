"""Command-line examples of the public ROS2 interfaces."""
import argparse
import json
import time
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import math
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID
from .navigator import Navigator


def main(args=None):
    parser=argparse.ArgumentParser(description='PM01 standard ROS2 navigation client; yaw in radians')
    sub=parser.add_subparsers(dest='command',required=True)
    for name in ('goal','initial-pose','plan'):
        p=sub.add_parser(name);p.add_argument('x',type=float);p.add_argument('y',type=float);p.add_argument('yaw',type=float,nargs='?',default=0.)
    for name in ('enable-motion','release-motion','save-map','status','relocalize'):sub.add_parser(name)
    p=sub.add_parser('cancel');p.add_argument('--uuid',help='Goal UUID; omitted means all goals on this Nav2 instance')
    parser.add_argument('--timeout',type=float,default=120.)
    cli,rosargs=parser.parse_known_args(args)
    if not math.isfinite(cli.timeout) or cli.timeout <= 0:
        parser.error('--timeout must be a positive finite number of seconds')
    rclpy.init(args=rosargs);nav=Navigator()
    planning = None
    try:
        end=time.monotonic()+cli.timeout
        if cli.command=='plan':
            if not all(math.isfinite(v) for v in (cli.x,cli.y,cli.yaw)):
                raise ValueError('Planning coordinates must be finite SI values')
            planning = ActionClient(nav, ComputePathToPose, '/compute_path_to_pose')
            if not planning.wait_for_server(timeout_sec=min(10.,cli.timeout)):
                raise TimeoutError('ComputePathToPose server unavailable')
            request = ComputePathToPose.Goal()
            request.goal = PoseStamped()
            request.goal.header.frame_id = 'map'
            request.goal.header.stamp = nav.get_clock().now().to_msg()
            request.goal.pose.position.x, request.goal.pose.position.y = cli.x,cli.y
            request.goal.pose.orientation.z,request.goal.pose.orientation.w = math.sin(cli.yaw/2), math.cos(cli.yaw/2)
            future = planning.send_goal_async(request)
            while not future.done() and time.monotonic()<end: nav.poll(.05)
            if not future.done(): raise TimeoutError('Planning acceptance unknown; no navigation task sent')
            handle = future.result()
            if not handle.accepted: raise RuntimeError('Planning rejected')
            future = handle.get_result_async()
            while not future.done() and time.monotonic()<end: nav.poll(.05)
            if not future.done(): raise TimeoutError('Planning result unknown; no navigation task sent')
            response = future.result()
            if response.status != 4: raise RuntimeError('Planning failed: '+str(response.status))
            path = response.result.path
            print(json.dumps({'path_poses':len(path.poses),'frame':path.header.frame_id,'navigation_goal_sent':False}))
        elif cli.command=='goal':
            if not nav.wait_until_ready(min(30.,cli.timeout)):raise TimeoutError('Nav2/map/TF not ready; saved mode requires initial pose')
            t=nav.navigate_to_pose(cli.x,cli.y,cli.yaw);print('task_uuid='+t.uuid,flush=True)
            while not t.done and time.monotonic()<end:
                nav.poll(.1)
            print(json.dumps({'uuid':t.uuid,'state':t.state,'error':t.error}))
            if not t.done:raise TimeoutError('Goal outcome unknown; timeout did not cancel it. Use cancel --uuid.')
            if t.state not in ('succeeded','canceled'):raise RuntimeError(t.state)
        elif cli.command=='initial-pose':
            # Wait for discovery rather than publish-and-exit with no subscriber.
            while (nav.initial_pub.get_subscription_count()==0 or (nav.get_parameter('use_sim_time').value and nav.get_clock().now().nanoseconds==0)) and time.monotonic()<end:nav.poll(.1)
            if nav.initial_pub.get_subscription_count()==0:raise TimeoutError('No /initialpose subscriber; start saved mode')
            nav.set_initial_pose(cli.x,cli.y,cli.yaw)
            flush_end=time.monotonic()+.5
            while time.monotonic()<flush_end: nav.poll(.05)
            print('Initial pose published; inspect /pm01/localization_status and RViz')
        elif cli.command=='status':
            sample_end=time.monotonic()+min(5.,cli.timeout)
            while time.monotonic()<sample_end: nav.poll(.05)
            print(json.dumps(nav.get_status(),ensure_ascii=False,indent=2))
        else:
            if cli.command=='cancel':
                client=nav.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
                if not client.wait_for_service(timeout_sec=min(5.,cli.timeout)):raise TimeoutError('Cancel service unavailable')
                request=CancelGoal.Request()
                if cli.uuid:request.goal_info.goal_id=UUID(uuid=list(bytes.fromhex(cli.uuid.replace('-',''))))
                future=client.call_async(request)
            elif cli.command=='relocalize':
                if not nav.relocalize_client.wait_for_service(timeout_sec=min(5.,cli.timeout)):
                    raise TimeoutError('AMCL global relocalization service unavailable; start saved mode with slam_amcl')
                future=nav.relocalize()
            else:
                name=cli.command.replace('-','_')
                if not nav.trigger_clients[name].wait_for_service(timeout_sec=min(5.,cli.timeout)):raise TimeoutError(name+' unavailable')
                future=getattr(nav,name)()
            while not future.done() and time.monotonic()<end:nav.poll(.1)
            if not future.done():raise TimeoutError('Response unknown; no successful cancellation or release implied')
            response=future.result();print(str(response))
            if hasattr(response,'success') and not response.success:raise RuntimeError(response.message)
            if cli.command=='cancel':print('Cancel request acknowledged; check final action result/status to confirm completion')
            elif cli.command=='relocalize':print('Global localization requested; convergence is not confirmed. Inspect localization status and RViz.')
    except (TimeoutError, RuntimeError, ValueError) as error:
        parser.exit(1, str(error)+'\n')
    finally:
        if planning is not None: planning.destroy()
        nav.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
