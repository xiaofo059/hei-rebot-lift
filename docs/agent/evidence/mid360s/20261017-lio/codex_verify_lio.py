import os,signal,subprocess,time,json,math,pathlib,hashlib
import rclpy,numpy as np
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
R=pathlib.Path('/home/robo/robotopia/hei-rebot-lift');D=pathlib.Path('/home/robo/hei-data/mid360s_lio_20261017/codex_smoke');D.mkdir(exist_ok=True)
paths=[R/'extensions/ros2_ws/src/hei_robot_localization/hei_robot_localization/lio_input_audit.py',R/'extensions/ros2_ws/src/hei_robot_localization/launch/fast_lio_mid360s_offline.launch.py',R/'extensions/third_party/fast_lio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping']
manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
def reset():
 signal.signal(signal.SIGINT,signal.SIG_DFL);signal.signal(signal.SIGTERM,signal.SIG_DFL)
rclpy.init();node=rclpy.create_node('codex_lio_verifier');q=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST,depth=300)
od=[];clouds={};tfs=set()
def onodom(m):
 p=m.pose.pose.position;o=m.pose.pose.orientation
 od.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'frame':m.header.frame_id,'child':m.child_frame_id,'p':[p.x,p.y,p.z],'q':[o.x,o.y,o.z,o.w]})
def oncloud(topic,m):
 st=clouds.setdefault(topic,{'count':0,'points':0,'bad_xyz':0,'frames':set(),'stamps':[]})
 st['count']+=1;st['frames'].add(m.header.frame_id);st['points']+=m.width*m.height;st['stamps'].append(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
 fields={f.name:f for f in m.fields};names=['x','y','z'];assert all(fields[n].datatype==7 for n in names)
 dt=np.dtype({'names':names,'formats':['>f4' if m.is_bigendian else '<f4']*3,'offsets':[fields[n].offset for n in names],'itemsize':m.point_step})
 for row in range(m.height):
  arr=np.frombuffer(m.data,dtype=dt,count=m.width,offset=row*m.row_step)
  st['bad_xyz']+=int(np.count_nonzero(~(np.isfinite(arr['x'])&np.isfinite(arr['y'])&np.isfinite(arr['z']))))
node.create_subscription(Odometry,'/Odometry',onodom,q)
for topic in ['/cloud_registered','/cloud_registered_body']:
 node.create_subscription(PointCloud2,topic,lambda m,t=topic:oncloud(t,m),q)
node.create_subscription(TFMessage,'/tf',lambda m:tfs.update((t.header.frame_id,t.child_frame_id) for t in m.transforms),q)
cmd=['ros2','launch','hei_robot_localization','fast_lio_mid360s_offline.launch.py','bag_path:=/home/robo/hei-data/mid360s_live_20261017_retest01','uncalibrated:=true','rate:=2.0','audit_seconds:=6.0','audit_report:='+str(D/'audit.json')]
log=open(D/'launch.log','w');env=dict(os.environ,ROS_LOG_DIR=str(D/'ros_log'))
p=subprocess.Popen(cmd,cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=reset)
status=pathlib.Path(f'/proc/{p.pid}/status').read_text();ignored=int(next(l.split()[1] for l in status.splitlines() if l.startswith('SigIgn:')),16)&2
escalated=False
try:
 end=time.monotonic()+27
 while time.monotonic()<end:
  assert p.poll() is None,'launch exited before verification window'
  rclpy.spin_once(node,timeout_sec=.1)
finally:
 if p.poll() is None:
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=12)
  except subprocess.TimeoutExpired:
   escalated=True;os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
 log.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
text=(D/'launch.log').read_text()
mono=lambda a:bool(a) and all(y>x for x,y in zip(a,a[1:]))
for st in clouds.values():st['frames']=sorted(st['frames']);st['stamp_monotonic']=mono(st.pop('stamps'))
finite=bool(od) and all(math.isfinite(x) for o in od for x in o['p']+o['q'])
result={'kind':'independent laptop offline nominal-extrinsic smoke','launch_exit':p.returncode,'sigint_ignored_at_start':bool(ignored),'stop_escalated':escalated,'odom_count':len(od),'odom_all_finite':finite,'odom_stamp_monotonic':mono([o['stamp'] for o in od]),'odom_span_s':od[-1]['stamp']-od[0]['stamp'] if od else None,'odom_frames':sorted({(o['frame'],o['child']) for o in od}),'max_abs_position_m':max((abs(v) for o in od for v in o['p']),default=None),'quaternion_norm_range':[min(math.sqrt(sum(v*v for v in o['q'])) for o in od),max(math.sqrt(sum(v*v for v in o['q'])) for o in od)] if od else None,'clouds':clouds,'tf_pairs':sorted(tfs),'child_clean_exit_count':text.count('process has finished cleanly'),'child_died_log':'process has died' in text,'runtime_hashes_before':manifest,'runtime_hashes_unchanged':all(hashlib.sha256(pathlib.Path(k).read_bytes()).hexdigest()==v for k,v in manifest.items()),'limits':'Only stationary bag/nominal sensor extrinsics; not motion accuracy, calibrated robot TF, hardware sync or full SLAM.'}
result['verdict']='PASS' if p.returncode==0 and not escalated and not ignored and finite and result['odom_stamp_monotonic'] and result['runtime_hashes_unchanged'] and len(clouds)==2 and all(st['count']>0 and st['points']>0 and st['bad_xyz']==0 and st['stamp_monotonic'] for st in clouds.values()) and not result['child_died_log'] and result['child_clean_exit_count']==3 and ('camera_init','body') in tfs else 'FAIL'
(D/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(0 if result['verdict']=='PASS' else 1)
