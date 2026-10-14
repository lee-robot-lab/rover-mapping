#!/usr/bin/env python3
# ================================================================
# mapping_probe.py
# 설명: 원본 stereo·CameraInfo·metadata와 host 성능을 모터 통신과 별도로 측정한다.
# 사용법: mapping_probe.py --run-dir RUN --left /left --right /right --left-info /left/camera_info --right-info /right/camera_info
# ================================================================
import argparse
from collections import deque
import json
import os
from pathlib import Path
import shutil
import time
from prepare_mapping_run import atomic_json


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--run-dir',type=Path,required=True)
    for name in ('left','right','left-info','right-info'): parser.add_argument('--'+name,required=True)
    parser.add_argument('--metadata-topics',nargs='*',default=[])
    parser.add_argument('--duration',type=float,default=30.)
    args=parser.parse_args()
    if args.duration<=0: parser.error('duration must be positive')
    args.run_dir.mkdir(parents=True,exist_ok=True)
    if (args.run_dir/'probe.json').exists(): parser.error('probe output exists; use new directory')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo,Image
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    rclpy.init(); node=rclpy.create_node('mapping_input_probe')
    stats={s:{'count':0,'non_increasing':0,'gaps_over_50ms':0,'encoding':None,'width':None,'height':None,
              'first_source_ns':None,'last_source_ns':None,'max_age_s':None,'min_age_s':None,'last_age_s':None,
              'profile_changes':0} for s in ('left','right')}
    stamps={s:deque(maxlen=1000) for s in stats}; infos={}; metadata={}; subscriptions=[]
    delta_samples=[]; started=time.monotonic(); next_system=started; system=[]
    def image_callback(side: str, message: Image) -> None:
        entry=stats[side]; stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if entry['count'] and (entry['width'],entry['height'],entry['encoding'])!=(message.width,message.height,message.encoding):
            entry['profile_changes']+=1
        previous=entry['last_source_ns']; entry['count']+=1
        if previous is not None:
            entry['non_increasing']+=int(stamp<=previous)
            entry['gaps_over_50ms']+=int(stamp-previous>50_000_000)
        else: entry['first_source_ns']=stamp
        entry.update(last_source_ns=stamp,width=message.width,height=message.height,encoding=message.encoding,
                     frame_id=message.header.frame_id,step=message.step)
        age=(node.get_clock().now().nanoseconds-stamp)*1e-9
        entry['last_age_s']=age; entry['max_age_s']=age if entry['max_age_s'] is None else max(age,entry['max_age_s'])
        entry['min_age_s']=age if entry['min_age_s'] is None else min(age,entry['min_age_s'])
        stamps[side].append(stamp)
        if side=='right' and stamps['left']:
            delta_samples.append(min(abs(stamp-t) for t in stamps['left']))
    for side in stats:
        subscriptions.append(node.create_subscription(Image,getattr(args,side),lambda msg,s=side:image_callback(s,msg),qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(CameraInfo,getattr(args,side+'_info'),lambda msg,s=side:infos.update({s:dict(message_to_ordereddict(msg))}),qos_profile_sensor_data))
    try:
        while rclpy.ok() and time.monotonic()-started<args.duration:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()>=next_system:
                types=dict(node.get_topic_names_and_types())
                for topic in args.metadata_topics:
                    if topic in types and topic not in metadata:
                        metadata[topic]={'type':types[topic][0],'last':None}
                        subscriptions.append(node.create_subscription(get_message(types[topic][0]),topic,
                            lambda msg,t=topic:metadata[t].update(last=dict(message_to_ordereddict(msg))),qos_profile_sensor_data))
                item={'host_ns':time.time_ns(),'loadavg':list(os.getloadavg()),'disk_free_bytes':shutil.disk_usage(args.run_dir).free,
                      'proc_stat':Path('/proc/stat').read_text().splitlines()[0],
                      'rss_bytes':int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE'),
                      'temperatures_mC':{p.parent.name:p.read_text().strip() for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')}}
                system.append(item); next_system=time.monotonic()+1
    finally:
        elapsed=time.monotonic()-started
        for entry in stats.values(): entry['receipt_rate_hz']=entry['count']/elapsed
        counts={'left':stats['left']['count'],'right':stats['right']['count']}
        atomic_json(args.run_dir/'probe.json',{'status':'captured' if min(counts.values())>0 and len(infos)==2 else 'incomplete',
                    'stats':stats,'camera_info':infos,'metadata':metadata,'elapsed_s':elapsed,
                    'nearest_delta_max_ns':max(delta_samples) if delta_samples else None,
                    'exact_pair_candidates':len(set(stamps['left']).intersection(stamps['right'])),
                    'pair_window_note':'last 1000 per stream; source stamps unchanged', 'system':system,
                    'clock':{'use_sim_time':node.get_parameter('use_sim_time').value,'age_source':'ros_now_minus_source_stamp'},
                    'imu_clock_verified':False})
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
