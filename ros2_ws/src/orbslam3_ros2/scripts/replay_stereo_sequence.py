#!/usr/bin/env python3
# ================================================================
# replay_stereo_sequence.py
# 설명: EuRoC cam0/cam1 CSV와 원본 영상만 ROS로 재생한다. GT·렌더 pose는 읽지 않는다.
# 사용법: replay_stereo_sequence.py --sequence SEQUENCE --max-pairs 900 --rate 1
# ================================================================
import argparse
from array import array
import csv
import time
from pathlib import Path
import cv2


def read_pairs(sequence: Path) -> list[tuple[int, Path, Path]]:
    streams = []
    for side in ('cam0', 'cam1'):
        root = sequence/'mav0'/side
        with (root/'data.csv').open() as stream:
            rows = [(int(row[0]), root/'data'/row[1].strip()) for row in csv.reader(stream)
                    if row and not row[0].lstrip().startswith('#')]
        if any(b[0] <= a[0] for a,b in zip(rows,rows[1:])):
            raise ValueError('non-increasing source image stamps')
        if any(not file.is_file() or file.resolve().parent != (root/'data').resolve() for _,file in rows):
            raise ValueError('missing or nonlocal image file')
        streams.append(rows)
    if not streams[0] or [t for t,_ in streams[0]] != [t for t,_ in streams[1]]:
        raise ValueError('source stereo stamps differ or sequence empty')
    return [(a[0],a[1],b[1]) for a,b in zip(*streams)]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--sequence', type=Path, required=True)
    parser.add_argument('--left-topic', default='/rover/stereo/left/image_raw')
    parser.add_argument('--right-topic', default='/rover/stereo/right/image_raw')
    parser.add_argument('--start-index',type=int,default=0)
    parser.add_argument('--max-pairs', type=int)
    parser.add_argument('--min-subscribers',type=int,default=1)
    parser.add_argument('--rate', type=float, default=1.)
    args = parser.parse_args()
    if args.rate <= 0 or (args.max_pairs is not None and args.max_pairs <= 0): parser.error('positive rate/max-pairs required')
    pairs = read_pairs(args.sequence)
    if not 0 <= args.start_index < len(pairs): parser.error('start-index outside sensor sequence')
    pairs=pairs[args.start_index:]
    if args.max_pairs: pairs = pairs[:args.max_pairs]
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    rclpy.init(); node = rclpy.create_node('stereo_sensor_replay')
    publishers = [node.create_publisher(Image,t,qos_profile_sensor_data) for t in (args.left_topic,args.right_topic)]
    clock = node.create_publisher(Clock,'/clock',10)
    # Bounded discovery wait only; never relabel sensor timestamps.
    deadline = time.monotonic()+10
    while rclpy.ok() and any(p.get_subscription_count()<args.min_subscribers for p in publishers) and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    if any(p.get_subscription_count()<args.min_subscribers for p in publishers):
        raise RuntimeError('stereo subscribers not ready before discovery deadline')
    started = time.monotonic(); sent=0
    try:
        for stamp,left,right in pairs:
            target = started + (stamp-pairs[0][0])*1e-9/args.rate
            while rclpy.ok() and time.monotonic() < target:
                rclpy.spin_once(node,timeout_sec=min(.02,target-time.monotonic()))
            if not rclpy.ok(): break
            tick = Clock(); tick.clock.sec,tick.clock.nanosec = divmod(stamp,10**9); clock.publish(tick)
            for publisher,path,frame in zip(publishers,(left,right),('camera_infra1_optical_frame','camera_infra2_optical_frame')):
                image = cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
                if image is None: raise ValueError(f'unreadable sensor image: {path}')
                message=Image(); message.header.stamp=tick.clock; message.header.frame_id=frame
                message.height,message.width=image.shape; message.encoding='mono8'; message.step=message.width
                # ROS uint8[] accepts array directly; bytes triggers a slow per-pixel Python conversion.
                message.data=array('B',image.tobytes()); publisher.publish(message)
            sent+=1
        print(f'published stereo pairs: {sent}/{len(pairs)}; GT inputs: none')
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
