"""Match a browser detection to its server-owned RGB-D frame before retargeting."""
from __future__ import annotations

import time

from .rgbd_fusion import FusionUnavailable, fuse_hand


def prepare_packet(source, packet, side):
    """Return (trusted packet, captured snapshot, fused hand, diagnostics).

    The browser supplies landmark hypotheses only. Depth, calibration, capture
    time and lifetime always come from the retained native capture. Missing
    depth is a non-tracking observation, never a silent monocular fallback.
    """
    snapshot = None
    fused = None
    diagnostics = {'status': 'no_hand', 'measured_count': 0}
    hand = packet.get('hand')
    frame_id, stream_id = packet.get('realsense_frame_id'), packet.get('realsense_stream_id')
    metadata = {'source': 'realsense_rgbd', 'source_joint_set': 'mediapipe-21',
                'source_joint_count': 21, 'units': 'm',
                'coordinate_space': 'realsense_color_camera',
                'output_coordinate_space': 'camera_axes_wrist_relative',
                'normalized_is_image': True}
    if frame_id is not None and stream_id is not None:
        try:
            snapshot = source.consume(packet['session'], frame_id, stream_id)
        except ValueError as exc:
            diagnostics.update(status='unavailable', reason=str(exc))
    elif hand is not None:
        diagnostics.update(status='unavailable', reason='missing_rgbd_frame')
    if snapshot is not None:
        for key in ('frame_id', 'stream_id', 'revision', 'capture_unix_ms', 'timestamp_ms',
                    'device_id', 'device_name', 'depth_mode', 'color_mode', 'age_ms'):
            if key in snapshot:
                metadata[key] = (dict(snapshot[key]) if key in ('depth_mode', 'color_mode')
                                 else snapshot[key])
        metadata['intrinsics'] = dict(snapshot['intrinsics'])
        if 'observed_at' in snapshot:
            metadata['age_at_fusion_start_ms'] = max(0., (source.clock() - snapshot['observed_at']) * 1000)
        if hand is not None:
            if not isinstance(hand, dict) or hand.get('label') not in ('Left', 'Right'):
                raise ValueError('检测手别无效')
            if hand['label'] != side.title():
                diagnostics.update(status='no_hand', reason='wrong_hand')
            else:
                began = time.perf_counter()
                try:
                    fused = fuse_hand(hand['normalized'], hand['world'], snapshot['depth_m'],
                                      snapshot['intrinsics'], deproject=snapshot.get('deproject'))
                    diagnostics = dict(fused['diagnostics'], status='fused')
                    fused = dict(world=fused['world'], normalized=fused['normalized'], label=hand['label'])
                except FusionUnavailable as exc:
                    diagnostics.update(status='unavailable', reason=str(exc))
                diagnostics['fusion_ms'] = (time.perf_counter() - began) * 1000
                metadata['mediapipe_world_prior'] = hand['world']
                metadata['mediapipe_normalized'] = hand['normalized']
    metadata['depth_fusion'] = diagnostics
    trusted = dict(packet, hand=fused, source='realsense_rgbd', source_metadata=metadata,
                   raw_webxr=None, raw_wuji=None, raw_leapmotion=None)
    if snapshot is not None:
        trusted['capture_unix_ms'] = snapshot['capture_unix_ms']
        intrinsics = snapshot['intrinsics']
        trusted['camera_size'] = [intrinsics['width'], intrinsics['height']]
    else:
        trusted['capture_unix_ms'] = time.time() * 1000
    return trusted, snapshot, fused, diagnostics
