#!/usr/bin/env python3
"""
Sensor Fusion - FIXED DISTANCE ESTIMATION (No Video Recording)
- Radar-prioritized: Radar is much more accurate for distance
- Adaptive gating: Rejects bad associations when sensors disagree
- Shows improvement in distance estimation
"""

import sys
import time
import math
import cv2
import numpy as np
import pymoos
import socket
import threading
from dataclasses import dataclass
from typing import Dict, Set, Optional, Tuple
from collections import deque
from openvino.runtime import Core

# ============= Configuration =============
MOOS_SERVER = "192.168.143.16"
MOOS_PORT = 9000
RADAR_UDP_PORT = 8091

# Camera
RTSP_URL = "rtsp://douglas:Brazil2025!~@192.168.143.14:554/Streaming/Channels/101"
YOLO_MODEL_PATH = "yolo11n_openvino_model/yolo11n.xml"

# Camera calibration
CAMERA_HEIGHT = 2.13
AVERAGE_BOAT_HEIGHT = 2.5
HORIZONTAL_FOV = 87
VERTICAL_FOV = 46
DISTANCE_CALIBRATION_FACTOR = 0.5
EXCLUSION_ZONE = (0.3, 0.8, 0.65, 1.0)

# Fusion parameters - RADAR PRIORITIZED
ASSOCIATION_THRESHOLD = 25.0
QUALITY_THRESHOLD = 0.5

# Measurement noise (trust radar much more for distance!)
RADAR_POS_NOISE = 3.0    # ±3m (radar is very accurate)
RADAR_VEL_NOISE = 0.5    # ±0.5 m/s
VISION_POS_NOISE = 25.0  # ±25m (vision distance is unreliable)

# Sensor disagreement rejection
MAX_SENSOR_DISAGREEMENT = 0.4  # Reject if sensors differ by >40%

# Temporal confirmation
MIN_RADAR_UPDATES = 3
MIN_VISION_UPDATES = 3

# ============= Data Structures =============
@dataclass
class Detection:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    confidence: float = 1.0
    sensor_type: str = 'radar'
    timestamp: float = 0.0

class ExtendedKalmanFilter:
    def __init__(self, x, y, vx=0, vy=0):
        self.state = np.array([x, y, vx, vy])
        self.P = np.eye(4) * 100
        self.Q = np.diag([0.5, 0.5, 0.2, 0.2])
        
    def predict(self, dt):
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q
        
    def update_position_velocity(self, x, y, vx, vy, pos_noise=3.0, vel_noise=0.5):
        """Radar update - high accuracy"""
        H = np.eye(4)
        R = np.diag([pos_noise**2, pos_noise**2, vel_noise**2, vel_noise**2])
        z = np.array([x, y, vx, vy])
        y_innov = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y_innov
        self.P = (np.eye(4) - K @ H) @ self.P
        
    def update_position_only(self, x, y, pos_noise=25.0):
        """Vision update - low accuracy for position"""
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        R = np.diag([pos_noise**2, pos_noise**2])
        z = np.array([x, y])
        y_innov = z - H @ self.state
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y_innov
        I4 = np.eye(4)
        self.P = (I4 - K @ H) @ self.P

class FusedTrack:
    def __init__(self, track_id, detection):
        self.track_id = track_id
        self.ekf = ExtendedKalmanFilter(detection.x, detection.y, 
                                        detection.vx, detection.vy)
        self.sensor_sources = {detection.sensor_type}
        self.radar_updates = 1 if detection.sensor_type == 'radar' else 0
        self.vision_updates = 1 if detection.sensor_type == 'vision' else 0
        self.total_updates = 1
        self.last_update_time = detection.timestamp
        self.creation_time = detection.timestamp
        
        # Store last raw measurements
        self.last_radar_pos = None
        self.last_vision_pos = None
        
        # Statistics
        self.rejected_vision_updates = 0  # Count rejected vision measurements
        
    def check_sensor_agreement(self, new_x, new_y, sensor_type, own_x, own_y):
        """Check if new measurement agrees with existing estimate"""
        if sensor_type == 'radar':
            return True  # Always trust radar
        
        # For vision, check if it agrees with radar (if we have radar data)
        if self.last_radar_pos is None:
            return True  # No radar to compare with
        
        # Calculate distances
        radar_x, radar_y = self.last_radar_pos
        radar_dist = math.sqrt((radar_x - own_x)**2 + (radar_y - own_y)**2)
        vision_dist = math.sqrt((new_x - own_x)**2 + (new_y - own_y)**2)
        
        # Reject if they disagree too much
        if radar_dist > 0:
            disagreement = abs(radar_dist - vision_dist) / radar_dist
            if disagreement > MAX_SENSOR_DISAGREEMENT:
                return False  # Reject this vision measurement
        
        return True
        
    def update(self, detection, dt, own_x, own_y):
        """Update with sensor agreement checking"""
        # Check if this measurement makes sense
        agrees = self.check_sensor_agreement(detection.x, detection.y, 
                                            detection.sensor_type, own_x, own_y)
        
        if not agrees:
            self.rejected_vision_updates += 1
            return  # Don't update with bad measurement
        
        if dt > 0.001:
            self.ekf.predict(dt)
        
        if detection.sensor_type == 'radar':
            self.ekf.update_position_velocity(detection.x, detection.y,
                                             detection.vx, detection.vy,
                                             pos_noise=RADAR_POS_NOISE,
                                             vel_noise=RADAR_VEL_NOISE)
            self.radar_updates += 1
            self.last_radar_pos = (detection.x, detection.y)
        elif detection.sensor_type == 'vision':
            self.ekf.update_position_only(detection.x, detection.y,
                                         pos_noise=VISION_POS_NOISE)
            self.vision_updates += 1
            self.last_vision_pos = (detection.x, detection.y)
        
        self.sensor_sources.add(detection.sensor_type)
        self.total_updates += 1
        self.last_update_time = detection.timestamp
    
    def is_fused(self):
        return (self.radar_updates >= MIN_RADAR_UPDATES and 
                self.vision_updates >= MIN_VISION_UPDATES)
        
    def get_quality(self, current_time):
        age = current_time - self.last_update_time
        age_factor = max(0, 1 - age / 5.0)
        
        if self.is_fused():
            fusion_bonus = 1.5
            status = "FUSED"
        else:
            fusion_bonus = 0.3
            status = "PARTIAL"
            
        update_factor = min(1.0, self.total_updates / 10.0)
        quality = 0.9 * age_factor * fusion_bonus * update_factor
        
        return min(1.0, quality), status
    
    def get_all_distances(self, own_x, own_y) -> Tuple[Optional[float], Optional[float], float, float]:
        """Returns: (radar_dist, vision_dist, fused_dist, fused_bearing)"""
        # Fused distance (from EKF state)
        fused_x, fused_y = self.ekf.state[0], self.ekf.state[1]
        dx = fused_x - own_x
        dy = fused_y - own_y
        fused_dist = math.sqrt(dx**2 + dy**2)
        fused_bearing = math.degrees(math.atan2(dx, dy)) % 360
        
        # Radar distance
        radar_dist = None
        if self.last_radar_pos is not None:
            rx, ry = self.last_radar_pos
            dx_r = rx - own_x
            dy_r = ry - own_y
            radar_dist = math.sqrt(dx_r**2 + dy_r**2)
        
        # Vision distance
        vision_dist = None
        if self.last_vision_pos is not None:
            vx, vy = self.last_vision_pos
            dx_v = vx - own_x
            dy_v = vy - own_y
            vision_dist = math.sqrt(dx_v**2 + dy_v**2)
        
        return radar_dist, vision_dist, fused_dist, fused_bearing
    
    def build_node_report(self, current_time):
        x, y, vx, vy = self.ekf.state
        speed_ms = np.sqrt(vx**2 + vy**2)
        speed_kts = speed_ms / 0.514444
        heading = np.degrees(np.arctan2(vx, vy)) % 360
        
        node_report = f"NAME=fused_{self.track_id},"
        node_report += "TYPE=ship,"
        node_report += f"TIME={current_time:.2f},"
        node_report += f"X={x:.2f},"
        node_report += f"Y={y:.2f},"
        node_report += f"SPD={speed_kts:.2f},"
        node_report += f"HDG={heading:.2f},"
        node_report += f"YAW={heading:.2f},"
        node_report += "DEPTH=0.0,"
        node_report += "LENGTH=100,"
        node_report += "MODE=DRIVE"
        
        return node_report

# ============= Sensor Fusion Engine =============
class SensorFusion:
    def __init__(self):
        self.tracks: Dict[int, FusedTrack] = {}
        self.next_track_id = 1
        self.moos = None
        
        self.own_x = 0.0
        self.own_y = 0.0
        self.own_heading = 0.0
        
        self.stats_radar_detections = 0
        self.stats_vision_detections = 0
        self.stats_vision_rejected = 0
        self.stats_fused_published = 0
        self.stats_partial_filtered = 0
        
    def connect_moos(self):
        print("🔌 Connecting to MOOSDB...")
        self.moos = pymoos.comms()
        
        def on_connect():
            print("✅ Connected to MOOSDB")
            self.moos.register("NAV_X", 0)
            self.moos.register("NAV_Y", 0)
            self.moos.register("NAV_HEADING", 0)
            return True
        
        def on_mail():
            msgs = self.moos.fetch()
            for msg in msgs:
                key = msg.key()
                if key == "NAV_X":
                    self.own_x = msg.double()
                elif key == "NAV_Y":
                    self.own_y = msg.double()
                elif key == "NAV_HEADING":
                    self.own_heading = msg.double()
            return True
        
        self.moos.set_on_connect_callback(on_connect)
        self.moos.set_on_mail_callback(on_mail)
        self.moos.run(MOOS_SERVER, MOOS_PORT, 'pySensorFusion')
        time.sleep(1)
        
    def associate_detection(self, detection):
        best_track_id = None
        best_distance = ASSOCIATION_THRESHOLD
        
        for track_id, track in self.tracks.items():
            x, y = track.ekf.state[:2]
            dist = np.sqrt((detection.x - x)**2 + (detection.y - y)**2)
            if dist < best_distance:
                best_distance = dist
                best_track_id = track_id
                
        if best_track_id is not None:
            dt = detection.timestamp - self.tracks[best_track_id].last_update_time
            # Pass own position for agreement checking
            self.tracks[best_track_id].update(detection, dt, self.own_x, self.own_y)
        else:
            new_track = FusedTrack(self.next_track_id, detection)
            self.tracks[self.next_track_id] = new_track
            self.next_track_id += 1
            
    def cleanup_old_tracks(self, current_time):
        to_remove = []
        for track_id, track in self.tracks.items():
            age = current_time - track.last_update_time
            if age > 5.0:
                to_remove.append(track_id)
        for track_id in to_remove:
            del self.tracks[track_id]
            
    def publish_fused_only(self, current_time):
        published = 0
        partial = 0
        
        fused_tracks = [t for t in self.tracks.values() if t.is_fused()]
        if fused_tracks:
            print("\n" + "="*110)
            print("📍 FUSED CONTACTS - RADAR-PRIORITIZED DISTANCE (FIXED)")
            print("="*110)
        
        for track in self.tracks.values():
            if track.is_fused():
                quality, status = track.get_quality(current_time)
                if quality > QUALITY_THRESHOLD:
                    radar_dist, vision_dist, fused_dist, fused_bearing = track.get_all_distances(
                        self.own_x, self.own_y)
                    
                    # Publish to MOOS
                    node_report = track.build_node_report(current_time)
                    self.moos.notify('NODE_REPORT', node_report, current_time)
                    published += 1
                    self.stats_fused_published += 1
                    
                    # Print detailed comparison
                    print(f"\n🎯 Track {track.track_id}:")
                    
                    if radar_dist is not None:
                        print(f"   📡 RADAR:  {radar_dist:6.1f}m (HIGH ACCURACY - trusted)")
                    else:
                        print(f"   📡 RADAR:  N/A")
                    
                    if vision_dist is not None:
                        print(f"   🎥 VISION: {vision_dist:6.1f}m (LOW ACCURACY - confirmation only)")
                    else:
                        print(f"   🎥 VISION: N/A")
                    
                    print(f"   🔗 FUSED:  {fused_dist:6.1f}m @ {fused_bearing:5.1f}° (Radar-weighted estimate)")
                    
                    # Show disagreement and rejection stats
                    if radar_dist is not None and vision_dist is not None:
                        diff = abs(radar_dist - vision_dist)
                        error_pct = (diff / radar_dist) * 100 if radar_dist > 0 else 0
                        
                        if error_pct > MAX_SENSOR_DISAGREEMENT * 100:
                            print(f"   ⚠️  HIGH DISAGREEMENT: {diff:.1f}m ({error_pct:.1f}%) - vision measurements being rejected")
                        else:
                            print(f"   ✅ Agreement: {diff:.1f}m ({error_pct:.1f}%)")
                    
                    if track.rejected_vision_updates > 0:
                        print(f"   🚫 Rejected {track.rejected_vision_updates} bad vision measurements")
                    
                    print(f"   📊 Quality={quality:.2f}, Updates: R={track.radar_updates}, V={track.vision_updates}")
            else:
                partial += 1
                self.stats_partial_filtered += 1
        
        if fused_tracks:
            print("="*110)
                
        return published, partial
    
    def get_fused_tracks(self):
        return [t for t in self.tracks.values() if t.is_fused()]

# ============= Radar Receiver =============
class RadarReceiver(threading.Thread):
    def __init__(self, fusion):
        super().__init__(daemon=True)
        self.fusion = fusion
        self.running = True
        
    def parse_ttm(self, sentence):
        if not sentence.startswith('$') or 'TTM' not in sentence:
            return None
        if '*' in sentence:
            sentence = sentence.split('*')[0]
        fields = sentence.split(',')
        if len(fields) < 7:
            return None
        try:
            range_nm = float(fields[2])
            bearing_deg = float(fields[3])
            speed_kts = float(fields[5])
            course_deg = float(fields[6])
            return {
                'range_nm': range_nm,
                'bearing_deg': bearing_deg,
                'speed_kts': speed_kts,
                'course_deg': course_deg
            }
        except:
            return None
    
    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', RADAR_UDP_PORT))
        sock.settimeout(0.1)
        print(f"📡 Radar receiver listening on UDP {RADAR_UDP_PORT}")
        
        while self.running:
            try:
                data, _ = sock.recvfrom(1024)
                lines = data.decode('utf-8', errors='ignore').split('\n')
                
                for line in lines:
                    ttm = self.parse_ttm(line.strip())
                    if ttm is None:
                        continue
                    
                    range_m = ttm['range_nm'] * 1852.0
                    bearing_rad = np.radians(ttm['bearing_deg'])
                    x = self.fusion.own_x + range_m * np.sin(bearing_rad)
                    y = self.fusion.own_y + range_m * np.cos(bearing_rad)
                    
                    speed_ms = ttm['speed_kts'] * 0.514444
                    course_rad = np.radians(ttm['course_deg'])
                    vx = speed_ms * np.sin(course_rad)
                    vy = speed_ms * np.cos(course_rad)
                    
                    detection = Detection(x, y, vx, vy, 
                                        sensor_type='radar',
                                        timestamp=time.time())
                    self.fusion.associate_detection(detection)
                    self.fusion.stats_radar_detections += 1
                    
            except socket.timeout:
                continue
            except Exception:
                pass
    
    def stop(self):
        self.running = False

# ============= Vision Functions =============
def is_in_exclusion_zone(x1, y1, x2, y2, img_width, img_height):
    ex_x1 = int(EXCLUSION_ZONE[0] * img_width)
    ex_y1 = int(EXCLUSION_ZONE[1] * img_height)
    ex_x2 = int(EXCLUSION_ZONE[2] * img_width)
    ex_y2 = int(EXCLUSION_ZONE[3] * img_height)
    bbox_center_x = (x1 + x2) / 2
    bbox_center_y = (y1 + y2) / 2
    return (ex_x1 <= bbox_center_x <= ex_x2 and 
            ex_y1 <= bbox_center_y <= ex_y2)

def estimate_distance(bbox_height_pixels, image_height):
    if bbox_height_pixels < 10:
        return None
    vertical_fov_rad = math.radians(VERTICAL_FOV)
    focal_length_pixels = (image_height / 2) / math.tan(vertical_fov_rad / 2)
    distance = (AVERAGE_BOAT_HEIGHT * focal_length_pixels) / bbox_height_pixels
    return distance * DISTANCE_CALIBRATION_FACTOR

def calculate_relative_bearing(bbox_center_x, image_width):
    offset_x = bbox_center_x - (image_width / 2)
    max_angle = HORIZONTAL_FOV / 2
    normalized_offset = offset_x / (image_width / 2)
    return normalized_offset * max_angle

# ============= Main =============
def main():
    print("=" * 80)
    print("🚢 SENSOR FUSION - RADAR-PRIORITIZED DISTANCE (FIXED)")
    print("   Radar: ±3m noise (HIGH accuracy, trusted)")
    print("   Vision: ±25m noise (LOW accuracy, confirmation only)")
    print("   Rejects vision when disagreement >40%")
    print("=" * 80)
    
    fusion = SensorFusion()
    fusion.connect_moos()
    
    radar_thread = RadarReceiver(fusion)
    radar_thread.start()
    
    print(f"🎥 Loading YOLO model: {YOLO_MODEL_PATH}")
    ie = Core()
    model = ie.read_model(model=YOLO_MODEL_PATH)
    compiled_model = ie.compile_model(model=model, device_name="CPU")
    input_layer = compiled_model.input(0)
    output_layer = compiled_model.output(0)
    model_input_shape = input_layer.shape
    
    print(f"📹 Connecting to camera: {RTSP_URL}")
    cap = cv2.VideoCapture(RTSP_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    if not cap.isOpened():
        print("❌ Failed to open camera!")
        return
    
    width = int(cap.get(3))
    height = int(cap.get(4))
    
    print(f"\n✅ System running - Press Ctrl+C to stop")
    print("=" * 80)
    
    last_stats_time = time.time()
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.5)
                continue
            
            # YOLO inference (no video recording)
            img_resized = cv2.resize(frame, (model_input_shape[3], model_input_shape[2]))
            img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
            img_normalized = img_rgb.astype(np.float32) / 255.0
            img_transposed = np.transpose(img_normalized, (2, 0, 1))
            img_input = np.expand_dims(img_transposed, axis=0)
            
            output = compiled_model([img_input])[output_layer]
            predictions = output[0].T
            
            boxes = predictions[:, :4]
            class_scores = predictions[:, 4:]
            class_ids = np.argmax(class_scores, axis=1)
            confidences = np.max(class_scores, axis=1)
            mask = (confidences >= 0.25) & (class_ids == 8)
            
            for box, conf in zip(boxes[mask], confidences[mask]):
                x_center, y_center, w, h = box
                x1 = int((x_center - w/2) * width / model_input_shape[3])
                y1 = int((y_center - h/2) * height / model_input_shape[2])
                x2 = int((x_center + w/2) * width / model_input_shape[3])
                y2 = int((y_center + h/2) * height / model_input_shape[2])
                
                if is_in_exclusion_zone(x1, y1, x2, y2, width, height):
                    continue
                
                bbox_height = y2 - y1
                bbox_center_x = (x1 + x2) / 2
                dist = estimate_distance(bbox_height, height)
                
                if dist is None:
                    continue
                
                relative_bearing = calculate_relative_bearing(bbox_center_x, width)
                true_bearing = (fusion.own_heading + relative_bearing) % 360
                bearing_rad = math.radians(true_bearing)
                
                delta_x = dist * math.sin(bearing_rad)
                delta_y = dist * math.cos(bearing_rad)
                target_x = fusion.own_x + delta_x
                target_y = fusion.own_y + delta_y
                
                detection = Detection(target_x, target_y, 0.0, 0.0,
                                    confidence=float(conf),
                                    sensor_type='vision',
                                    timestamp=time.time())
                fusion.associate_detection(detection)
                fusion.stats_vision_detections += 1
            
            # Fusion processing
            current_time = time.time()
            fusion.cleanup_old_tracks(current_time)
            published, partial = fusion.publish_fused_only(current_time)
            
            # Count total rejected vision measurements
            total_rejected = sum(t.rejected_vision_updates for t in fusion.tracks.values())
            
            if current_time - last_stats_time > 10.0:
                print(f"\n📊 Total: Radar={fusion.stats_radar_detections}, "
                      f"Vision={fusion.stats_vision_detections}, "
                      f"Vision Rejected={total_rejected}, "
                      f"Fused Published={fusion.stats_fused_published}")
                last_stats_time = current_time
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down...")
        radar_thread.stop()
        cap.release()
        
        print("\n" + "=" * 80)
        print("📊 FINAL STATISTICS")
        print("=" * 80)
        print(f"Total radar detections:  {fusion.stats_radar_detections}")
        print(f"Total vision detections: {fusion.stats_vision_detections}")
        print(f"Total tracks created:    {fusion.next_track_id - 1}")
        print(f"✅ Fused published:      {fusion.stats_fused_published}")
        print(f"❌ Partial filtered:     {fusion.stats_partial_filtered}")
        print("=" * 80)

if __name__ == '__main__':
    main()
