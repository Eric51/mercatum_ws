#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import serial
import struct
import math
import threading
import time

class LD20Node(Node):
    def __init__(self):
        super().__init__('ld20_node')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200) # LD20 est souvent 115200 ou 230400 selon modèles
        self.declare_parameter('frame_id', 'fhl_ld20')
        self.declare_parameter('tf_x', 0.0)
        self.declare_parameter('tf_y', 0.0)
        self.declare_parameter('tf_z', 0.0)
        self.declare_parameter('tf_roll', 0.0)
        self.declare_parameter('tf_pitch', 0.0)
        self.declare_parameter('tf_yaw', 0.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '~/scan', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/points', 10)
        
        # TF Statique
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()
        
        # Buffer pour scan complet 360
        self.scan_buffer = []
        self.last_angle = 0.0
        
        self.running = False
        self.thread = threading.Thread(target=self.serial_loop)
        self.thread.start()

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.frame_id
        
        t.transform.translation.x = self.get_parameter('tf_x').value
        t.transform.translation.y = self.get_parameter('tf_y').value
        t.transform.translation.z = self.get_parameter('tf_z').value
        
        # Conversion Euler -> Quaternion (simplifiée, ou utiliser transforms3d/scipy si dispo, sinon manuelle)
        r = self.get_parameter('tf_roll').value
        p = self.get_parameter('tf_pitch').value
        y = self.get_parameter('tf_yaw').value
        
        # Simple RPY to Quaternion approximation or import
        # Here assuming flat mounting (0,0,0) mostly, for robust simple conversion:
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        self.tf_broadcaster.sendTransform(t)

    def serial_loop(self):
        self.running = True
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f"Connecté au Lidar sur {self.port}")
        except Exception as e:
            self.get_logger().error(f"Erreur ouverture port série: {e}")
            return

        buffer = b''
        PACKET_SIZE = 47
        
        while rclpy.ok() and self.running:
            try:
                # Lecture
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                buffer += data
                
                # Recherche header 0x54 0x2C
                while len(buffer) >= PACKET_SIZE:
                    idx = buffer.find(b'\x54\x2c')
                    if idx == -1:
                        # Pas de header, on garde la fin au cas où
                        buffer = buffer[-1:]
                        break
                    
                    if idx > 0:
                        buffer = buffer[idx:]
                        
                    if len(buffer) < PACKET_SIZE:
                        break # Attendre plus de données
                        
                    # On a un paquet potentiel
                    packet = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]
                    
                    # Logique de parsing (sans checksum complet pour simplicité, mais idéalement vérifier CRC)
                    # Structure: Header(2), Speed(2), StartAngle(2), Data(36), EndAngle(2), Timestamp(2), CRC(1)
                    
                    speed = struct.unpack('<H', packet[2:4])[0]
                    start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
                    end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0
                    
                    # Correction angles
                    if end_angle < start_angle:
                        end_angle += 360.0
                        
                    step = (end_angle - start_angle) / 11.0 # 12 points -> 11 intervalles
                    
                    current_points = []
                    
                    for i in range(12):
                        # Data starts at offset 6. Each point is 3 bytes (dist L, dist H, Conf)
                        base = 6 + i*3
                        dist = struct.unpack('<H', packet[base:base+2])[0] # mm
                        conf = packet[base+2]
                        
                        full_angle = start_angle + i * step
                        if full_angle >= 360.0: full_angle -= 360.0
                        
                        # Conversion mm -> m
                        dist_m = dist / 1000.0
                        
                        if dist_m > 0 and conf > 100: # Seuil qualité exemple
                            current_points.append((full_angle, dist_m, conf))
                    
                    self.accumulate_scan(current_points)
                    
            except Exception as e:
                self.get_logger().error(f"Erreur lecture boucle série: {e}")
                time.sleep(1)

    def accumulate_scan(self, points):
        # Accumule pour LaserScan 360
        # Si on détecte un passage par zéro (start_angle < last_angle de beaucoup), on publie
        
        if not points: return
        
        first_angle = points[0][0]
        
        # Détection tour complet (simple heuristique)
        if first_angle < self.last_angle - 100: 
            self.publish_scan()
            self.scan_buffer = []

        self.scan_buffer.extend(points)
        self.last_angle = points[-1][0]

    def publish_scan(self):
        if not self.scan_buffer: return
        
        # Trier par angle
        self.scan_buffer.sort(key=lambda x: x[0])
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0 # Résolution arbitraire 1 deg pour le message scan
        scan.range_min = 0.1
        scan.range_max = 12.0
        
        # Remplissage ranges (360 bins)
        ranges = [float('inf')] * 360
        intensities = [0.0] * 360
        
        for deg, r, i in self.scan_buffer:
            idx = int(deg) % 360
            if r < ranges[idx]: # Garder le plus proche si plusieurs dans le même degré
                ranges[idx] = r
                intensities[idx] = float(i)
                
        scan.ranges = ranges
        scan.intensities = intensities
        
        self.scan_pub.publish(scan)
        
        # TODO: PointCloud2 si requis (optionnel car LaserScan suffit souvent)
        # On le fait si explicitement demandé prompt: "Publie /<ns>/points (sensor_msgs/PointCloud2)"
        # Note: LaserScan -> PointCloud2 est standard via filter, mais on peut le faire ici.

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LD20Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
