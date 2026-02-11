#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import subprocess
import threading
import json
import time
import os
import signal
import random

class IMX500Node(Node):
    def __init__(self):
        super().__init__('imx500_node')
        
        # Paramètres
        # On essaie de trouver le fichier dans le workspace si chemin relatif
        default_pp = 'imx500_postprocess.json'
        if not os.path.exists(default_pp):
            expanded = os.path.expanduser(f'~/mercatum_ws/{default_pp}')
            if os.path.exists(expanded):
                default_pp = expanded

        self.declare_parameter('postprocess_file', default_pp)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        self.postprocess_file = self.get_parameter('postprocess_file').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        # Publishers
        self.image_pub = self.create_publisher(CompressedImage, '/imx500/image/compressed', 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/imx500/detections', 10)
        
        self.process = None
        self.running = False
        self.restart_needed = False
        
        # Crash detection logic
        self.crash_count = 0
        self.crash_timestamps = []
        self.MAX_CRASH_RATE = 5 # 5 crashes...
        self.CRASH_WINDOW = 60 # ...in 60 seconds triggers mock mode
        
        # Callback pour paramètre dynamique
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Thread de gestion du processus
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.process_loop)
        self.thread.start()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'postprocess_file':
                new_val = param.value
                if new_val != self.postprocess_file:
                    self.get_logger().info(f"Paramètre postprocess_file changé: {new_val}")
                    self.postprocess_file = new_val
                    self.restart_needed = True
        return rclpy.node.SetParametersResult(successful=True)

    def find_executable(self):
        # Cherche rpicam-vid ou libcamera-vid dans le PATH
        candidates = ['rpicam-vid', 'libcamera-vid']
        import shutil
        for c in candidates:
            if shutil.which(c):
                return c
        return None

    def start_process(self):
        exe = self.find_executable()
        if not exe:
            self.get_logger().warn("Aucun exécutable caméra trouvé (rpicam-vid/libcamera-vid). MODE SIMULATION ACTIVÉ.")
            self.start_mock_mode()
            return True

        cmd = [
            exe,
            '-t', '0',
            '--width', str(self.width),
            '--height', str(self.height),
            '--codec', 'mjpeg',
            '--inline', # Headers MJPEG
            '-o', '-'  # stdout
        ]
        
        # Ajout post-process si fichier existe (ou chemin valide)
        if self.postprocess_file and self.postprocess_file != "none":
             if os.path.exists(self.postprocess_file):
                 cmd.extend(['--post-process-file', self.postprocess_file])
             else:
                 self.get_logger().warn(f"Fichier post-process introuvable: {self.postprocess_file}")

        self.get_logger().info(f"Lancement commande: {' '.join(cmd)}")
        
        try:
            # On capture stdout pour l'image et stderr pour les métadonnées/logs
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0 # Unbuffered
            )
        except Exception as e:
            self.get_logger().error(f"Erreur lancement rpicam-vid: {e}")
            return False
            
        return True

    def stop_process(self):
        if self.process:
            self.get_logger().info("Arrêt du processus rpicam-vid...")
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None

    def process_loop(self):
        self.running = True
        
        while rclpy.ok() and self.running:
            if self.restart_needed:
                self.stop_process()
                self.restart_needed = False
            
            # Si mode Mock actif, on ne lance pas de sous-processus
            if hasattr(self, 'mock_mode') and self.mock_mode:
                time.sleep(0.1)
                continue

            if self.process is None or self.process.poll() is not None:
                if not self.start_process():
                    # Si start_process a activé le mock, on continue
                    if hasattr(self, 'mock_mode') and self.mock_mode:
                        continue
                    time.sleep(1)
                    continue
            
            # Lecture des threads séparés pour stdout et stderr
            # Note: Pour simplifier dans un seul thread loop, c'est complexe car bloquant.
            # On va lancer 2 threads dédiés à la lecture des flux du sous-processus courant.
            
            stop_event = threading.Event()
            
            # Si on a un process réel, on lance les threads de lecture
            t_vid = None
            t_meta = None
            
            if self.process:
                t_vid = threading.Thread(target=self.read_video, args=(self.process.stdout, stop_event))
                t_meta = threading.Thread(target=self.read_metadata, args=(self.process.stderr, stop_event))
                t_vid.start()
                t_meta.start()
            
            # Surveillance du process
            while rclpy.ok() and not self.restart_needed:
                if self.process and self.process.poll() is not None:
                    self.get_logger().warn("Processus rpicam-vid terminé inopinément.")
                    
                    # Analyser la fréquence des crashs
                    now = time.time()
                    self.crash_timestamps.append(now)
                    # Garder seulement les crashs récents
                    self.crash_timestamps = [t for t in self.crash_timestamps if now - t < self.CRASH_WINDOW]
                    
                    if len(self.crash_timestamps) >= self.MAX_CRASH_RATE:
                        self.get_logger().error(f"Trop de crashs ({len(self.crash_timestamps)} en {self.CRASH_WINDOW}s). Passage forcé en MODE SIMULATION.")
                        self.stop_process()
                        self.start_mock_mode()
                        break 
                        
                    break
                
                # Check mock mode status if needed
                if hasattr(self, 'mock_mode') and self.mock_mode:
                     time.sleep(1)
                     continue

                time.sleep(0.5)
                
            stop_event.set()
            if t_vid: t_vid.join()
            if t_meta: t_meta.join()
            
            # Si on sort, c'est pour restart ou shutdown
            self.stop_process()

    def read_video(self, pipe, stop_event):
        # Lecture MJPEG basique : cherche SOI (0xFF 0xD8) et EOI (0xFF 0xD9)
        # Note: ceci est une implémentation simple. Pour plus de perf, lire par chunks.
        buffer = b''
        while not stop_event.is_set():
            try:
                chunk = pipe.read(4096)
                if not chunk:
                    break
                buffer += chunk
                
                a = buffer.find(b'\xff\xd8')
                b = buffer.find(b'\xff\xd9')
                
                if a != -1 and b != -1:
                    if b > a:
                        jpg = buffer[a:b+2]
                        
                        msg = CompressedImage()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.format = "jpeg"
                        msg.data = jpg
                        self.image_pub.publish(msg)
                        
                        buffer = buffer[b+2:]
                    else:
                        # Cas rare où EOI avant SOI dans le buffer
                        buffer = buffer[b+2:]
            except Exception as e:
                # self.get_logger().error(f"Erreur lecture vidéo: {e}")
                break

    def read_metadata(self, pipe, stop_event):
        # Lit ligne par ligne stderr pour chercher du JSON
        import selectors
        sel = selectors.DefaultSelector()
        sel.register(pipe, selectors.EVENT_READ)

        while not stop_event.is_set():
            if sel.select(0.1):
                try:
                    line = pipe.readline()
                    if not line:
                        break
                    line_str = line.decode('utf-8', errors='ignore').strip()
                    
                    # Tenter de parser JSON
                    if line_str.startswith('{') and line_str.endswith('}'):
                        try:
                            data = json.loads(line_str)
                            self.publish_detections(data)
                        except json.JSONDecodeError:
                            self.get_logger().warn(f"rpicam stderr (non-JSON): {line_str}")
                    else:
                        self.get_logger().warn(f"rpicam stderr: {line_str}")
                except Exception:
                    break

    def publish_detections(self, data):
        # Adapter selon le format réel de sortie de rpicam-vid post-process
        # Exemple: {"detections": [{"x": 10, "y": 20, "w": 50, "h": 60, "label": 1, "score": 0.9}]}
        if "detections" in data:
            msg = Detection2DArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            for d in data["detections"]:
                det = Detection2D()
                # vision_msgs/BoundingBox2D center is vision_msgs/Pose2D
                # vision_msgs/Pose2D has 'position' (Point2D) and 'theta'
                det.bbox.center.position.x = float(d.get("x", 0) + d.get("w", 0)/2)
                det.bbox.center.position.y = float(d.get("y", 0) + d.get("h", 0)/2)
                det.bbox.size_x = float(d.get("w", 0))
                det.bbox.size_y = float(d.get("h", 0))
                
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(d.get("label", "unknown"))
                hyp.hypothesis.score = float(d.get("score", 0.0))
                det.results.append(hyp)
                
                msg.detections.append(det)
            
            self.det_pub.publish(msg)

    def start_mock_mode(self):
        self.mock_mode = True
        self.mock_thread = threading.Thread(target=self.mock_loop)
        self.mock_thread.start()

    def mock_loop(self):
        while rclpy.ok() and self.running and self.mock_mode:
            # Fake Image (Black Image)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            # 1 pixel JPEG approx
            msg.data = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\xff\xc0\x00\x11\x08\x00\x20\x00\x20\x03\x01\x22\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xda\x00\x0c\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00\xbf\x80\xff\xd9'
            self.image_pub.publish(msg)

            # Fake Detection
            data = {"detections": [{"x": random.randint(0, 300), "y": random.randint(0, 300), "w": 50, "h": 50, "label": "person", "score": 0.95}]}
            self.publish_detections(data)
            
            time.sleep(0.1)

    def destroy_node(self):
        self.running = False
        self.stop_process()
        if hasattr(self, 'mock_mode'): self.mock_mode = False
        if self.thread.is_alive():
            self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMX500Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
