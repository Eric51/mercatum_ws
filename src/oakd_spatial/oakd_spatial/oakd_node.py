#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math
from cv_bridge import CvBridge # Note: cv_bridge est compilé pour le python system, devrait être accessible via --system-site-packages
import depthai as dai
import cv2
import numpy as np
import threading
import time

class OakDNode(Node):
    def __init__(self):
        super().__init__('oakd_node')
        
        self.declare_parameter('blob_path', '')
        self.blob_path = self.get_parameter('blob_path').value
        
        self.declare_parameter('usb_speed', 'SUPER')
        self.usb_speed = self.get_parameter('usb_speed').value.upper()
        
        self.color_pub = self.create_publisher(CompressedImage, '~/color/image/compressed', 10)
        self.depth_pub = self.create_publisher(Image, '~/depth/image', 10) # 16UC1 mm
        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        
        self.bridge = CvBridge()
        self.running = False
        self.device = None
        self.restart_needed = False
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # TF Parameters
        self.declare_parameter('tf_x', 0.1) # Ex: 10cm devant
        self.declare_parameter('tf_y', 0.0)
        self.declare_parameter('tf_z', 0.2) # Ex: 20cm hauteur
        self.declare_parameter('tf_roll', 0.0)
        self.declare_parameter('tf_pitch', 0.0)
        self.declare_parameter('tf_yaw', 0.0)
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'oak_d_frame')
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.thread = threading.Thread(target=self.run_pipeline)
        self.thread.start()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'blob_path':
                self.blob_path = param.value
                self.restart_needed = True
            elif param.name == 'usb_speed':
                self.usb_speed = param.value.upper()
                self.restart_needed = True
        return rclpy.node.SetParametersResult(successful=True)

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('parent_frame').value
        t.child_frame_id = self.get_parameter('child_frame').value
        
        t.transform.translation.x = self.get_parameter('tf_x').value
        t.transform.translation.y = self.get_parameter('tf_y').value
        t.transform.translation.z = self.get_parameter('tf_z').value
        
        r = self.get_parameter('tf_roll').value
        p = self.get_parameter('tf_pitch').value
        y = self.get_parameter('tf_yaw').value
        
        # Simple Euler to Quat
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

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        
        # Caméra Couleur
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setPreviewSize(300, 300)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        
        # Caméras Mono (Depth)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        monoRight = pipeline.create(dai.node.MonoCamera)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        
        # Sorties XLink
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        camRgb.video.link(xoutRgb.input)
        
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        stereo.depth.link(xoutDepth.input)
        
        # Neural Network (si blob fourni)
        if self.blob_path and self.blob_path != "none":
            nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            nn.setBlobPath(self.blob_path)
            nn.setConfidenceThreshold(0.5)
            nn.input.setBlocking(False)
            
            camRgb.preview.link(nn.input)
            stereo.depth.link(nn.inputDepth)
            
            xoutDet = pipeline.create(dai.node.XLinkOut)
            xoutDet.setStreamName("nn")
            nn.out.link(xoutDet.input)
            
        return pipeline

    def run_pipeline(self):
        self.running = True
        while rclpy.ok() and self.running:
            try:
                pipeline = self.create_pipeline()
                
                # Map string to dai.UsbSpeed
                speed_map = {
                    'SUPER': dai.UsbSpeed.SUPER,
                    'HIGH': dai.UsbSpeed.HIGH,
                    'FULL': dai.UsbSpeed.FULL
                }
                # Default to SUPER if unknown, or allow auto if empty? Let's default to SUPER/Auto
                req_speed = speed_map.get(self.usb_speed, dai.UsbSpeed.SUPER)
                
                with dai.Device(pipeline, maxUsbSpeed=req_speed) as device:
                    self.device = device
                    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                    qDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                    
                    qDet = None
                    if "nn" in device.getOutputQueueNames():
                        qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
                    
                    while rclpy.ok() and not self.restart_needed:
                        # Lecture bloquante ou non (ici blocking=False dans queue, mais on tryGet)
                        inRgb = qRgb.tryGet()
                        inDepth = qDepth.tryGet()
                        inDet = qDet.tryGet() if qDet else None
                        
                        ts = self.get_clock().now().to_msg()
                        
                        if inRgb:
                            # Encodage JPEG pour transport
                            frame = inRgb.getCvFrame() # BGR
                            msg = CompressedImage()
                            msg.header.stamp = ts
                            msg.format = "jpeg"
                            success, jpg = cv2.imencode('.jpg', frame)
                            if success:
                                msg.data = jpg.tobytes()
                                self.color_pub.publish(msg)
                        
                        if inDepth:
                            frame = inDepth.getFrame() # uint16 mm
                            # Conversion ROS
                            msg = self.bridge.cv2_to_imgmsg(frame, encoding="16UC1")
                            msg.header.stamp = ts
                            msg.header.frame_id = "oak_d_frame"
                            self.depth_pub.publish(msg)
                            
                        if inDet:
                            detections = inDet.detections
                            msg = Detection2DArray()
                            msg.header.stamp = ts
                            msg.header.frame_id = "oak_d_frame"
                            
                            for d in detections:
                                det = Detection2D()
                                # Bounding Box
                                det.bbox.center.position.x = float(d.xmin + (d.xmax - d.xmin)/2) * 300 # Ex: normalized
                                det.bbox.center.position.y = float(d.ymin + (d.ymax - d.ymin)/2) * 300 
                                det.bbox.size_x = float(d.xmax - d.xmin) * 300
                                det.bbox.size_y = float(d.ymax - d.ymin) * 300
                                
                                # Hypothèse avec Pose 3D
                                hyp = ObjectHypothesisWithPose()
                                hyp.hypothesis.class_id = str(d.label)
                                hyp.hypothesis.score = d.confidence
                                
                                # Position spatiale (X, Y, Z en mètres)
                                # DepthAI coords: X right, Y down, Z forward
                                hyp.pose.pose.position.x = d.spatialCoordinates.x / 1000.0
                                hyp.pose.pose.position.y = d.spatialCoordinates.y / 1000.0
                                hyp.pose.pose.position.z = d.spatialCoordinates.z / 1000.0
                                
                                det.results.append(hyp)
                                msg.detections.append(det)
                            
                            self.det_pub.publish(msg)
                        
                        time.sleep(0.001)

            except Exception as e:
                self.get_logger().error(f"Erreur pipeline DepthAI: {e}")
                time.sleep(1)
            
            if self.restart_needed:
                self.get_logger().info("Redémarrage pipeline...")
                self.restart_needed = False
                time.sleep(0.5)

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OakDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
