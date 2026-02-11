import serial
import threading
import time
import struct

class Rosmaster:
    def __init__(self, port='/dev/ttyUSB0', baud=115200, car_type=1):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            print(f"Erreur série: {e}")
        
        self.running = False
        self.imu_data = {'ax':0, 'ay':0, 'az':0, 'gx':0, 'gy':0, 'gz':0}
        self.battery = 0.0
        self.joints = [0.0]*6
        
    def create_receive_threading(self):
        self.running = True
        t = threading.Thread(target=self.__receive_data)
        t.start()
        
    def __receive_data(self):
        while self.running and self.ser and self.ser.is_open:
            # Simulation lecture protocole 
            # (A adapter selon protocole réel Yahboom)
            try:
                # Mock: lire ligne ou bytes
                if self.ser.in_waiting:
                    self.ser.read(self.ser.in_waiting)
                    # Mise à jour valeurs fictives pour test si pas de vrai hardware
                    # self.battery = 12.0
            except:
                pass
            time.sleep(0.05)
            
    def set_car_motion(self, vx, vy, wz):
        # Envoi commande motion
        # Protocole exemple: $V,vx,vy,wz#
        if not self.ser: return
        try:
            cmd = f"$V,{vx:.2f},{vy:.2f},{wz:.2f}#"
            self.ser.write(cmd.encode())
        except:
            pass

    def set_pwm_servo(self, servo_id, angle):
        # Commande servo
        if not self.ser: return
        try:
            cmd = f"$S,{servo_id},{int(angle)}#"
            self.ser.write(cmd.encode())
        except:
            pass
            
    def __del__(self):
        self.running = False
        if self.ser:
            self.ser.close()
