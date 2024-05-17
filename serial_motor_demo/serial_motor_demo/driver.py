import threading
import json
import websocket
import rclpy
import math
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import MotorEncs

class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')
        self.initialize_websocket()
        
        self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)
        self.encoder_pub = self.create_publisher(MotorEncs, 'encoder_vals', 10)
        self.subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

    def initialize_websocket(self):
        # Crea una conexión WebSocket
        websocket_url = "ws://192.168.101.79:8070/messaging"
        self.ws = websocket.WebSocketApp(websocket_url,
                                          on_message=self.on_message,
                                          on_error=self.on_error,
                                          on_close=self.on_close)
        # Inicia el bucle del hilo principal para mantener la aplicación en ejecución
        threading.Thread(target=self.ws.run_forever).start()

    def on_message(self, ws, message):
        # Procesa el mensaje recibido del servidor
        server_data = json.loads(message)
        self.publish_motor_encoders(server_data)
        self.publish_motor_velocities(server_data)

    def on_error(self, ws, error):
        print("WebSocket error:", error)
        
    def calcular_velocidad_angular(self, m1, m2, L):
        v = (m1 + m2) / 2
        if m1 == m2:
            omega = 0
        else:
            omega = (m1 - m2) / L
        return v, omega
    
    def calcular_angulo_giro(self, omega):
        if omega == 0:
            return 90  # Si no hay rotación, el ángulo de giro es 90 grados.
        elif omega > 0:
            return 90 - math.degrees(math.atan(1 / omega))
        else:
            return 270 - math.degrees(math.atan(1 / omega))

    def on_close(self, ws, close_status_code, close_msg):
        print(f"WebSocket connection closed with status code {close_status_code}: {close_msg}")

    def motor_command_callback(self, motor_command):
        v, omega = self.calcular_velocidad_angular(
            motor_command.mot_1_req_rad_sec, 
            motor_command.mot_2_req_rad_sec,
            0.15
            )
        # Crea el JSON con las velocidades requeridas para los motores
        json_message = {
                        "type": "motor_traction",
                        "data": {
                            "speed_left": motor_command.mot_1_req_rad_sec,
                            "speed_right": motor_command.mot_2_req_rad_sec,
                        }
        }
        # Convierte el JSON a cadena y envíalo por el WebSocket
        json_message_str = json.dumps(json_message)
        self.ws.send(json_message_str)

    def publish_motor_velocities(self, server_data):
        # Publica las velocidades de los motores obtenidas del servidor
        spd_msg = MotorVels()
        spd_msg.mot_1_left_sec = server_data["speed"]["left_motor_1"]
        spd_msg.mot_2_left_sec = server_data["speed"]["left_motor_2"]
        spd_msg.mot_1_right_sec = server_data["speed"]["right_motor_1"]
        spd_msg.mot_2_right_sec = server_data["speed"]["right_motor_2"]
        self.speed_pub.publish(spd_msg)
        
    def publish_motor_encoders(self, server_data):
        # Publica las velocidades de los motores obtenidas del servidor
        enc_msg = MotorEncs()
        enc_msg.mot_1_left_en = server_data["rpm"]["left_motor_1"]
        enc_msg.mot_2_left_en = server_data["rpm"]["left_motor_2"]
        enc_msg.mot_1_right_en = server_data["rpm"]["right_motor_1"]
        enc_msg.mot_2_right_en = server_data["rpm"]["right_motor_2"]
        self.encoder_pub.publish(enc_msg)
        
def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriver()
    try:
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        pass
    motor_driver.destroy_node()
    rclpy.shutdown()