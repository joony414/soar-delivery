import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from subprocess import Popen, PIPE
from threading import Thread, Timer
import Jetson.GPIO as GPIO
import serial


# NVIDIA Jetson 40 pin - 서보모터 연결
# 15pin (pwm1) --- 다리 서보모터 2개
# 33pin (pwm5) --- 아크릴 문 서보모터


class DeliveryNode(Node):
    def __init__(self):
        super().__init__("delivery_node")
        qos_profile = QoSProfile(depth=10)

        self.delivery_flag_subscriber = self.create_subscription(
            Bool, "/delivery_state", self.subscribe_delivery_flag, qos_profile
        )
        self.delivery_done_publisher = self.create_publisher(
            Bool, "/delivery_node/delivery_done", qos_profile
        )

        self.delivery_flag = False
        self.delivery_started = False
        # self.timer = self.create_timer(5, self.stop_subscription)

        # 40핀 설정
        self.PWM1_PIN = 15
        # self.PWM5_PIN = 33
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PWM1_PIN, GPIO.OUT, initial=GPIO.HIGH)
        # GPIO.setup(self.PWM5_PIN, GPIO.OUT, initial=GPIO.HIGH)
        self.pwm1 = GPIO.PWM(self.PWM1_PIN, 50)  # 50Hz => 20ms
        # self.pwm5 = GPIO.PWM(self.PWM5_PIN, 50)  # 50Hz => 20ms

        # Duty Ratio 설정 [0.0 ~ 100.0]
        self.duty_ratio_for_up = 10.75  # 배송 전 다리를 가장 접은 상태
        self.duty_ratio_for_down = 3.5  # 배송 중 다리를 가장 펼친 상태
        # self.duty_ratio_for_lock = 5.4  # 배송 전 문을 잠그고 있는 상태
        # self.duty_ratio_for_open = 13.0  # 배송 중 문을 여는 상태

        # 배송 전 상태 시작
        self.before_deleivering()

    def subscribe_delivery_flag(self, msg):
        self.delivery_flag = msg.data
        if self.delivery_flag == True and self.delivery_started == False:
            self.delivery_started = True
            self.start_delivering()

    # 배송 전
    def before_deleivering(self):
        self.pwm1.start(self.duty_ratio_for_up)
        # self.pwm5.start(self.duty_ratio_for_lock)
        self.get_logger().info("Current State : Before Delivery")

    # 배송 시작, 다리 하강
    def start_delivering(self):
        self.pwm1.stop()
        self.pwm1.start(self.duty_ratio_for_down)
        self.get_logger().info("Current State : Start Delivering")
        self.descend_timer = self.create_timer(10, self.open_door_command)  # 10초간 다리 하강

    # 문 개방
    def open_door_command(self):
        self.descend_timer.cancel()
        # self.pwm5.stop()
        # self.pwm5.start(self.duty_ratio_for_open)
        self.py_serial = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
        self.serial_open_timer = self.create_timer(
            2, self.open_door
        )  # 2초간 시리얼 통신 포트 오픈 대기

    def open_door(self):
        self.serial_open_timer.cancel()
        self.py_serial.write("a".encode("utf-8"))
        self.get_logger().info("Current State : Opening door")
        self.open_timer = self.create_timer(4, self.finish_delivering)  # 4초간 문 개방

    # 문 원래대로, 다리 상승
    def finish_delivering(self):
        self.open_timer.cancel()
        # self.pwm5.stop()
        # self.pwm5.start(self.duty_ratio_for_lock)
        self.pwm1.stop()
        self.pwm1.start(self.duty_ratio_for_up)
        self.get_logger().info("Current State : Finish Delivering")
        self.ascend_timer = self.create_timer(30, self.publish_done)  # 30초간 다리 상승

    def publish_done(self):
        self.ascend_timer.cancel()
        msg = Bool()
        msg.data = True
        self.delivery_done_publisher.publish(msg)
        self.get_logger().info("Delivery Done messgae published!")


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.pwm1.stop()
        # node.pwm5.stop()  
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
