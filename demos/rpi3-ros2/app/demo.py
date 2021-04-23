
import time
import rclpy
from example_app.msg import ThunderStruck

def main():
    rclpy.init()
    node = rclpy.create_node('python_demo_publisher')
    publisher = node.create_publisher(ThunderStruck, '/thunder', 10)
    msg = ThunderStruck()
    msg.sine = 22.0
    msg.cosine = 3.2
    msg.funky = 13.12
    while rclpy.ok():
        msg.cosine += 1
        msg.sine -= 1
        msg.funky += 13
        publisher.publish(msg)
        time.sleep(0.2)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
