import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class Camera(Node):
    def __init__(self):
        super().__init__("cv_camera")
        
        # Declare and get parameter for camera device
        self.declare_parameter('camera_device', 0)
        self.camera_device = self.get_parameter('camera_device').value
        self.get_logger().info(f'Using camera device: {self.camera_device}')
        
        # Create publisher
        self.image_pub = self.create_publisher(Image, "camera/image", 1)
        self.cv_bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.camera_device)
        self.window_name = "Camera Preview"
        
        # Create timer for publishing camera frames
        self.timer = self.create_timer(0.033, self.timer_clk)  # ~30fps
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Start OpenCV window in a separate thread
        self.display_thread = threading.Thread(target=self.display_frame)
        self.display_thread.daemon = True  # Terminates when main thread ends
        self.display_thread.start()
        
    def parameters_callback(self, params):
        success = True
        reason = ""
        
        for param in params:
            if param.name == 'camera_device' and param.type_ == Parameter.Type.INTEGER:
                try:
                    # Close existing camera
                    if self.cap is not None:
                        self.cap.release()
                    
                    # Update camera device and reopen
                    self.camera_device = param.value
                    self.get_logger().info(f'Switching to camera device: {self.camera_device}')
                    self.cap = cv2.VideoCapture(self.camera_device)
                    
                    if not self.cap.isOpened():
                        success = False
                        reason = f'Failed to open camera device {self.camera_device}'
                        self.get_logger().error(reason)
                except Exception as e:
                    success = False
                    reason = f'Error switching camera: {str(e)}'
                    self.get_logger().error(reason)
        
        # Must return a SetParametersResult object, not a boolean
        return SetParametersResult(successful=success, reason=reason)
        
    def display_frame(self):
        while rclpy.ok():
            if self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    cv2.imshow(self.window_name, frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                        break
        cv2.destroyAllWindows()
        
    def timer_clk(self):
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                img_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    
    # Properly clean up resources
    if camera.cap is not None:
        camera.cap.release()
    cv2.destroyAllWindows()
    
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

