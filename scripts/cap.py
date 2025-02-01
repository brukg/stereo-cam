#!/usr/bin/python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import os

class StereoImageCapture(Node):
    def __init__(self):
        super().__init__('stereo_image_capture')
        self.bridge = CvBridge()
        self.image_count = 0
        
        # Create message filter subscribers
        self.left_sub = Subscriber(self, Image, 'camera/left/image_raw')
        self.right_sub = Subscriber(self, Image, 'camera/right/image_raw')
        
        # Synchronize the topics with a 0.1 second tolerance
        self.ts = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Create directories if they don't exist
        self.image_dir = 'left_right_image'
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
            
        self.get_logger().info("Stereo image capture node initialized")
        print("Press 'q' to exit")
        print("Press 'k' to save current stereo pair")

    def generate_calibration_file(self):
        xml_content = '<?xml version="1.0"?>\n<opencv_storage>\n<imagelist>\n'
        
        # Get all files in the directory
        files = os.listdir(self.image_dir)
        left_files = sorted([f for f in files if f.startswith('left')])
        right_files = sorted([f for f in files if f.startswith('right')])
        
        # Add paths to XML
        for left, right in zip(left_files, right_files):
            xml_content += f'./{self.image_dir}/{left}\n'
            xml_content += f'./{self.image_dir}/{right}\n'
        
        xml_content += '</imagelist>\n</opencv_storage>'
        
        # Write the XML file
        with open('stereo_calib.xml', 'w') as f:
            f.write(xml_content)
        
        self.get_logger().info("Generated stereo_calib.xml")

    def sync_callback(self, left_msg, right_msg):
        try:
            # Convert both images
            left_image = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            right_image = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
            
            # Concatenate images horizontally for display
            stereo_image = cv2.hconcat([left_image, right_image])
            
            # Display the combined image
            cv2.imshow("Stereo Cameras", stereo_image)
            key = cv2.waitKey(1)
            
            if key == ord('q'):
                self.get_logger().info('Shutting down...')
                rclpy.shutdown()
            elif key == ord('k'):
                # Save both images separately
                left_name = f"{self.image_dir}/left{self.image_count}.jpg"
                right_name = f"{self.image_dir}/right{self.image_count}.jpg"
                cv2.imwrite(left_name, left_image)
                cv2.imwrite(right_name, right_image)
                self.get_logger().info(f"Saved image pair {self.image_count}")
                self.image_count += 1
                
                # Generate calibration file after each save
                self.generate_calibration_file()
                
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = StereoImageCapture()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
