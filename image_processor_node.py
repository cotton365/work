#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        # 保留原有的图像订阅
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("图像处理节点已启动，使用RGB颜色空间识别")

    def detect_colors_rgb(self, frame):
        """RGB颜色空间识别红蓝色块"""
        # 分离RGB通道
        b, g, r = cv2.split(frame)
        
        # 红色检测：R通道值高，B和G通道值低
        red_condition = (r > 150) & (g < 100) & (b < 100)
        mask_red = np.zeros_like(r, dtype=np.uint8)
        mask_red[red_condition] = 255
        
        # 蓝色检测：B通道值高，R和G通道值低
        blue_condition = (b > 150) & (r < 100) & (g < 100)
        mask_blue = np.zeros_like(b, dtype=np.uint8)
        mask_blue[blue_condition] = 255
        
        return mask_red, mask_blue

    def find_and_draw_blocks(self, frame, mask, color, color_name):
        """查找色块并在图像上绘制"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result_image = frame.copy()
        blocks_info = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:  # 过滤小噪点
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 绘制边界框和中心点
                cv2.rectangle(result_image, (x, y), (x + w, y + h), color, 2)
                cv2.circle(result_image, (center_x, center_y), 5, color, -1)
                cv2.putText(result_image, f'{color_name}({int(area)})', (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                
                blocks_info.append({
                    'position': (center_x, center_y),
                    'size': (w, h),
                    'area': area
                })
                
                self.get_logger().info(f"{color_name}块: 位置({center_x}, {center_y}), 面积{int(area)}")
        
        return blocks_info, result_image

    def listener_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 使用RGB方法识别颜色
            mask_red, mask_blue = self.detect_colors_rgb(cv_image)
            
            # 处理红色块
            red_blocks, result_image = self.find_and_draw_blocks(cv_image, mask_red, (0, 0, 255), "RED")
            
            # 处理蓝色块
            blue_blocks, result_image = self.find_and_draw_blocks(result_image, mask_blue, (255, 0, 0), "BLUE")
            
            # 显示结果 - 保留原有显示功能
            cv2.imshow('Original Image', cv_image)
            cv2.imshow('Red Mask', mask_red)
            cv2.imshow('Blue Mask', mask_blue)
            cv2.imshow('RGB Detection Result', result_image)
            cv2.waitKey(1)
            
            self.get_logger().info(f'RGB方法检测: {len(red_blocks)}红, {len(blue_blocks)}蓝', throttle_duration_sec=1.0)
            
        except Exception as e:
            self.get_logger().error(f"处理图像时出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessorNode()
    try:
        rclpy.spin(image_processor_node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        image_processor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
