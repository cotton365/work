#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # 保留原有的图像发布者
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        
        # 新增：发布颜色掩码和检测结果
        self.color_mask_pub = self.create_publisher(Image, 'camera/color_mask', 10)
        self.detection_pub = self.create_publisher(Image, 'camera/detection', 10)
        
        self.bridge = CvBridge()
        
        # 尝试打开物理摄像头
        self.cap = None
        self.test_mode = True  # 默认使用测试模式
        
        # 尝试打开摄像头
        for i in range(4):  # 尝试多个摄像头索引
            self.cap = cv2.VideoCapture(i)
            if self.cap.isOpened():
                self.test_mode = False
                self.get_logger().info(f"成功打开物理摄像头索引 {i}")
                break
            else:
                self.cap = None
                
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("无法打开物理摄像头，使用测试模式")
            self.test_mode = True
        
        # 设置定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.get_logger().info("摄像头节点已启动")

    def detect_colors_hsv(self, frame):
        """HSV颜色空间识别红蓝色块"""
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 红色范围1 (0-10)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        # 红色范围2 (160-180)
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # 蓝色范围
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([140, 255, 255])
        
        # 创建掩码
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        return mask_red, mask_blue

    def find_and_draw_blocks(self, frame, mask, color, color_name):
        """查找色块并在图像上绘制"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detection_image = frame.copy()
        blocks_info = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:  # 过滤小噪点
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 绘制边界框和中心点
                cv2.rectangle(detection_image, (x, y), (x + w, y + h), color, 2)
                cv2.circle(detection_image, (center_x, center_y), 5, color, -1)
                cv2.putText(detection_image, f'{color_name}({int(area)})', (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                
                blocks_info.append({
                    'position': (center_x, center_y),
                    'size': (w, h),
                    'area': area
                })
                
                self.get_logger().info(f"{color_name}块: 位置({center_x}, {center_y}), 面积{int(area)}")
        
        return blocks_info, detection_image

    def create_test_image(self):
        """创建测试图像，包含红蓝色块"""
        self.counter += 1
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 绘制背景
        cv2.rectangle(test_image, (0, 0), (640, 480), (100, 100, 100), -1)
        
        # 绘制一些随机移动的红蓝色块
        for i in range(3):
            # 红色块
            red_x = 100 + (self.counter + i * 50) % 400
            red_y = 100 + (self.counter + i * 30) % 200
            cv2.rectangle(test_image, (red_x, red_y), (red_x + 60, red_y + 60), (0, 0, 255), -1)
            
            # 蓝色块
            blue_x = 200 + (self.counter + i * 70) % 300
            blue_y = 300 + (self.counter + i * 40) % 100
            cv2.rectangle(test_image, (blue_x, blue_y), (blue_x + 50, blue_y + 50), (255, 0, 0), -1)
        
        # 添加文字说明
        cv2.putText(test_image, "Camera Test - HSV Detection", (50, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return test_image

    def timer_callback(self):
        # 获取图像帧
        if self.test_mode:
            frame = self.create_test_image()
        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("读取摄像头帧失败")
                return
        
        # 使用HSV方法识别颜色
        mask_red, mask_blue = self.detect_colors_hsv(frame)
        
        # 处理红色块
        red_blocks, detection_image = self.find_and_draw_blocks(frame, mask_red, (0, 0, 255), "RED")
        
        # 处理蓝色块
        blue_blocks, detection_image = self.find_and_draw_blocks(detection_image, mask_blue, (255, 0, 0), "BLUE")
        
        # 发布原始图像
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(msg)
        
        # 发布颜色掩码
        color_mask = cv2.merge([mask_blue, mask_red, np.zeros_like(mask_red)])
        mask_msg = self.bridge.cv2_to_imgmsg(color_mask, "bgr8")
        self.color_mask_pub.publish(mask_msg)
        
        # 发布检测结果图像
        detection_msg = self.bridge.cv2_to_imgmsg(detection_image, "bgr8")
        self.detection_pub.publish(detection_msg)
        
        self.get_logger().info(f'检测到 {len(red_blocks)} 个红色块, {len(blue_blocks)} 个蓝色块', throttle_duration_sec=1.0)

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
