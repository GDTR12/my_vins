import rclpy
import rclpy.subscription
from sensor_msgs.msg._image import Image
from sensor_msgs.msg._point_cloud2 import PointCloud2
from my_vins_msg.msg import FeatureMatchPrevRequest
from my_vins_msg.msg import FeatureMatchPrevResponse
from rclpy.node import Node
from cv_bridge import CvBridge
import torch
import cv2


class FrontEnd(Node):
    def __init__(self):
        super().__init__('my_vins_FrontEnd')
        self.get_logger().info('Hello, ROS 2 Python Node!')
        self.bridge = CvBridge()

        self.sub_match = self.create_subscription(
            FeatureMatchPrevRequest,
            'front/match_prev_request',
            self.featureMatchPrevCallback,
            1000
        )

        self.pub_match = self.create_publisher(
            FeatureMatchPrevResponse,
            'front/match_prev_response',
            1000,
        )
        self.pub_matchvis = self.create_publisher(
            Image,
            'front/match_prev_show',
            1000,
        )
        self.pub_track = self.create_publisher(
            Image,
            'front/track',
            1000,
        )

    def featureMatchPrevCallback(self, request:FeatureMatchPrevRequest):
        return
    
    def createMatchPrevShow(self, img0, kpts0, img1, kpts1, matches0):
        img = cv2.hconcat([img0, img1])
        w = img0.shape[1]
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        pts0 = torch.round(kpts0[0]).int().cpu().squeeze()
        pts1 = torch.round(kpts1[0]).int().cpu().squeeze()
        k0 = torch.split(pts0, 1, dim=0)
        k1 = torch.split(pts1, 1, dim=0)
        # self.get_logger().info(f'{k0[0].shape}')
        for idx, m in enumerate(torch.split(matches0[0][0], 1)):
            m = m.reshape((1))
            if m == -1:
                continue
            else:
                # self.get_logger().info(f'{k0[idx][0,0].item(), k0[idx][0,1].item()}')
                cv2.circle(img, (k0[m][0,0].item(), k0[m][0,1].item()), 2, (0,0,255))
                cv2.circle(img, (k1[idx][0,0].item() + w, k1[idx][0,1].item()), 2, (0,0,255))
                cv2.line(img, 
                         (k0[m][0,0].item(), k0[m][0,1].item()),
                         (k1[idx][0,0].item() + w, k1[idx][0,1].item()),
                         (0,255,0),
                         1)
        
        msg = self.bridge.cv2_to_imgmsg(img)
        self.pub_matchvis.publish(msg)

        
        
