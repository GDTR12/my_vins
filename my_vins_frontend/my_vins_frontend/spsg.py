import rclpy
import rclpy.logging
from rclpy.node import Node
from .frontend_base import FrontEnd
from pathlib import Path
import argparse
import numpy as np
import cv2
import threading
import torch
from .dl_feature.SuperGluePretrainedNetwork.models.superglue import SuperGlue
from .dl_feature.SuperGluePretrainedNetwork.models.superpoint import SuperPoint
from .dl_feature.SuperGluePretrainedNetwork.models.utils import (AverageTimer,
                          make_matching_plot_fast, frame2tensor)
from my_vins_msg.msg import FeatureMatchPrevRequest
from my_vins_msg.msg import FeatureMatchPrevResponse

class SPSG(FrontEnd):
    default_config = {
        'superglue': {
            'descriptor_dim': 256,
            'weights': 'indoor',
            'keypoint_encoder': [32, 64, 128, 256],
            'GNN_layers': ['self', 'cross'] * 9,
            'sinkhorn_iterations': 20,
            'match_threshold': 0.2,
        },
        'superpoint': {
            'descriptor_dim': 256,
            'nms_radius': 4,
            'keypoint_threshold': 0.005,
            'max_keypoints': 300,
            'remove_borders': 4,
        }
    }
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    prev_extract = {}
    request_list = []
    previmg = []

    def __init__(self, config={}):
        super().__init__()
        self.default_config = {**self.default_config, **config}
        self.superpoint = SuperPoint(self.default_config.get('superpoint', {})).eval().to(self.device)
        self.superglue = SuperGlue(self.default_config.get('superglue', {})).eval().to(self.device)
        self.thd_hd_rqt = threading.Thread(target=self.handleRequests) 
        self.lock_rqt_list = threading.Lock()
        self.cv_lock_rqt = threading.Condition()
        self.get_logger().info('='*20)
        self.get_logger().info(f'Use device: {self.device}')
        self.get_logger().info('='*20)

        self.thd_hd_rqt.start()

    def setOneConfig(self, config: dict):
        for k, v in config.items():
            if k == 'superpoint':
                self.superpoint.config = {**self.superpoint.config, **v}
            elif k == 'superglue':
                self.superglue.config = {**self.superglue.config, **v}

    # def featureExtractCallback(self, request, response):
    #     img = self.bridge.imgmsg_to_cv2(request.img)
    #     img_tensor = frame2tensor(img, self.device)

    #     result = self.extract(img_tensor)
    #     keypoints = result.get('keypoints', {})
    #     scores = result.get('scores', {})
    #     descriptors = result.get('descriptors', {})
    #     keypoints = keypoints[0].reshape((-1))
    #     scores = scores[0]
    #     descriptors =  descriptors[0].reshape((-1))
    #     response.dim = 256
    #     response.kpts = keypoints.tolist()
    #     response.scores = scores.tolist()
    #     response.descriptors = descriptors.tolist()
    #     return response



    def featureMatchPrevCallback(self, request):
        with self.lock_rqt_list:
            self.request_list.append({
                'frame' : request.frame,
                'id' : request.id,
                'image' : request.img
            })
        with self.cv_lock_rqt:
            self.cv_lock_rqt.notify()

    def handleRequests(self):
        while True:
            with self.cv_lock_rqt:
                self.cv_lock_rqt.wait_for(lambda: len(self.request_list) > 0)
            with self.lock_rqt_list:
                request = self.request_list.pop(0)
            response = FeatureMatchPrevResponse()
            img = self.bridge.imgmsg_to_cv2(request['image'])
            if len(img.shape) != 2:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img1 = frame2tensor(img, self.device)
            # self.get_logger().info(f'{img1.shape}')

            extract = self.superpoint({'image': img1})

            if len(self.prev_extract.items()) == 0:

                response.dim = 256
                response.kpts = extract.get('keypoints', {})[0].reshape((-1)).tolist()
                response.scores = extract.get('scores', {})[0].reshape((-1)).tolist()
                response.descriptors = extract.get('descriptors', {})[0].reshape((-1)).tolist()
                response.match_prev_id = np.full((len(response.kpts) // 2, 1), -1, dtype=np.int32).reshape((-1)).tolist()
                response.match_prev_score = np.full((len(response.kpts) // 2, 1), 0, dtype=np.float32).reshape((-1)).tolist()
            else:
                data = {
                    **self.prev_extract,
                    **{k+'1': v for k, v in extract.items()},
                    'image1':img1
                }
                
                for k in data:
                    if isinstance(data[k], (list, tuple)):
                        data[k] = torch.stack(data[k])
                        # self.get_logger().info(f'{k}:{data[k].shape}')

                result = self.superglue(data)
                matches0 = result.get('matches0', {})
                matches1 = result.get('matches1', {})
                scores0 = result.get('matching_scores0', {})
                scores1 = result.get('matching_scores1', {})
                
                # self.get_logger().info(f"{data['image0'].squeeze().cpu().numpy().shape}")
                self.createMatchPrevShow(self.previmg,
                                         data['keypoints0'], 
                                         img, 
                                         data['keypoints1'],
                                         matches0,
                                         matches1)
                # self.get_logger().info(f'matches0:{matches0}, matches1:{matches1}')
                # self.get_logger().info(f'scores1:{scores0}, scores1:{scores1}')
                response.id = request['id']
                response.frame = request['frame']
                response.dim = 256
                response.kpts = extract.get('keypoints', {})[0].reshape((-1)).tolist()
                response.scores = extract.get('scores', {})[0].reshape((-1)).tolist()
                response.descriptors = extract.get('descriptors', {})[0].reshape((-1)).tolist()
                response.match_prev_id = matches1[0].reshape((-1)).tolist()
                response.match_prev_score = scores1[0].reshape((-1)).tolist()

            self.prev_extract = {k+'0': v for k, v in extract.items()}
            self.prev_extract['image0'] = img1
            self.previmg = img
            self.pub_match.publish(response)
def main(args=None):
    rclpy.init(args=args)
    node = SPSG()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
