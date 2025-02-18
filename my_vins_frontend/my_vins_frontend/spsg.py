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
    max_track = 150
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
            'max_keypoints': 4 * max_track,
            'remove_borders': 4,
        }
    }
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    request_list = []
    previmg = []
    prev_rqt = []
    first_frame_published = False

    pdata = []
    ppdata = []


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

    def featureMatchPrevCallback(self, request):
        with self.lock_rqt_list:
            self.request_list.append({
                'frame' : request.frame,
                'id' : request.id,
                'image' : request.img
            })
        with self.cv_lock_rqt:
            self.cv_lock_rqt.notify()


    def track_update(self, data0, data1, keeped_size):
        matches1 = data1["matches"][0][0]
        match_scores1 = data1["match_scores"][0][0]
        kpts1 = data1['kpts'][0]
        dsp1 = data1['dsps'][0].permute(1, 0)
        scores1 = data1['scores'][0]
        # keeped_size = min(keeped_size, torch.sum(matches1 != -1))
        
        # if not isinstance(self.ppdata, list):

            # 证明这个至少是第三帧
        mask0 = torch.logical_and(matches1 >= 0, matches1 < keeped_size)
        mask1 = torch.logical_or(matches1 < 0, matches1 >= keeped_size)
        # print(int(torch.nonzero(mask0, as_tuple=True)[0].size()[0]))
        last_match0 = torch.nonzero(mask0, as_tuple=True)[0]
        last_match1 = torch.nonzero(mask1, as_tuple=True)[0]
        last_match0_score = match_scores1[mask0]
        last_match1_score = match_scores1[mask1]
        mask01 = torch.argsort(last_match0_score, descending=True)
        mask11 = torch.argsort(last_match1_score, descending=True)
        front_match = last_match0[mask01]
        back_match = last_match1[mask11]
        mask = torch.cat((front_match, back_match),dim=0)

        # mask = torch.cat((mask01, mask11),dim=0)
        sorted_matches1 = matches1[mask]
        sorted_pts = kpts1[mask]
        sorted_dsp = dsp1[mask]
        sorted_socres = scores1[mask]
        sorted_match_scores = match_scores1[mask]
        ret = {'kpts': [sorted_pts], 
                'dsps': [sorted_dsp.permute(1,0)], 
                'scores': [sorted_socres], 
                'matches': [torch.tensor([sorted_matches1.tolist()])], 
                'match_scores': [torch.tensor([sorted_match_scores.tolist()])],
                'track_size': int(torch.nonzero(mask0, as_tuple=True)[0].size()[0])}
        return ret
        # else:
        #     # 第二帧
        #     sorted_indices = torch.argsort(match_scores1, descending=True)  # 按得分降序排序
        #     sorted_matches1 = matches1[sorted_indices]
        #     sorted_pts = kpts1[sorted_indices]
        #     sorted_dsp = dsp1[sorted_indices]
        #     sorted_match_scores = match_scores1[sorted_indices]
        #     sorted_socres = scores1[sorted_indices]
        #     ret = {'kpts': [sorted_pts], 
        #            'dsps': [sorted_dsp.permute(1,0)], 
        #            'scores': [sorted_socres], 
        #            'matches': [torch.tensor([sorted_matches1.tolist()])], 
        #            'match_scores': [torch.tensor([sorted_match_scores.tolist()])],
        #            'track_size': keeped_size}
        #     return ret

    def handleRequests(self):
        while True:
            with self.cv_lock_rqt:
                self.cv_lock_rqt.wait_for(lambda: len(self.request_list) > 0)
            with self.lock_rqt_list:
                request = self.request_list.pop(0)
            img = self.bridge.imgmsg_to_cv2(request['image'])
            if len(img.shape) != 2:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img1 = frame2tensor(img, self.device)

            extract = self.superpoint({'image': img1})
            # print(extract)
            
            data = {'kpts': extract['keypoints'], 'scores': extract['scores'], 'dsps': extract['descriptors'], 'image': img1, 'track_size': self.max_track}
            if isinstance(self.pdata, list):
                # 第一帧
                pass
            else:
                match_data = {
                    'keypoints0': self.pdata['kpts'],
                    'scores0': self.pdata['scores'],
                    'descriptors0': self.pdata['dsps'],
                    'image0': self.pdata['image'],
                    **{k+'1': v for k, v in extract.items()},
                    'image1':img1
                }

                for k in match_data:
                    if isinstance(match_data[k], (list, tuple)):
                        match_data[k] = torch.stack(match_data[k])
                        # self.get_logger().info(f'{k}:{data[k].shape}')

                # print(match_data)
                result = self.superglue(match_data)
                data['matches'] = [result['matches1']]
                data['match_scores'] = [result['matching_scores1']]

                data = self.track_update(self.pdata, data, self.max_track)
                # print(data['track_size'], flush=True)
                data['image'] = img1
                # print(data['matches'], flush=True)
                
                self.createMatchPrevShow(self.previmg,
                                         self.pdata['kpts'], 
                                         img, 
                                         data['kpts'],
                                         data['matches'])

                img_show_track = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                pts_int = torch.round(data['kpts'][0]).int().cpu().squeeze()
                
                for idx, pt in enumerate(torch.split(pts_int, 1, dim=0)):
                    if (idx < data['track_size']):
                        cv2.circle(img_show_track, (pt[0,0].item(), pt[0,1].item()), 3, (0,255,0))
                    elif (idx < self.max_track):
                        cv2.circle(img_show_track, (pt[0,0].item(), pt[0,1].item()), 3, (0,0,255))
                
                msg = self.bridge.cv2_to_imgmsg(img_show_track)
                self.pub_track.publish(msg)
                # self.get_logger().info(f'matches0:{matches0}, matches1:{matches1}')
                # self.get_logger().info(f'scores1:{scores0}, scores1:{scores1}')
                if isinstance(self.ppdata, list):
                    response0 = FeatureMatchPrevResponse()
                    response0.id = self.prev_rqt['id']
                    response0.frame = self.prev_rqt['frame']
                    response0.dim = 256
                    response0.kpts = self.pdata['kpts'][0][:self.max_track].reshape((-1)).tolist()
                    response0.scores = self.pdata['scores'][0][:self.max_track].reshape((-1)).tolist()
                    response0.descriptors = self.pdata['dsps'][0].permute(1,0)[:self.max_track].reshape((-1)).tolist()
                    # print(self.pdata['dsps'][0].permute(1,0)[:self.max_track].shape)
                    response0.match_prev_id = np.full((len(response0.kpts) // 2, 1), -1, dtype=np.int32).reshape((-1)).tolist()
                    response0.match_prev_score = np.full((len(response0.kpts) // 2, 1), 0, dtype=np.float32).reshape((-1)).tolist()

                    # print(response0.id)
                    # print(response0.frame)
                    # print(response0.dim)
                    # print(len(response0.kpts))
                    # print(len(response0.scores))
                    # print(len(response0.descriptors))
                    # # print(data['dsps'][0].permute(1,0)[:self.max_track].shape)
                    # print(len(response0.match_prev_id))
                    # print(len(response0.match_prev_score))
                    self.pub_match.publish(response0)


                response = FeatureMatchPrevResponse()
                response.id = request['id']
                # print("==")
                # print(response.id)
                response.frame = request['frame']
                # print(response.frame)
                response.dim = 256
                # print(response.dim)
                response.kpts = data['kpts'][0][:self.max_track].reshape((-1)).tolist()
                # print(len(response.kpts))
                response.scores = data['scores'][0][:self.max_track].reshape((-1)).tolist()
                # print(len(response.scores))
                response.descriptors = data['dsps'][0].permute(1,0)[:self.max_track].reshape((-1)).tolist()
                # print(len(response.descriptors))
                # # print(data['dsps'][0].permute(1,0)[:self.max_track].shape)
                match = data['matches'][0][0][:data['track_size']]
                match = torch.nn.functional.pad(match, (0, self.max_track - data['track_size']), value=-1)
                response.match_prev_id = match.reshape((-1)).tolist()
                # print(response.match_prev_id)
                # print(len(response.match_prev_id))
                response.match_prev_score = data['match_scores'][0][0][:self.max_track].reshape((-1)).tolist()
                # print(len(response.match_prev_score))
                self.pub_match.publish(response)
                
            # self.pextract = {k+'0': v for k, v in extract.items()}
            # self.pextract['image0'] = img1
            self.previmg = img
            self.ppdata = self.pdata
            self.pdata = data
            self.prev_rqt = request

def main(args=None):
    rclpy.init(args=args)
    node = SPSG()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

