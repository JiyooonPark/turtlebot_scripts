#!/usr/bin/env python3
import argparse
import os
import sys
import time
import warnings

import cv2
import rospy
import torch
from std_msgs.msg import String

sys.path.append(os.path.join(os.path.dirname(__file__), 'thirdparty/fast-reid'))

from detector import build_detector
from deep_sort import build_tracker
from utils.draw import draw_boxes
from utils.parser import get_config
from utils.log import get_logger
from utils.io import write_results

publish_text = ''


def talker():
    pub = rospy.Publisher('deepsort_poition_topic', String, queue_size=10)
    rospy.init_node('deepsort_poition_node', anonymous=True)
    rate = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(publish_text)
        pub.publish(publish_text)
        rate.sleep()


class VideoTracker(object):
    def __init__(self, pub, cfg, args, video_path):
        self.cfg = cfg
        self.args = args
        self.video_path = video_path
        self.logger = get_logger("root")
        self.pub = pub

        use_cuda = args.use_cuda and torch.cuda.is_available()
        if not use_cuda:
            warnings.warn("Running in cpu mode which maybe very slow!", UserWarning)

        if args.display:
            cv2.namedWindow("test", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("test", args.display_width, args.display_height)

        if args.cam != -1:
            print("Using webcam " + str(args.cam))
            self.vdo = cv2.VideoCapture(args.cam)
        else:
            self.vdo = cv2.VideoCapture()
        self.detector = build_detector(cfg, use_cuda=use_cuda)
        self.deepsort = build_tracker(cfg, use_cuda=use_cuda)
        self.class_names = self.detector.class_names

    def __enter__(self):
        if self.args.cam != -1:
            ret, frame = self.vdo.read()
            assert ret, "Error: Camera error"
            self.im_width = frame.shape[0]
            self.im_height = frame.shape[1]

        else:
            assert os.path.isfile(self.video_path), "Path error"
            self.vdo.open(self.video_path)
            self.im_width = int(self.vdo.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.im_height = int(self.vdo.get(cv2.CAP_PROP_FRAME_HEIGHT))
            assert self.vdo.isOpened()

        if self.args.save_path:
            os.makedirs(self.args.save_path, exist_ok=True)

            # path of saved video and results
            self.save_video_path = os.path.join(self.args.save_path, "results.avi")
            self.save_results_path = os.path.join(self.args.save_path, "results.txt")

            # create video writer
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.writer = cv2.VideoWriter(self.save_video_path, fourcc, 20, (self.im_width, self.im_height))

            # logging
            self.logger.info("Save results to {}".format(self.args.save_path))

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        if exc_type:
            print(exc_type, exc_value, exc_traceback)

    def run(self):
        global publish_text
        results = []
        idx_frame = 0
        while self.vdo.grab():
            idx_frame += 1
            if idx_frame % self.args.frame_interval:
                continue

            start = time.time()
            _, ori_im = self.vdo.retrieve()
            ori_im = cv2.flip(ori_im, 1)
            height, width = ori_im.shape[:2]
            im = cv2.cvtColor(ori_im, cv2.COLOR_BGR2RGB)

            # do detection
            bbox_xywh, cls_conf, cls_ids = self.detector(im)

            # select person class
            mask = cls_ids == 0

            bbox_xywh = bbox_xywh[mask]
            # bbox dilation just in case bbox too small, delete this line if using a better pedestrian detector
            bbox_xywh[:, 3:] *= 1.2
            cls_conf = cls_conf[mask]

            # do tracking
            outputs = self.deepsort.update(bbox_xywh, cls_conf, im)

            # draw boxes for visualization
            if len(outputs) > 0:
                bbox_tlwh = []
                bbox_xyxy = outputs[:, :4]
                identities = outputs[:, -1]
                ori_im = draw_boxes(ori_im, bbox_xyxy, identities)

                for bb_xyxy in bbox_xyxy:
                    bbox_tlwh.append(self.deepsort._xyxy_to_tlwh(bb_xyxy))
                    print('====================')
                    print(bb_xyxy[0] / width, bb_xyxy[1] / height)
                    cv2.putText(ori_im, f'{round(bb_xyxy[0] / width * 100, 2)}, {round(bb_xyxy[1] / height * 100, 2)}',
                                (bb_xyxy[0], bb_xyxy[1]), cv2.FONT_HERSHEY_PLAIN, 1, (100, 0, 50), 2)
                    cv2.putText(ori_im, f'{round(bb_xyxy[2] / width * 100, 2)}, {round(bb_xyxy[3] / height * 100, 2)}',
                                (bb_xyxy[2], bb_xyxy[3]), cv2.FONT_HERSHEY_PLAIN, 1, (100, 0, 50), 2)
                    size = ((bb_xyxy[2] - bb_xyxy[0]) * (bb_xyxy[0] - bb_xyxy[3])) / (width * height) * 100
                publish_text = f'{round((bb_xyxy[0] + bb_xyxy[2]) / (2 * width) * 100, 2)},{round((bb_xyxy[1] + bb_xyxy[3]) / (2 * height) * 100, 2)}, {-size}'
                pub.publish(publish_text)
                results.append((idx_frame - 1, bbox_tlwh, identities))

            end = time.time()

            if self.args.display:
                cv2.imshow("test", ori_im)
                cv2.waitKey(1)

            if self.args.save_path:
                self.writer.write(ori_im)

            # save results
            write_results(self.save_results_path, results, 'mot')

            # logging
            self.logger.info("time: {:.03f}s, fps: {:.03f}, detection numbers: {}, tracking numbers: {}" \
                             .format(end - start, 1 / (end - start), bbox_xywh.shape[0], len(outputs)))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("VIDEO_PATH", type=str)
    parser.add_argument("--config_mmdetection", type=str, default="./configs/mmdet.yaml")
    parser.add_argument("--config_detection", type=str, default="./configs/yolov3.yaml")
    parser.add_argument("--config_deepsort", type=str, default="./configs/deep_sort.yaml")
    parser.add_argument("--config_fastreid", type=str, default="./configs/fastreid.yaml")
    parser.add_argument("--fastreid", action="store_true")
    parser.add_argument("--mmdet", action="store_true")
    # parser.add_argument("--ignore_display", dest="display", action="store_false", default=True)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--frame_interval", type=int, default=1)
    parser.add_argument("--display_width", type=int, default=800)
    parser.add_argument("--display_height", type=int, default=600)
    parser.add_argument("--save_path", type=str, default="./output/")
    parser.add_argument("--cpu", dest="use_cuda", action="store_false", default=True)
    parser.add_argument("--camera", action="store", dest="cam", type=int, default="-1")
    return parser.parse_args()


if __name__ == "__main__":
    pub = rospy.Publisher('deepsort_poition_topic', String, queue_size=10)
    rospy.init_node('deepsort_poition_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    args = parse_args()
    cfg = get_config()
    if args.mmdet:
        cfg.merge_from_file(args.config_mmdetection)
        cfg.USE_MMDET = True
    else:
        cfg.merge_from_file(args.config_detection)
        cfg.USE_MMDET = False
    cfg.merge_from_file(args.config_deepsort)
    if args.fastreid:
        cfg.merge_from_file(args.config_fastreid)
        cfg.USE_FASTREID = True
    else:
        cfg.USE_FASTREID = False
    # try:
    with VideoTracker(pub, cfg, args, video_path=args.VIDEO_PATH) as vdo_trk:
        vdo_trk.run()
    # except rospy.ROSInterruptException:
    #     pass
