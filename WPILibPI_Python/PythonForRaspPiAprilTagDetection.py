#!/usr/bin/env python3

from cscore import CameraServer
from ntcore import NetworkTableInstance, EventFlags

import cv2
import json
import numpy as np
import time

import robotpy_apriltag
from wpimath.geometry import Transform3d
import math

team = 9082
server = False


def main():
    with open('/boot/frc.json') as f:
        config = json.load(f)
    camera = config['cameras'][0]
    camera2 = config['cameras'][1]

    width = camera['width']
    height = camera['height']
    CameraServer.startAutomaticCapture()
    input_stream = CameraServer.getVideo(camera)
    input_stream2 = CameraServer.getVideo(camera2)
    #input_stream = CameraServer.getVideo()
    output_stream = CameraServer.putVideo('Processed', width, height)
    output_stream2 = CameraServer.putVideo('Camera2_Processed', width, height)
    img = np.zeros(shape=(height, width, 3), dtype=np.uint8)
    img2 = np.zeros(shape = (height, width, 3), dtype=np.uint8)

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    # Table for vision output information
    vision_nt = ntinst.getTable('Vision')

    # Wait for NetworkTables to start
    time.sleep(0.5)

    prev_time = time.time()
    while True:
        start_time = time.time()

        frame_time, input_img = input_stream.grabFrame(img)
        input_img2 = input_stream2.grabFrame(img2)
        output_img = np.copy(input_img)
        output_img2 = np.copy(input_img2)

        # Coordinates of found targets, for NT output:
        x_list = []
        y_list = []
        z_list = []
        id_list = []

        # Notify output of error and skip iteration
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue

        # April Tag detection:
        detector = robotpy_apriltag.AprilTagDetector()
        detector.addFamily("tag36h11")
        
        estimator = robotpy_apriltag.AprilTagPoseEstimator(
            robotpy_apriltag.AprilTagPoseEstimator.Config(
                0.1524, 699.3778103158814, 677.7161226395344, 345.6059345433618, 207.12741326228522
            )
        )
        # Detect apriltag
        DETECTION_MARGIN_THRESHOLD = 80
        DETECTION_ITERATIONS = 50

        gray = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
        tag_info = detector.detect(gray)
        filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]

        for tag in filter_tags:

            est = estimator.estimateOrthogonalIteration(tag, DETECTION_ITERATIONS)
            pose = est.pose1
            tag_id = tag.getId()
            center = tag.getCenter()

            print(f"{tag_id}: {pose}")
            
            # Highlight the edges of all recognized tags and label them with their IDs:
            if ((tag_id > 0) & (tag_id < 17)):
                col_box = (0,255,0)
                col_txt = (255,255,255)
            else:
                col_box = (0,0,255)
                col_txt = (0,255,255)

            # Draw a frame around the tag:
            corner0 = (int(tag.getCorner(0).x), int(tag.getCorner(0).y))
            corner1 = (int(tag.getCorner(1).x), int(tag.getCorner(1).y))
            corner2 = (int(tag.getCorner(2).x), int(tag.getCorner(2).y))
            corner3 = (int(tag.getCorner(3).x), int(tag.getCorner(3).y))
            cv2.line(output_img, corner0, corner1, color = col_box, thickness = 2)
            cv2.line(output_img, corner1, corner2, color = col_box, thickness = 2)
            cv2.line(output_img, corner2, corner3, color = col_box, thickness = 2)
            cv2.line(output_img, corner3, corner0, color = col_box, thickness = 2)
            # Label the tag with the ID:
            cv2.putText(output_img, f"{tag_id}", (int(center.x), int(center.y)), cv2.FONT_HERSHEY_SIMPLEX, 1, col_txt, 2)

            x_list.append(pose.x_feet*((4)/15.2))
            y_list.append(pose.y_feet*((4)/15.2))
            z_list.append(pose.z_feet*((4)/15.2))
            id_list.append(tag_id)

        vision_nt.putNumberArray('target_x', x_list)
        vision_nt.putNumberArray('target_y', y_list)
        vision_nt.putNumberArray('target_z', z_list)
        vision_nt.putNumberArray('target_id', id_list)

        processing_time = start_time - prev_time
        prev_time = start_time
    
        fps = 1 / processing_time
        cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        output_stream.putFrame(output_img)
        output_stream2.putFrame(output_img2)
        output_stream.setResolution(1280,720)
        output_stream2.setResolution(1280,720)


main()