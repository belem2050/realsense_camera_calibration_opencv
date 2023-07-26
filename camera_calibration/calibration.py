
import numpy as np
import cv2
import sys
import os
import time

import pyrealsense2 as rs

t =[]

class CameraCalibration():
    def __init__(self) -> None:

        self.board_size = (11,8)
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((self.board_size[0]*self.board_size[1],3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.board_size[0],0:self.board_size[1]].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.
        self.captured_images_dir = "captured_images"


    def rs_setup(self):

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)


    def save_image(self, color_image, image_number):

        if not os.path.exists(self.captured_images_dir):
            os.makedirs(self.captured_images_dir)

        filename = os.path.join(self.captured_images_dir,f"image_{image_number}.png")
        cv2.imwrite(filename, color_image)
        print(image_number)



    def images_aquision(self, images_number, delay):   
        
        image_number = 1     
        try:
            self.pipeline.start(self.config)

            while(image_number <=images_number):

                frame = self.pipeline.wait_for_frames()
                color_frame  = frame.get_color_frame()
                color_image = np.asanyarray(color_frame.get_data())
                self.save_image(color_image, image_number)
        
                time.sleep(delay)
                image_number += 1

            self.pipeline.stop()

        except :
            print("Error")


    def calibration(self):
    
        captured_images = os.listdir(self.captured_images_dir)
       
        for captured_image in captured_images:
            filename = os.path.join(self.captured_images_dir,captured_image)
            print(filename)
            image = cv2.imread(filename)
            #print(captured_image)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, self.board_size, None)
            print(f"found: {found}")

            if found:
                self.objpoints.append(self.objp)
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
                self.imgpoints.append(corners2)
                # Draw and display the corners
                cv2.drawChessboardCorners(image, self.board_size, corners2, True)
                cv2.imshow(f"{captured_image}", image)
                cv2.waitKey()
            print(self.objpoints)

        #calibrate the camera
        ret, mtx, dist, rvecs, tvecs , stdDeviationsIntrinsics= cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)
        # print("ret: ", ret)
        print("mtx: ", mtx)
        print("dist: ", dist)
        print("stdDeviationsIntrinsics: ", stdDeviationsIntrinsics)

        return mtx, dist

def main():
        cameraCalib =CameraCalibration()
        #cameraCalib.rs_setup()
        #cameraCalib.images_aquision(images_number=10, delay=0.5)
        cameraCalib.calibration()


if __name__ =="__main__":
    main()
