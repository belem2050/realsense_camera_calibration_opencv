
import numpy as np
import cv2
import os
import time

import pyrealsense2 as rs
import argparse

class CameraCalibration():
    def __init__(self) -> None:

        self.board_size = (10,8)
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((self.board_size[0]*self.board_size[1],3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.board_size[0],0:self.board_size[1]].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.
        self.captured_images_dir = "captured_images"
        self.corners_found_images_dir = "corners_images"


    def rs_setup(self):

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)


    def save_image(self, color_image, image_number, dir):
        if not os.path.exists(dir):
            os.makedirs(dir)

        filename = os.path.join(dir,f"image_{image_number}.png")
        cv2.imwrite(filename, color_image)

    def remove_dir(self):
        if os.path.exists(self.captured_images_dir):
            os.removedirs(self.captured_images_dir)


    def images_aquision(self, images_number, delay):   
        
        image_number = 1    
        try:
            self.pipeline.start(self.config)

            while(image_number <=images_number):

                frame = self.pipeline.wait_for_frames()
                color_frame  = frame.get_color_frame()
                color_image = np.asanyarray(color_frame.get_data())
                self.save_image(color_image, image_number, self.captured_images_dir)

                print(f"image {image_number}")
                time.sleep(delay)
                image_number += 1
            self.pipeline.stop()
        except :
            print("Error")


    def calibrate(self):
    
        captured_images = os.listdir(self.captured_images_dir)
        image_number = 0
    
       
        for captured_image in captured_images:
            filename = os.path.join(self.captured_images_dir,captured_image)
            #print(filename)

            image = cv2.imread(filename)
            #print(captured_image)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, self.board_size, None)
            #print(f"found: {found}")

            if found:
                self.objpoints.append(self.objp)
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
                self.imgpoints.append(corners2)
                # Draw and display the corners
                cv2.drawChessboardCorners(image, self.board_size, corners2, True)
                cv2.imshow(f"{captured_image}", image)
                self.save_image(image, image_number, self.corners_found_images_dir)
                cv2.waitKey(1000)
        #print(self.objpoints)
                image_number += 1

        #calibrate the camera
        try :
            ret, mtx, dist, rvecs ,tvecs= cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)
            print(f"ret: {ret}\n")
            print(f"mtx:\n {mtx}\n")
            print(f"dist: {dist}\n")
            #print("stdDeviationsIntrinsics: ", stdDeviationsIntrinsics)
            #return mtx, dist, stdDeviationsIntrinsics
            return ret, mtx, dist, rvecs, tvecs
        except :
            print("Not calibrated")


    def get_serial_number(self):
        try:
            context = rs.context()
            devices = context.query_devices()
            if len(devices) == 0:
                raise Exception("No RealSense devices found.")
            else:
                serial_number = devices[0].get_info(rs.camera_info.serial_number)
                return serial_number
        except Exception as e:
            print("Error:", e)
            return None

        

def main():

        parser = argparse.ArgumentParser(description='Arguments for calibration script run')
        parser.add_argument("--i", type=str, help="set True if images acquisition need for calibration ", default="False")
        parser.add_argument("--n", type=int, help="number of images to acquire", default=10)
        parser.add_argument("--d", type=int, help="delay between images pitchuring", default=2)  
        parser.add_argument("--serial_number", type=str, help="delay between images pitchuring", default="False")  


        
        cameraCalib =CameraCalibration()
        args = parser.parse_args()
        
        if args.serial_number == "True" :
            print(f"serial number : {cameraCalib.get_serial_number()}")
            return

        if  args.i == "False" :
            cameraCalib.calibrate()
            return

        cameraCalib.rs_setup()
        cameraCalib.images_aquision(args.n, args.d)
        cameraCalib.calibrate()


if __name__ =="__main__":
    main()
