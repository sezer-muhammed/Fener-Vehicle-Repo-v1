#!usr/bin/python3
import rclpy
import jetson.inference
import jetson.utils
import time
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

class DistanceEstimator:
    def __init__(self):
        self.FPS = 10
        self.u = 0.3
        self.std_acc = 2
        self.std_meas = 0.4
        self.diff = 0
        self.estimatedDiff = 2
        self.distance = [[0], [0]]
        self.SevvalinVerecegiKatsayi = 5000
        self.kf = KalmanFilter(1/self.FPS, self.u, self.std_acc, self.std_meas)
        self.kf.update(20)
        self.estimatedDiff = self.kf.predict()

    def resize(self, img, resize_factor):
        resized_img = jetson.utils.cudaAllocMapped(width=img.width * resize_factor[0], height=img.height * resize_factor[1], format=img.format)
        jetson.utils.cudaResize(img, resized_img)
        return resized_img

    def crop(self, img, cropRoi):
        width = cropRoi[2] - cropRoi[0]
        height = cropRoi[3] - cropRoi[1]
        crop_img = jetson.utils.cudaAllocMapped(width=width, height=height, format=img.format)
        jetson.utils.cudaCrop(img, crop_img, cropRoi)
        return crop_img

    def getData(self, frameSag, frameSol, koords):
        self.sagFrame = frameSag
        self.solFrame = frameSol
        self.koords = koords

    def findDiff(self):
        sagROI = self.crop(self.sagFrame, self.koords)
        resize_factor = 150 / (self.koords[2] - self.koords[0])
        if resize_factor > 1.2:
            resize_factor = 1.2
        x_new = min(1639, self.koords[2] + int((self.koords[2] - self.koords[0]) * 1.3))
        y_min_new = max(0, self.koords[1] - 5)
        y_max_new = min(921, self.koords[3] + 25)
        solROI = self.crop(self.solFrame, [self.koords[0], y_min_new, x_new, y_max_new])
        sagROI = self.resize(sagROI, [resize_factor, resize_factor])
        solROI = self.resize(solROI, [resize_factor, resize_factor])
        solROI = jetson.utils.cudaToNumpy(solROI)
        sagROI = jetson.utils.cudaToNumpy(sagROI)
        #cv2.imshow("SAG", sagROI)
        #cv2.waitKey(1)
        jetson.utils.cudaDeviceSynchronize()
        res = cv2.matchTemplate(solROI, sagROI, cv2.TM_CCOEFF)
        _, _, _, top_left = cv2.minMaxLoc(res)
        self.diff = (abs(self.koords[0] - x_new - top_left[0]) / resize_factor)
    def predict(self):
        self.estimatedDiff = self.kf.predict()
        self.kf.update(self.diff)

    def convertDistance(self):
        self.distance = self.SevvalinVerecegiKatsayi / float(self.estimatedDiff[0][0])
        self.distance = self.regress(self.distance)
    def regress(self, x):
        terms = [
             3.1397915273014743e+001,
             4.1097531718172640e+001,
            -2.1871988872633099e+000,
             5.8809468456784655e-002,
            -4.0138771063754920e-004
        ]
        t = 1
        r = 0
        for c in terms:
            r += c * t
            t *= x
        return r

    def estimateAngleAndDistance(self):
        if self.koords[0] > 50 and self.koords[1] < 1590:
            self.findDiff()
            self.predict()
            self.convertDistance()
            return self.distance, (self.koords[2] + self.koords[0] - 1640)//2
        else:
            return -999, -999

class KalmanFilter(object):
    def __init__(self, dt, u, std_acc, std_meas):
        self.dt = dt
        self.u = u
        self.std_acc = std_acc
        self.A = np.matrix([[1, self.dt],
                            [0, 1]])
        self.B = np.matrix([[(self.dt**2)/2], [self.dt]])
        self.H = np.matrix([[1, 0]])
        self.Q = np.matrix([[(self.dt**4)/4, (self.dt**3)/2],
                            [(self.dt**3)/2, self.dt**2]]) * self.std_acc**2
        self.R = std_meas**2
        self.P = np.eye(self.A.shape[1])
        self.x = np.matrix([[0], [0]])

    def predict(self):
        # Ref :Eq.(9) and Eq.(10)
        # Update time state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        # Calculate error covariance
        # P= A*P*A' + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        # Ref :Eq.(11) , Eq.(11) and Eq.(13)
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Eq.(11)
        self.x = np.round(
            self.x + np.dot(K, (z - np.dot(self.H, self.x))))  # Eq.(12)
        I = np.eye(self.H.shape[1])
        self.P = (I - (K * self.H)) * self.P  # Eq.(13)

class Detector:
    def __init__(self):
        self.net = jetson.inference.detectNet(argv=['--model=/home/fener/ros2_fener/src/fener_package/fener_package/ssd-mobilenet.onnx', '--labels=/home/fener/ros2_fener/src/fener_package/fener_package/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'])
        self.h, self.w, self.c = 922, 1640, 3
        self.lastCoord = [0, 0, self.w, self.h]
        self.coord = [0, 0, self.w, self.h]
        self.coordsToDetect = [0, 0, self.w, self.h]
        self.conf = 5


    def Detect(self, frame):
        self.frame = frame
        self.prepareFrame()
        self.detections = self.net.Detect(self.frame)
        self.boxes = []
        for detected in self.detections:
            self.boxes.append(int(detected.Left))
            self.boxes.append(int(detected.Top))
            self.boxes.append(int(detected.Right))
            self.boxes.append(int(detected.Bottom))
            break
        if np.size(self.boxes):
            if np.min(self.boxes) < 0:
                self.boxes = self.boxes - np.min(self.boxes)
            self.boxes[0] += self.coordsToDetect[0]
            self.boxes[2] += self.coordsToDetect[0]
            self.boxes[1] += self.coordsToDetect[1]
            self.boxes[3] += self.coordsToDetect[1]
            self.coord = self.boxes
            self.lastCoord = self.boxes
            self.conf = max(0, self.conf - 2)
        else:
            self.coord = np.array([-1, 0, 0, 0], np.int16)
            self.conf = min(5, self.conf + 1)
        if self.conf < 5 or (self.coord[2] - self.coord[0] > 10):
            return [self.coord[0], self.coord[1],
                    self.coord[2], self.coord[3]]
        else:
            return [-1, 0, 0, 0]

    def prepareFrame(self):
        if self.conf == 5:
            self.coordsToDetect[0] = 0
            self.coordsToDetect[1] = 0
            self.coordsToDetect[2] = self.w
            self.coordsToDetect[3] = self.h
            return
        else:
            maxGen = max(
                self.lastCoord[2] - self.lastCoord[0], self.lastCoord[3] - self.lastCoord[1])
            objCen = [(self.lastCoord[0] + self.lastCoord[2])//2,
                      (self.lastCoord[1] + self.lastCoord[3])//2]
            minCropGen = min(maxGen + 160, min(self.h, self.w))
            maxCropGen = min(self.w, self.h)
            cropCoef = (maxCropGen - minCropGen) // 25
            cropGen = (minCropGen + cropCoef * self.conf * self.conf)//2
            self.coordsToDetect = [objCen[0] - cropGen, objCen[1] -
                                   cropGen, objCen[0] + cropGen, objCen[1] + cropGen]
            if self.coordsToDetect[0] < 0:
                self.coordsToDetect[2] -= self.coordsToDetect[0]
                self.coordsToDetect[0] = 0 
            if self.coordsToDetect[1] < 0:
                self.coordsToDetect[3] -= self.coordsToDetect[1]
                self.coordsToDetect[1] = 0 

            if self.coordsToDetect[2] >= self.w:
                self.coordsToDetect[0] -= self.coordsToDetect[2] - self.w
                self.coordsToDetect[2] = self.w - 1
            if self.coordsToDetect[3] >= self.h:
                self.coordsToDetect[1] -= self.coordsToDetect[3] - self.h
                self.coordsToDetect[3] = self.h - 1
            self.frame = self.crop(self.frame, self.coordsToDetect)
            return

    def crop(self, img, cropRoi):
        width = cropRoi[2] - cropRoi[0]
        height = cropRoi[3] - cropRoi[1]
        crop_img = jetson.utils.cudaAllocMapped(width=width, height=height, format=img.format)
        jetson.utils.cudaCrop(img, crop_img, cropRoi)
        return crop_img

def main():
    print('Obje Tespiti Node Başladı')
    rclpy.init()
    node = rclpy.create_node("distance_and_angle_of_object")
    publisher = node.create_publisher(Float32MultiArray ,"object/distance_angle", 1)
    veri = Float32MultiArray()
    det = Detector()
    #net = jetson.inference.detectNet(argv=['--model=/home/jetson/Desktop/canta_bir.onnx', '--labels=/home/jetson/Desktop/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'])
    camera_left = jetson.utils.videoSource("csi://1")      # '/dev/video0' for V4L2
    camera_right = jetson.utils.videoSource("csi://0")  
    estimator = DistanceEstimator()
    while True:
        img_right = camera_right.Capture()
        img = camera_left.Capture()
        coords = det.Detect(img_right)
        estimator.getData(img_right, img, coords)
        res = estimator.estimateAngleAndDistance()
        veri = Float32MultiArray(data = res)
        publisher.publish(veri)




if __name__ == '__main__':
    main()
