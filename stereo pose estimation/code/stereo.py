# %% Imports

import cv2
import numpy as np
import glob
import yaml
import os
from numpy.linalg import inv
from matplotlib import pyplot as plt


# %% Class and function definitions

class Calibration:
    """
    Class for all stereo calibration related operations
    """

    def __init__(self, left_file, right_file):
        """
        Initialization function
        """
        print("Initializing calibration object!")
        self.left_file = left_file
        self.right_file = right_file
        self.left_height = 0
        self.left_width = 0
        self.right_height = 0
        self.right_width = 0
        self.left_K = []
        self.left_D = []
        self.right_K = []
        self.right_D = []
        self.extrinsics_R = []
        self.extrinsics_T = []
        self.load_calibration()

    # Parse camera calibration yaml file - intrinsics
    def load_intrinsics(self, calib_data):
        """
        Load EUROC camera intrinsic data
        Taken from: https://github.com/lrse/sptam
        """
        width, height = calib_data['resolution']
        D = np.array(calib_data['distortion_coefficients'])
        fx, fy, px, py = calib_data['intrinsics']
        K = np.array([[fx, 0, px],
                      [0, fy, py],
                      [0, 0, 1]])
        return height, width, K, D

    # Parse camera calibration yaml file - extrinsics
    def load_extrinsics(self, calib_data):
        """
        Load EUROC stereo extrinsics data
        Taken from: https://github.com/lrse/sptam
        """
        # read homogeneous rotation and translation matrix
        transformation_base_camera = np.array(calib_data['T_BS']['data'])
        transformation_base_camera = transformation_base_camera.reshape((4, 4))
        return transformation_base_camera

    # Read calibration file into single structure
    def load_calibration(self):
        """
        Load calibration data into self object
        """
        # Open .yaml files
        left_calib = open(self.left_file, 'r')
        left_calib_data = yaml.load(left_calib, Loader=yaml.FullLoader)
        right_calib = open(self.right_file, 'r')
        right_calib_data = yaml.load(right_calib, Loader=yaml.FullLoader)

        # Parse yaml contents - intrinsics
        self.left_height, self.left_width, self.left_K, self.left_D \
            = self.load_intrinsics(left_calib_data)
        self.right_height, self.right_width, self.right_K, self.right_D \
            = self.load_intrinsics(right_calib_data)

        # Parse yaml contents - extrinsics
        tr_base_left = self.load_extrinsics(left_calib_data)
        tr_base_right = self.load_extrinsics(right_calib_data)

        # Calculate transformation from L_camera to R_camera
        tr_right_base = inv(tr_base_right)
        tr_right_left = tr_right_base.dot(tr_base_left)

        # Assign extrinsics to class
        self.extrinsics_R = tr_right_left[0:3, 0:3]
        self.extrinsics_T = tr_right_left[0:3, 3]

    # Display camera intrinsics
    def display_intrinsics(self, camera):
        """
        Print camera intrinsics
        """
        if camera == "left":
            print("==== Left Camera ====")
            print("Height : {}".format(self.left_height))
            print("Width : {}".format(self.left_width))
            print("K : {}".format(self.left_K))
            print("D : {}".format(self.left_D))
        elif camera == "right":
            print("==== Right Camera ====")
            print("Height : {}".format(self.right_height))
            print("Width : {}".format(self.right_width))
            print("K : {}".format(self.right_K))
            print("D : {}".format(self.right_D))
        else:
            print("Use option 'left' or 'right' only!")
            exit()

    # Display camera extrinsics
    def display_extrinsics(self):
        """
        Print camera extrinsics
        """
        print("==== Camera Extrinsics ====")
        print("Rotation: {}".format(self.extrinsics_R))
        print("Translation: {}".format(self.extrinsics_T))

    def apply_rectification(self):
        """
        Function to apply camera and image rectification
        """
        # Calculate post-rectification matrices
        R_left_new = np.empty([3, 3])
        R_right_new = np.empty([3, 3])
        P_left = np.empty([3, 4])
        P_right = np.empty([3, 4])

        # Call OpenCV 'stereo rectify function
        cv2.stereoRectify(self.left_K,
                          self.left_D,
                          self.right_K,
                          self.right_D,
                          (self.left_height,
                           self.right_width),
                          self.extrinsics_R,
                          self.extrinsics_T,
                          R_left_new,
                          R_right_new,
                          P_left,
                          P_right,
                          None,
                          cv2.CALIB_ZERO_DISPARITY,
                          -1)
        return R_left_new, R_right_new, P_left, P_right


class StereoDataSet:
    """
      Class to load a EUROC dataset
    """

    def __init__(self, main_data_dir):
        "Init"

        if not (os.path.exists(main_data_dir)):
            raise Exception('specified directory does not exist')
        else:

            self.main_data_dir = main_data_dir

            left_camera_dir = main_data_dir + "cam0/"
            right_camera_dir = main_data_dir + "cam1/"
            left_camera_img_dir = left_camera_dir + "data/"
            right_camera_img_dir = right_camera_dir + "data/"

            # Read directories
            self.left_images = glob.glob(left_camera_img_dir + "*.png")
            self.right_images = glob.glob(right_camera_img_dir + "*.png")
            self.left_images.sort()
            self.right_images.sort()

            # Verify that no. of left images = no. of right images
            assert (len(self.left_images) == len(self.right_images))

            self.number_of_frames = len(self.left_images)

            # Prepare calibration file paths
            left_calib_file = left_camera_dir + "sensor.yaml"
            right_calib_file = right_camera_dir + "sensor.yaml"

            # Read Calibration Parameters
            stereo_calibration = Calibration(left_calib_file, right_calib_file)

            # Display Intrinsics for Left and Right cameras
            stereo_calibration.display_intrinsics("left")
            stereo_calibration.display_intrinsics("right")

            # Display Extrinsics for stereo pair
            stereo_calibration.display_extrinsics()

            # Apply stereo rectification and calculate updated matrices
            R_left_new, R_right_new, P_left, P_right = stereo_calibration.apply_rectification()

            # %% Calculate rectification maps (faster)
            # - Left image remap:
            leftMapX, leftMapY = cv2.initUndistortRectifyMap(stereo_calibration.left_K,
                                                             stereo_calibration.left_D,
                                                             R_left_new,
                                                             P_left,
                                                             (stereo_calibration.left_width,
                                                              stereo_calibration.left_height),
                                                             cv2.CV_32FC1)
            # - Right image remap:
            rightMapX, rightMapY = cv2.initUndistortRectifyMap(stereo_calibration.right_K,
                                                               stereo_calibration.right_D,
                                                               R_right_new,
                                                               P_right,
                                                               (stereo_calibration.left_width,
                                                                stereo_calibration.left_height),
                                                               cv2.CV_32FC1)

            # Save calibration information
            self.stereo_calibration = stereo_calibration

            self.leftMapX, self.leftMapY = leftMapX, leftMapY
            self.rightMapX, self.rightMapY = rightMapX, rightMapY

            # Extract rectified camera matrix - same for both left and right images
            self.rectified_camera_matrix = P_left[:, 0:3]

            # Compute length of stereo_baseline in meters
            self.stereo_baseline = np.linalg.norm(stereo_calibration.extrinsics_T)

    def load_stereo_pair(self, index):
        left_image = cv2.imread(self.left_images[index], 0)
        right_image = cv2.imread(self.right_images[index], 0)

        return left_image, right_image

    def process_stereo_pair(self, index):
        left_image = cv2.imread(self.left_images[index], 0)
        right_image = cv2.imread(self.right_images[index], 0)

        #  Apply rectification co-ordinate remap
        left_rectified = cv2.remap(left_image, self.leftMapX, self.leftMapY,
                                   cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
        right_rectified = cv2.remap(right_image, self.rightMapX, self.rightMapY,
                                    cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)

        return StereoPair(left_image, right_image, left_rectified, right_rectified)


class StereoPair:
    """
    Class to model a pair of stereo images loaded from the dataset
    """

    def __init__(self, left_image, right_image, left_rectified, right_rectified):
        # %% Initiate ORB detector
        orb = cv2.ORB_create()

        # find the keypoints and descriptors with ORB
        kp1, des1 = orb.detectAndCompute(left_rectified, None)
        kp2, des2 = orb.detectAndCompute(right_rectified, None)

        #  create BFMatcher object
        bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)

        # Match descriptors.
        matches = bf.match(des1, des2)

        # Filter stereo matches based on row coordinates and to enforce u_l >= ur
        valid_matches = [m for m in matches
                         if ((abs(kp1[m.queryIdx].pt[1] - kp2[m.trainIdx].pt[1]) < 2) and
                             (kp1[m.queryIdx].pt[0] >= kp2[m.trainIdx].pt[0]))]

        # Store as class members

        self.left_image, self.right_image = left_image, right_image
        self.left_rectified, self.right_rectified = left_rectified, right_rectified

        # Filter to only include valid matches
        self.kp1 = [kp1[m.queryIdx] for m in valid_matches]
        self.kp2 = [kp2[m.trainIdx] for m in valid_matches]

        self.des1 = des1[[m.queryIdx for m in valid_matches], :]
        self.des2 = des2[[m.trainIdx for m in valid_matches], :]

        # This last step is necessary so the matches refer to the filtered set
        for (i, m) in enumerate(valid_matches):
            m.queryIdx = m.trainIdx = i

        self.matches = valid_matches

    def display_matches(self):
        img = cv2.drawMatches(self.left_rectified, self.kp1,
                              self.right_rectified, self.kp2,
                              self.matches, None, flags=2)
        plt.imshow(img)

    def display_unrectified_images(self):
        plt.subplot(121)
        plt.imshow(self.left_image, cmap='gray')
        plt.subplot(122)
        plt.imshow(self.right_image, cmap='gray')


class TemporalMatch:
    """
    Class to model matches over time
    """

    def __init__(self, stereo_pair_1, stereo_pair_2):
        self.stereo_pair_1 = stereo_pair_1
        self.stereo_pair_2 = stereo_pair_2

        #  create BFMatcher object
        bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)

        # Match descriptors across time - left image
        self.matches = bf.match(stereo_pair_1.des1, stereo_pair_2.des1)

    def display_matches(self):
        img = cv2.drawMatches(self.stereo_pair_1.left_rectified, self.stereo_pair_1.kp1,
                              self.stereo_pair_2.left_rectified, self.stereo_pair_2.kp1,
                              self.matches, None, flags=2)
        plt.imshow(img)

    def get_normalized_matches(self, rectified_camera_matrix, stereo_baseline):
        f = rectified_camera_matrix[0, 0]
        cx = rectified_camera_matrix[0, 2]
        cy = rectified_camera_matrix[1, 2]

        n = len(self.matches)

        uvd1 = np.zeros((3, n))
        uvd2 = np.zeros((3, n))

        for i in range(0, n):
            m = self.matches[i]

            P1_l = self.stereo_pair_1.kp1[m.queryIdx].pt
            P1_r = self.stereo_pair_1.kp2[m.queryIdx].pt

            P2_l = self.stereo_pair_2.kp1[m.trainIdx].pt
            P2_r = self.stereo_pair_2.kp2[m.trainIdx].pt

            u1 = P1_l[0]
            v1 = P1_l[1]
            d1 = (P1_l[0] - P1_r[0])

            u2 = P2_l[0]
            v2 = P2_l[1]
            d2 = (P2_l[0] - P2_r[0])

            uvd1[0, i] = (u1 - cx) / f
            uvd1[1, i] = (v1 - cy) / f
            uvd1[2, i] = d1 / (f * stereo_baseline)

            uvd2[0, i] = (u2 - cx) / f
            uvd2[1, i] = (v2 - cy) / f
            uvd2[2, i] = d2 / (f * stereo_baseline)

        return uvd1, uvd2
