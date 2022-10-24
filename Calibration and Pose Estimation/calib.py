
import cv2
from cv2 import aruco
import numpy as np
from pathlib import Path
from tqdm import tqdm

# root directory of repo for relative path specification.
root = Path(__file__).parent.absolute()

# Set path to the images
CalibImgPath = root.joinpath("aruco_data")

# For validating results, show aruco board to camera.
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)

#Provide length of the marker's side
markerLength =  3.75 # Here, measurement unit is centimetre.

# Provide separation between markers
markerSeparation = 0.5   # Here, measurement unit is centimetre.

# create arUco board
board = aruco.GridBoard_create(4, 5, markerLength, markerSeparation, aruco_dict)

arucoParams = aruco.DetectorParameters_create()

img_list = []
calib_fnms = CalibImgPath.glob('*.jpg')
print('Using ...', end='')
for idx, fn in enumerate(calib_fnms):
    print(idx, '', end='')
    img = cv2.imread(str(root.joinpath(fn)))
    img_list.append(img)
    h, w, c = img.shape
print('Calibration images')

counter, corners_list, id_list = [], [], []
first = True
for im in tqdm(img_list):
    img_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=arucoParams)
    if first == True:
        corners_list = corners
        id_list = ids
        first = False
    else:
        corners_list = np.vstack((corners_list, corners))
        id_list = np.vstack((id_list, ids))
    counter.append(len(ids))
print('Found {} unique markers'.format(np.unique(ids)))

counter = np.array(counter)
print("Calibrating camera .... Please wait...")
# mat = np.zeros((3,3), float)
ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(corners_list, id_list, counter, board, img_gray.shape, None,
                                                          None)

print("Camera matrix is \n", mtx, "\n Distortion coefficients : \n",
      dist)
np.save("calibration_matrix", mtx)
np.save("distortion_coefficients", dist)