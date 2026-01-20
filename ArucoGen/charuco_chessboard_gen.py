import cv2
from cv2 import aruco

# choose dictionary and board parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
squares_x = 5    # number of chessboard squares in X direction
squares_y = 7    # number of chessboard squares in Y direction
square_length = 0.035  # meters (35 mm)
marker_length = square_length * 0.7  # marker inside square

# create board
board = aruco.CharucoBoard_create(
    squaresX=squares_x,
    squaresY=squares_y,
    squareLength=square_length,
    markerLength=marker_length,
    dictionary=aruco_dict
)

# draw and save
img = board.draw((2000, 3000))  # adjust pixel size for print
cv2.imwrite("charuco_board.png", img)
