# importing the module
import cv2
import numpy as np
import math as m

# function to save the coordinates of the points clicked on the image
def click_event(event, x, y, flags, param):

	# retrieve the image via param
	img = param

	# define global variables to be returned by the LoadMap function
	global Xsave
	global Ysave

	# initialise an array to store the RGB values
	rgb = np.empty(3, dtype = int)

	# checking for left mouse clicks
	if event == cv2.EVENT_LBUTTONDOWN or event==cv2.EVENT_RBUTTONDOWN:

		# saving the RGB values
		rgb[0] = img[y, x, (2)]
		rgb[1] = img[y, x, (1)]
		rgb[2] = img[y, x, (0)]

		# If the selected point is a wall, an error message is printed on the shell
		if rgb[0]<=25 and rgb[1]<=25 and rgb[2]<=25:

			print('The selected point is a wall')

		else:

			Xsave = x
			Ysave = y

			# displaying a point on the image window
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(img, '.', (x-6,y), font, 1, (255, 0, 0), 5)
			cv2.imshow('ImageViewer', img)

# driver function
def LoadMap(ImagePath):

	# reading the image
	img = cv2.imread(ImagePath, 1)

	# resizing the image if necessary
	resized = False
	if img.shape[0]<250 and img.shape[1]<250:
		scale_percent = 350 # multiplier of the original size
		width = int(img.shape[1] * scale_percent / 100)
		height = int(img.shape[0] * scale_percent / 100)
		img = cv2.resize(img, (width, height), interpolation = cv2.INTER_CUBIC)
		resized = True # boolean variable used to return the right coordinates

	# displaying the image
	cv2.imshow('ImageViewer', img)

	# setting mouse handler for the image and calling the click_event() function
	cv2.setMouseCallback('ImageViewer', click_event, img)

	# wait for a key to be pressed to exit
	cv2.waitKey(0)

	# close the window
	cv2.destroyAllWindows()

	if resized == True:
		return m.floor(Xsave/scale_percent*100), m.floor(Ysave/scale_percent*100) #return the coordinates before resizing the image
	elif resized == False:
		return Xsave, Ysave