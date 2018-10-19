## IPV-Project

#PART I
Bbox_2D.m file:

 1) Loads the images from the dataset.
 2) Calculates Background Image.
 3) Subtract each image to the Bg Image.
 4) Run a filter through the images to delete noise.
 5) Label all the connected components (bwlabel function in matlab).
 6) Determine the 2D bbox of each connected component (regionprops function in matlab).
 7) Select the biggest bbox, by Area, and draw the bbox in the image (rectangle function in matlab).

What to do next:

 1) Compute the xyz coordinates of each pixel, with the depth image. 
 2) map the labeled components (only the persons) to the depth image, and find the maximum and minimum Z coordinates of those components.
 3) compute the xy coordinates for the 4 points of the 2D Bbox, with the minimum Z, and do the same for the maximum Z.
 4) Output the xyz coordinates of the 8 points (Test in the Point Cloud if the points are being calculated as intended). 


#PART II
What to do:
 - Same as in Part I, but with 2 cameras. 
 
 1) Transform and rotate the second camera frame so that it matches with the world frame (the first camera frame), superimpose both camera frames.
 2) With the aditional information from the second camera, find the 8 points for the 3D Bbox of each moving object. 
 3) The 8 points must be outputed according to the world frame (camera 1 frame).
