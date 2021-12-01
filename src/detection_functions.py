import numpy as np
import cv2


class apple_detection():

    def filter_color(self, rgb_img):
        #convert the image into the HSV color space --> more suitable for this application
        hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        # cv2.imshow("HSV Image",hsv_img)   
        # cv2.waitKey(3)
        #find the upper and lower bounds of the yellow color (apple color)

        # dark
        # yellowLower =(15, 130, 10)
        # yellowUpper = (70, 255, 175)

        # ligth
        # yellowLower =(15, 130, 10)
        # yellowUpper = (60, 255, 255)

        # lab
        yellowLower =(15, 140, 10)
        yellowUpper = (60, 255, 255)

        #define a mask using the lower and upper bounds of the yellow color 
        mask = cv2.inRange(hsv_img, yellowLower, yellowUpper) # it shows yellow pixels as white and other as black
        cv2.imshow("Mask Image",mask)   
        cv2.waitKey(3)
        return mask

    def getContours(self, binary_image):      
        #_, contours, hierarchy = cv2.findContours(binary_image, 
        #                                          cv2.RETR_TREE, 
        #                                           cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(binary_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 
        return contours

    def draw_ball_contour(self, binary_image, rgb_image, contours):
        #global cx, cy
        c_points = []
        if len(contours) != 0:
            c_max1 = max(contours, key = cv2.contourArea) # find contour with max area obtained
        for c in contours:
            area = cv2.contourArea(c)
            if (area >= 0.3*cv2.contourArea(c_max1)):
                cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                cx, cy = self.get_contour_center(c)
                x,y,w,h = cv2.boundingRect(c)
                # draw the biggest contour (c_max) in red
                cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(0,0,255),1)
                cv2.circle(rgb_image, (cx,cy),1,(255,0,0),2) # show the center in blue
                c_points.append([cx, cy])
            
                #print(type(contours)) #-->list

        cv2.imshow("RGB Image Contours",rgb_image)
        cv2.waitKey(3)
        #print(c_points)
        return c_points
        

    def get_contour_center(self, contour):
        M = cv2.moments(contour)
        cx=-1
        cy=-1
        if (M['m00']!=0):
            cx= int(M['m10']/M['m00'])
            cy= int(M['m01']/M['m00'])
        return cx, cy

    def get_center_depth(self, depth_img, bx, by):
        depth_of_center = depth_img[bx, by]
        return depth_of_center





    # ideally useful to fuse center pixel neighbour in case the center is "nan"
    def get_avg_center_depth(self, depth_img, bx, by, offset):
        bx_low = bx-offset
        bx_high = bx+offset
        by_low = by-offset
        by_high = by+offset+1 # it excludes last index
        depth_of_center_matrix = depth_img[bx_low:bx_high, by_low:by_high]
        depth_avg = np.sum(depth_of_center_matrix)/float(depth_of_center_matrix.size)
        return depth_avg
