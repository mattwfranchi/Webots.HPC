import cv2
import numpy as np
import math
import collections

if not cv2.useOptimized():
    cv2.setUseOptimized(True)

def crop_roi(image):
    h, w = image.shape
    cropped_img = image[int(h/2)+20:h,0:w]
    return cropped_img

def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")

def to_hls(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

def to_hsv(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    
            
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255 # <-- This line altered for grayscale.
    
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
    # If there are no lines to draw, exit.
    if lines is None:
        return
    # Make a copy of the original image.
    img = np.copy(img)
    # Create a blank image that matches the original in size.
    line_img = np.zeros((img.shape[0],img.shape[1],3),dtype=np.uint8)
    # Loop over all lines and draw them on the blank image.
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)    # Merge the image with the lines onto the original.
    
    img = cv2.addWeighted(img, 0.8,line_img , 1.0, 0.0)    # Return the modified image.
    return img


##########################
# Main Processing
###########################

#right_mem = collections.deque(maxlen=5)

q_length = 1
left_mem  = collections.deque(maxlen=q_length)
left_mem_slope = collections.deque(maxlen=q_length)

right_mem  = collections.deque(maxlen=q_length)
right_mem_slope = collections.deque(maxlen=q_length)


def init():
    global left_mem, left_mem_slope,q_length
    left_mem  = collections.deque(maxlen=q_length)
    left_mem_slope = collections.deque(maxlen=q_length)
    
    right_mem  = collections.deque(maxlen=q_length)
    right_mem_slope = collections.deque(maxlen=q_length)
    
###
# side = 0 left side, side 1 right side.
###
def calcualte_slope(filename,side):
    image = cv2.imread(filename,1)
    height, width,_ = image.shape
    vertices = [
        (0, height),
        (10 + width / 2, 10 + height / 2),
        (width, height)]

    cannyed_image = cv2.Canny(image, 100, 200)
    cropped_image = region_of_interest(cannyed_image,
                                       np.array([vertices],
                                       np.int32))
    
    #cv2.imshow('Canny Edge',cropped_image) 
    
    lines = cv2.HoughLinesP(cropped_image,
                            rho=6,
                            theta=np.pi / 180,
                            threshold=100,
                            lines=np.array([]),
                            minLineLength=40,
                            maxLineGap=300)
    

    
    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

    left_slope = []
    right_slope = []

    
    if(lines is not None):
        for line in lines:
            for x1,y1,x2,y2 in line:
                
                slope = (y2-y1)/(x2-x1)

                if(math.fabs(slope)<0.3):
                    continue
                if (slope<=0):
                    left_line_x.append(x1)
                    left_line_y.append(y1)
                    left_slope.append(slope)
                    ##print ('left side lane:', slope)
                else:
                    right_line_x.append(x1)
                    right_line_y.append(y1)
                    right_slope.append(slope)
                   
    new_lines = []
    
    got_left = False
    got_right  = False
    if(len(left_slope)>0 and side==0) :
        m = np.mean(left_slope)
        avg_x = np.mean(left_line_x)
        avg_y = np.mean(left_line_y)
        
        b = avg_y - m*avg_x
        
        x1 = int((height/2 - b)/m)
        x2 = int((height- b)/m)
        
        left_mem.append([(x1, int(height/2), x2, height)])
        
        new_lines.append(np.mean(left_mem, axis=0).astype(int))
        
        left_mem_slope.append(m)
        
        got_left = True
        #new_lines.append([(x1, int(height/2), x2, height)])
        #return np.array(left_slope).mean()

    if(len(right_slope)>0 and side==1) :
        m = np.mean(right_slope)
        avg_x = np.mean(right_line_x)
        avg_y = np.mean(right_line_y)
        
        b = avg_y - m*avg_x
        
        x1 = int((height/2 - b)/m)
        x2 = int((height- b)/m)
        
        right_mem.append([(x1, int(height/2), x2, height)])
        
        new_lines.append(np.mean(right_mem, axis=0).astype(int))
        
        right_mem_slope.append(m)
        
        got_right = True
        
    
    img_lines = draw_lines(image,new_lines)
    
    ##print (draw_lines)
    
 
    
    #if(img_lines is not None):
    #   cv2.imshow('Lane Keeping',img_lines) 
    
    #   cv2.waitKey(0)
       #pass
    
    right_s = 0.0
    #if(got_left and got_right):
    #    right_s = np.mean(right_mem_slope)
    #    left_s =  np.mean(left_mem_slope)
        
    #    #print ('left lane slope : ', left_s , ' \t right lane slope : ', right_s)
    #    return left_s
        
    if(got_right and side==1):
        right_s = np.mean(right_mem_slope)
        ##print ('                  \t right lane slope : ', right_s)
        #cv2.putText(img_lines,"slope:"+str(right_s), (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
        ##print (right_mem)
        #cv2.circle(img_lines,(x2, height), 5, (0,0,255), -1)
        #cv2.imshow('right side',img_lines) 
        #cv2.waitKey(0)
        return right_s
    
        
        ##print ('right slope : ', right_s)
    elif(got_left and side==0):
        left_s =  np.mean(left_mem_slope)
        ##print ('left lane slope : ', left_s)
        #cv2.putText(img_lines,"slope:"+str(left_s), (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
        
        ##print (left_mem)
        #cv2.circle(img_lines,(x2, height), 5, (0,0,255), -1)
        #cv2.imshow('left side',img_lines)
        #cv2.waitKey(0)
        return  left_s
    
    else :
        return -0.65
        
        
    return m 
    #if(len(right_slope)>0):
    #    return np.array(right_slope).mean()

    #left_avg = np.dot(left_lengths, left_lines)/np.sum(left_lengths) if len(left_lengths) > 0 else None
 
    

