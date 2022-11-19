import cv2
import numpy as np
import math

class LF:

    def region_of_interest(self,edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)
        
        polygon = np.array([[
            (0, height),
            (0,  height * 3 / 5),
            (width / 2, height / 3),
            (width , height * 3 / 5),
            (width , height),]], np.int32)
        
        
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        return [cropped_edges, polygon] 
    def detect_edges(self,frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        w,h,_ = hsv.shape
        hsvYellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)


        mask = cv2.inRange(hsv,lower_blue,upper_blue)
       
        edges = cv2.Canny(mask, 50, 100)
        
        return edges
    
    def detect_line_segments(self,cropped_edges):
        rho = 1  
        theta = np.pi / 180  
        min_threshold = 30  
        
        line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, np.array([]), minLineLength=5, maxLineGap=120)

        return line_segments
    def average_slope_intercept(self,frame, line_segments):
        lane_lines = []
        
        if line_segments is None:
           
            return lane_lines

        height, width,_ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1 / 3
        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary
        
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue
                
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - (slope * x1)
                
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))
        

        return lane_lines
    
    def make_single_line_on_each_side(self, lane_lines, frame_width):
        left_count = 0
        right_count = 0
        
        if lane_lines is not None:
            for line in lane_lines:
                for x1, y1, x2, y2 in line:
                    avg_x = (x1 + x2) / 2
                    if(avg_x < (frame_width / 2)):
                        left_count += 1
                    else:
                        right_count += 1

        if(left_count > 1 or right_count > 1):
            while(len(lane_lines) > 1 ):
                lane_lines.pop(1); 
        
        return lane_lines
    
    def make_points(self,frame, line):
        height, width, _ = frame.shape
        
        slope, intercept = line
        
        y1 = height  
        y2 = int(y1 / 2) 
        
        if slope == 0:
            slope = 0.1
            
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        
        return [[x1, y1, x2, y2]]
    def display_lines(self,frame, lines, line_color=(0, 255, 0), line_width=4):
        line_image = np.zeros_like(frame)
        
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
                    
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        
        return line_image
    def display_heading_line(self,frame, steering_angle, line_color=(0, 0, 255), line_width=5):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape
        
        steering_angle_radian = steering_angle / 180.0 * math.pi
        
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)
        
        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
        
        return heading_image
    def get_steering_angle(self,frame, lane_lines):
        
        height,width,_ = frame.shape
        
        
        if len(lane_lines) == 2:
            left_x1, _, left_x2, _ = lane_lines[0][0]
            right_x1, _, right_x2, _ = lane_lines[1][0]
            mid = int(width / 2)
            x_offset = (left_x2 + right_x2) / 2 - mid
            if(x_offset < 2 and x_offset > -2):
                temp = 0#print(left_x2 -  left_x1)
            y_offset = int(height / 2)
            
        elif len(lane_lines) == 1:
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
            y_offset = int(height / 2)
        
        elif len(lane_lines) == 0:
            x_offset = 0
            y_offset = int(height / 2)
        
        
        return steering_angle

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)

