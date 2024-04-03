import numpy as np
import cv2

def ordered_rictangle_points(points):
    # output: [top_left, top_right, bottom_right, bottom_left]
    # sort the points by x
    points = points[np.argsort(points[:, 0]), :]
    # sort the points by y
    top_points = points[:2, :]
    bottom_points = points[2:, :]
    top_points = top_points[np.argsort(top_points[:, 1]), :]
    bottom_points = bottom_points[np.argsort(bottom_points[:, 1]), :]
    # top_left, bottom_left,bottom_right, top_right
    return np.array([top_points[0], top_points[1], bottom_points[1], bottom_points[0]], dtype=np.float32)

def find_rectangles(binary_image,ratio):
    # Find contours in the binary image
    contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    # Filter out contours with internal contours (holes)
    contours_with_holes = []
    for i, contour in enumerate(contours):
        # Check if the contour has a child contour
        if hierarchy[0][i][2] != -1:
            contours_with_holes.append(contour)
    if len(contours_with_holes)==0:
        return None
    
    for contour in contours_with_holes:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx_polygon = cv2.approxPolyDP(contour, epsilon, True)

        # If the contour is a rectangle (4 sides), extract its corner coordinates
        min_score=1000
        if len(approx_polygon) == 4:
            max_x = np.max(approx_polygon[:, 0, 0])
            min_x = np.min(approx_polygon[:, 0, 0])
            max_y = np.max(approx_polygon[:, 0, 1])
            min_y = np.min(approx_polygon[:, 0, 1])
            # find the ratio btween the high and width
            high = max_y-min_y+0.0000001
            width = max_x-min_x+0.0000001

            ratio_poligon = abs(width/high)
            score=abs(ratio-ratio_poligon)
            if score<min_score:
                min_score=score
                min_rectangle=approx_polygon.reshape(-1, 2)
    if min_score>4:
        return None
    return min_rectangle
    
def find_green_rectangle_points(image,ratio):

    # do bileteral filter
    image = cv2.bilateralFilter(image, 20, 15, 15)
    image = cv2.bilateralFilter(image, 20, 15, 15)
    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the green color (in HSV)
    lower_green = np.array([60, 40, 40])
    upper_green = np.array([100, 255, 255])

    # Create a binary mask for the green color
    green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
    # do erode and dilate
    kernel = np.ones((5,5),np.uint8)
    green_mask = cv2.erode(green_mask,kernel,iterations = 1)
    green_mask = cv2.dilate(green_mask,kernel,iterations = 2)
    green_mask = cv2.erode(green_mask,kernel,iterations = 1)
    
    # create a white mask from the img
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([255, 30, 255])

    # Create a binary mask for the green color
    white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
    # do dilate
    white_mask = cv2.dilate(white_mask,kernel,iterations = 2)
    # combine the green mask and white mask
    new_green_mask = cv2.bitwise_and(green_mask, white_mask)

    return(find_rectangles(new_green_mask,ratio))


def calculate_camera_position(intrinsic_matrix, image_points, real_height,real_width):
    top_left, top_right, bottom_right, bottom_left = image_points
    width1 = np.linalg.norm(top_right - top_left)
    width2 = np.linalg.norm(bottom_right - bottom_left)
    img_width = (width1 + width2) / 2
    height1 = np.linalg.norm(top_right - bottom_right)
    height2 = np.linalg.norm(top_left - bottom_left)
    img_height = (height1 + height2) / 2

    
  
    ratio_img_world = ((real_width / img_width)+ (real_height / img_height))/2

    fx= intrinsic_matrix[0][0]
    fy= intrinsic_matrix[1][1]
    py=intrinsic_matrix[0][2]
    pz=intrinsic_matrix[1][2]

    dis_x1=fx*real_width/img_width
    dis_x2=fy*real_height/img_height
    dis_x=(dis_x1+dis_x2)/2
    
    dis_y=(py-top_left[0])*ratio_img_world
    dis_z=(pz-top_left[1])*ratio_img_world    

    print(f"dis_x={dis_x}, dis_y={dis_y}, dis_z={dis_z}")

    return np.array([dis_x, dis_y, dis_z], dtype=np.float32)

#return drone's location
def find_location(real_width,real_high,img):

    intrinsic_matrix = np.array([[9.105807265540521e+02, 0, 4.574269390215023e+02],
                                 [0, 9.139776296246570e+02, 3.343168681518988e+02],
                                 [0, 0, 1]], dtype=np.float32)
    
    ratio=real_width/real_high
    
    squre_point=find_green_rectangle_points(img,ratio)
    if squre_point is not None:
        top_left, bottom_left,bottom_right, top_right  = ordered_rictangle_points(squre_point)    
        img_points = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)

        camera_position = calculate_camera_position(intrinsic_matrix, img_points, real_high ,real_width)
        
    else:
        camera_position=None
    return camera_position   
