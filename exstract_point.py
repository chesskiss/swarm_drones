import cv2
import numpy as np

def exstract_points(binary_img,size,start_point,num_points=0,resolution=0.5):
    closed_curve=has_closed_curve(binary_img)
  
    if closed_curve:
          X_arr , Y_arr = split_close_curve(binary_img,num_points=num_points,resolution=resolution)
    else: 
          X_arr , Y_arr = split_open_curve(binary_img,num_points=num_points,resolution=resolution)
    idx=np.where(X_arr==min(X_arr))
    idx=idx[0][0]
    new_X=np.zeros_like(X_arr)
    new_Y=np.zeros_like(X_arr)
    for i in range(len(X_arr)):
        new_X[i]=X_arr[idx]
        new_Y[i]=Y_arr[idx]
        idx=(idx+1)%len(X_arr)
    new_X=new_X-new_X[0]
    new_Y=new_Y[0]-new_Y
    maxi = np.max(new_X) if np.max(new_X) > np.max(new_Y)-np.min(new_Y) else np.max(new_Y)-np.min(new_Y)
    points = []
    for i in range(len(new_X)):
        x = size*(new_X[i]/maxi)+start_point[0]
        y = size*(new_Y[i]/maxi)+start_point[1]  
        points.append([x,y])
    return points


def has_closed_curve(binary_img):
    
    binary_img=binary_img.astype(np.uint8)
    contours, hierarchy = cv2.findContours(binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for hir in hierarchy[0]:
        if hir[2]!=-1:
            return True
    
    return False

def split_open_curve(binary_image, num_points=0,resolution=0.5):
    num_point=num_points
    curve = np.where(binary_image == 1)
    start_y = int(curve[0][0])
    start_x = int(curve[1][0])

    distance_map = spread_img(binary_image, start_x, start_y)
    end_y, end_x = np.where(distance_map == np.max(distance_map))
    end_y, end_x = end_y[0], end_x[0]

    max_distance = np.max(spread_img(binary_image, end_x, end_y))
    if ((num_point==0) or (num_point==1)):
        num_point = int((max_distance-2)*resolution)
    distances = np.linspace(2, max_distance, num_point)
    distances = np.round(distances).astype(int)

    x_coords = np.zeros(num_point)
    y_coords = np.zeros(num_point)
    for i, distance in enumerate(distances):
        y_coords[i], x_coords[i] = np.mean(np.where(spread_img(binary_image, end_x, end_y) == distance), axis=1)

    return  x_coords,y_coords

def split_close_curve(binary_image, num_points=0,resolution=0.5):
    
    curve = np.where(binary_image == 1)
    start_y = int(curve[0][0])
    start_x = int(curve[1][0])

    distance_map = spread_img(binary_image, start_x, start_y)
    max_distance = np.max(distance_map)
    delete=np.zeros_like(binary_image)
    delete[np.where(distance_map==(int((2+max_distance)/2)))]=1
    delete=delete.astype(np.uint8)  
    contours, hierarchy = cv2.findContours(delete, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = contours[0]
    cv2.drawContours(delete, [cnt], 0, (0,255,0), 3)

    binary_image[np.where(delete==1)]=0
    X,Y=split_open_curve(binary_image, num_points+1,resolution=resolution)
    return X[1:], Y[1:]

def spread_img(binary_img1, thisX, thisY):
    binary_img = np.copy(binary_img1)
    spread_img = np.zeros_like(binary_img)
    spread_img[thisY, thisX] = 2
    binary_img[thisY, thisX] = 0
    queue = [(thisY, thisX)]
    while queue:
        y, x = queue.pop(0)
        for dy, dx in ((-1, 0), (0, 1), (1, 0), (0, -1)):
            ny, nx = y + dy, x + dx
            if (0 <= ny < binary_img.shape[0] and 0 <= nx < binary_img.shape[1] and binary_img[ny][nx]):
                queue.append((ny, nx))
                binary_img[ny][nx] = 0
                spread_img[ny][nx] = spread_img[y][x] + 1
    return spread_img


def img_to_binary_img(img):
    
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    threshold = 0.5
    binary_img = gray_img > threshold
    binary_img = binary_img.astype(int)
    return (binary_img)


