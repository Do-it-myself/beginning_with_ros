import cv2

def basic_thresholding(rgb_image, threshold_val):
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    ret, threshold_image = cv2.threshold(gray_image, threshold_val, 255, cv2.THRESH_BINARY)
    return threshold_image

def adaptive_thresholding(rgb_image, threshold_val):
    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    threshold_image = cv2.adaptiveThreshold(gray_image, 
                                                 255, 
                                                 cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                                 cv2.THRESH_BINARY,
                                                 threshold_val,
                                                 2)
    return threshold_image

def color_filtering(rgb_image):
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    lower = (0, 10, 60)
    upper = (20, 150, 210)
    mask = cv2.inRange(hsv, lower, upper)
    return mask

def draw_contours(rgb_image, adaptive):
    global contours, hierarchy

    if adaptive:
        binary_image = adaptive_thresholding(rgb_image, 2001)
    else: 
        binary_image = basic_thresholding(rgb_image, 127)
    cv2.imshow("Binary", binary_image)
    
    # find contours (return a tuple of arrays)
    contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print(contours)

    # draw contours
    index = -1 # all contours
    color = (0, 0, 255) # contour line color
    thickness = 2 # contour line thickness
    cv2.drawContours(rgb_image, contours, index, color, thickness)

def process_contours(rgb_image, adaptive):
    global contours, hierarchy

    draw_contours(rgb_image, adaptive)

    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True) # whether contour is closed or not
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        print((x, y))
        
        # get centre
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        else:
            cx, cy = -1, -1

        # display features
        print("Area: {}, Perimeter: {}".format(area, perimeter))
        cv2.circle(rgb_image, (int(x), int(y)), int(radius), (255, 0, 0), 2)
        cv2.circle(rgb_image, (cx, cy), 0, (0, 255, 0), 10)

def main(video):
    if video:
        video_capture = cv2.VideoCapture(0)

        while(True):
            ret, frame = video_capture.read()
            flipped = cv2.flip(frame, 1)

            #image = adaptive_thresholding(flipped, 5)
            #image = color_filtering(flipped)
            image = flipped

            cv2.imshow("Image", image)
            cv2.waitKey(1)
    else:
        frame = cv2.imread("C:/Users/chloe_rns2pbz/Desktop/beginning_with_ros/src/Perception/images/shapes.png")
        flipped = cv2.flip(frame, 1)

        #draw_contours(flipped, True)
        process_contours(flipped, True)

        image = flipped
        
        cv2.imshow("Image", image)
        cv2.waitKey(0)

if __name__ == "__main__":
    main(False)