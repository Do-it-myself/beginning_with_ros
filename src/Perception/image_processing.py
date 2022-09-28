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

def main():
    video_capture = cv2.VideoCapture(0)

    while(True):
        ret, frame = video_capture.read()
        # threshold_image = basic_thresholding(frame, 120)
        threshold_image = adaptive_thresholding(frame, 5)
        cv2.imshow("Image Window", threshold_image)

if __name__ == "__main__":
    main()