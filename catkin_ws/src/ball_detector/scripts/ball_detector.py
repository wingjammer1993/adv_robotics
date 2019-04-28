import cv2 as cv
import imutils

class BallDetection(object):
    def __init__(self, lower_th, higher_th):
        self.lower_th = lower_th
        self.higher_th = higher_th

    def convert_to_hsv(self, image):
        blur_img = cv.GaussianBlur(image, (11, 11), 0)
        return cv.cvtColor(blur_img, cv.COLOR_BGR2HSV)

    def generate_mask(self, image):
        hsv_img = self.convert_to_hsv(image)

        mask = cv.inRange(hsv_img, self.lower_th, self.higher_th)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)

        return mask

    def detect_ball(self, image):
        image_mask = self.generate_mask(image)
        centers = cv.findContours(image_mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        centers = self.grab_contours(centers)

        radius = 0.0

        if len(centers) > 0:
            c = max(centers, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)
            M = cv.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            return True, center, radius
            
#         if radius > 10:
#             # draw the circle and centroid on the frame,
#             # then update the list of tracked points
#             cv.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
#             cv.circle(image, center, 5, (0, 0, 255), -1)
#             plt.imshow(image)
#             plt.savefig("/Users/vachanda/Desktop/outputs/{}.jpg".format(time.time()))

        return False, (), radius