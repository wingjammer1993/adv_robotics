#!/usr/bin/env python
from ball_detector import BallDetection
import rospy
import queue


class Detector(object):
	def __init__(self, det, queue_size=5, min_dist = 3):
		self.det = dets
		self.image_queue = queue.Queue(queue_size)
		# self.depth_queue = queue.Queue(queue_size)
        self.ball_values = {
            "centres" : queue.Queue(queue_size),
            "detections": queue.Queue(queue_size),
            "radii": queue.Queue(queue_size),
            "count": 0
        }

		self.bridge = CvBridge()

		# Ball Thresholds
		self.min_dist_th = min_dist
        self.slope = 0.0
        self.intercept = 0.0

        self.bbox_x_min = 240
        self.bbox_x_max = 400

        # Publishes object recognition prediction
        rospy.init_node("ball_detector", anonymous=True)

        self.ball_pub = rospy.Publisher(
            'ball_detect/out',
             Bool,
             queue_size=10
         )

        # Subscribes to rs_camera color
        rospy.Subscriber(
            "rgb_frame",
            Image,
            self.receive_image,
            queue_size=1,
        )

        # # Subscribes to rs_camera stereo
        # rospy.Subscriber(
        #     "depth_image",
        #     Image,
        #     self.recieve_depth,
        #     queue_size=1,
        # )

    def process_image(self, image):
        detection, center, radius = self.det.detect_ball(image)

        if self.ball_values["count"] >= self.queue_size:
            self.ball_values["centres"].get()
            self.ball_values["radii"].get()
            self.ball_values["detections"].get()
        else:
            self.ball_values["count"] += 1

        self.ball_values["centres"].put(center)
        self.ball_values["detections"].put(detection)
        self.ball_values["radii"].put(radius)

    def receive_image(self, msg):
        curr_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_image(curr_image)

        if self.image_queue.qsize() >= self.image_queue_size:
            self.image_queue.get() # pop the old image

        self.image_queue.put(curr_image)

    def line_fit(self):
        centres = np.array(self.ball_values["centres"].queue)

        self.slope, self.intercept = np.polyfit(centres[:, 0], centres[:, 1], deg=1)

    def evaluate_collision(self):
        if self.image_queue.size() == self.image_queue_size:
            detections = np.array(self.ball_values["detections"].queue)
            if np.all(detections):
                self.line_fit()
                y_min = self.intercept + self.slope * self.bbox_x_min
                y_max = self.intercept + self.slope * self.bbox_x_max

                # Intersection with the bounding box
                if y_max >= 0 || y_min >= 0:
                    rospy.loginfo("Ball going to collide")

                    return True
                else:
                    rospy.loginfo("Ball not colliding")

                    return False

        return False

    def run(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.balL_pub.publish(self.evaluate_collision())

            r.sleep()


if __name__ == "__main__":
	blue_low_th, blue_high_th = (29, 86, 6), (200, 255, 255)
	b_det = BallDetection(blue_low_th, blue_high_th)

	main_det = Detector(b_det)
	main_det.run()

