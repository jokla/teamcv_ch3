import os
import rosbag
import cv2
import csv
from cv_bridge import CvBridge, CvBridgeError
from bisect import bisect

data_dir = "data"
rosbag_file = os.path.join(data_dir, "dataset.bag")


def get_image_dir(image_type):
    images_dir = os.path.join(data_dir, image_type)
    if not os.path.exists(images_dir):
        os.makedirs(images_dir)
    return images_dir

left_images_dir = get_image_dir("left")
center_images_dir = get_image_dir("center")
right_images_dir = get_image_dir("right")

steering_angle_file = os.path.join(data_dir, "gps.csv")

steering_report_topic = "/vehicle/steering_report"
gps_topic = '/vehicle/gps/fix'
left_camera_topic = "/left_camera/image_color"
center_camera_topic = "/center_camera/image_color"
right_camera_topic = "/right_camera/image_color"
topics = [steering_report_topic, gps_topic, left_camera_topic,
          center_camera_topic, right_camera_topic]

angle_timestamps = []
angle_values = []

gps_timestamps = []
gps_latitute_values = []
gps_longitude_values = []

left_image_timestamps = []
center_image_timestamps = []
right_image_timestamps = []

bridge = CvBridge()


def try_write_image(dir, msg):
    image_name = os.path.join(dir, str(msg.header.stamp.to_nsec()) + ".jpeg")
    try:
        cv2.imwrite(image_name, bridge.imgmsg_to_cv2(msg))
    except CvBridgeError as e:
        print(e)

with rosbag.Bag(rosbag_file, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == steering_report_topic:
            angle_timestamps.append(msg.header.stamp.to_nsec())
            angle_values.append(msg.steering_wheel_angle)
        if topic == gps_topic:
            gps_timestamps.append(msg.header.stamp.to_nsec())
            gps_latitute_values.append(msg.latitude)
            gps_longitude_values.append(msg.longitude)
        elif topic == left_camera_topic:
            left_image_timestamps.append(msg.header.stamp.to_nsec())
            try_write_image(left_images_dir, msg)
        elif topic == center_camera_topic:
            center_image_timestamps.append(msg.header.stamp.to_nsec())
            try_write_image(center_images_dir, msg)
        elif topic == right_camera_topic:
            right_image_timestamps.append(msg.header.stamp.to_nsec())
            try_write_image(right_images_dir, msg)


# make sure timestamps are in ascending chronological order
def assert_sorted(l):
    assert all(l[i] <= l[i+1] for i in xrange(len(l)-1))

assert_sorted(angle_timestamps)
assert_sorted(gps_timestamps)
assert_sorted(left_image_timestamps)
assert_sorted(center_image_timestamps)
assert_sorted(right_image_timestamps)


with open(steering_angle_file, "w") as angles_file:
    fieldnames = ["timestamp", "latitude","longitude"]
    writer = csv.DictWriter(angles_file, fieldnames=fieldnames)
    writer.writeheader()
    for i in range(0,len(gps_timestamps)):
        writer.writerow({"timestamp": gps_timestamps[i],
                         "latitude": gps_latitute_values[i],
                         "longitude": gps_longitude_values[i]})
