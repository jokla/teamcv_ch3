import os
import rosbag
import cv2
from cv_bridge import CvBridge, CvBridgeError

data_dir = "../data/"
rosbag_file = os.path.join(data_dir, "udacity-dataset_sensor_camera_center_2016-10-20-13-46-47_0.bag")


def get_image_dir(image_type):
    images_dir = os.path.join(data_dir, image_type)
    if not os.path.exists(images_dir):
        os.makedirs(images_dir)
    return images_dir


# left_images_dir = get_image_dir("left")
center_images_dir = get_image_dir("center")
# right_images_dir = get_image_dir("right")

# left_camera_topic = "/left_camera/image_color"
center_camera_topic = "/center_camera/image_color/compressed"
# right_camera_topic = "/right_camera/image_color"
topics = [center_camera_topic]


# left_image_timestamps = []
center_image_timestamps = []
# right_image_timestamps = []

bridge = CvBridge()


def try_write_image(dir, msg):
    image_name = os.path.join(dir, str(msg.header.stamp.to_nsec()) + ".jpg")
    try:
        cv2.imwrite(image_name, bridge.compressed_imgmsg_to_cv2(msg))
    except CvBridgeError as e:
        print(e)

with rosbag.Bag(rosbag_file, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        # if topic == left_camera_topic:
        #     left_image_timestamps.append(msg.header.stamp.to_nsec())
        #     try_write_image(left_images_dir, msg)
        if topic == center_camera_topic:
            center_image_timestamps.append(msg.header.stamp.to_nsec())
            try_write_image(center_images_dir, msg)
        # elif topic == right_camera_topic:
        #     right_image_timestamps.append(msg.header.stamp.to_nsec())
        #     try_write_image(right_images_dir, msg)

# make sure timestamps are in ascending chronological order
def assert_sorted(l):
    assert all(l[i] <= l[i+1] for i in xrange(len(l)-1))


# assert_sorted(left_image_timestamps)
assert_sorted(center_image_timestamps)
# assert_sorted(right_image_timestamps)
