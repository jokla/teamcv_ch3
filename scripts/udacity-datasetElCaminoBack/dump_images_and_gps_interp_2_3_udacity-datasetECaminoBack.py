import os
import rosbag
import csv
from bisect import bisect_left
import cv2
from cv_bridge import CvBridge, CvBridgeError


data_dir = "/media/jokla/Elements/udacity/challeng2/UdacitySDC_ElCamino/udacity-datasetElCaminoBack/"
rosbag_file_gps = os.path.join(data_dir, "udacity-dataset_io_vehicle_2016-10-20-15-13-30_0.bag")
rosbag_file_imgs = os.path.join(data_dir, "udacity-dataset_sensor_camera_center_2016-10-20-15-13-30_0.bag")

# Left Image structure
# left_image_topic = '/left_camera/image_color'
# left_image_struct = []

bridge = CvBridge()

# Center Image structure
center_image_topic = '/center_camera/image_color/compressed'
center_image_struct = []

# Right Image structure
# right_image_topic = '/right_camera/image_color'
# right_image_struct = []

# GPS structure
gps_value_topic = '/vehicle/gps/fix'
gps_vel_topic = '/vehicle/gps/vel'
gps_topics = [gps_value_topic,gps_vel_topic]
gps_struct = []
gps_vel_struct = []

#topics = [gps_topic, left_image_topic, center_image_topic, right_image_topic]

with rosbag.Bag(rosbag_file_gps, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=gps_topics):
        if topic == gps_value_topic:
            gps_struct.append( [msg.header.stamp.to_nsec(),
                                msg.latitude,
                                msg.longitude,
                                msg.altitude] )
        elif topic == gps_vel_topic:
            gps_vel_struct.append( [msg.header.stamp.to_nsec(), 
            						msg.twist.linear.x] )


        # elif topic == left_image_topic:
        #     left_image_struct.append( [msg.header.stamp.to_nsec()] )
        # elif topic == center_image_topic:
        #     center_image_struct.append( [msg.header.stamp.to_nsec()] )
        # elif topic == right_image_topic:
        #     right_image_struct.append( [msg.header.stamp.to_nsec()] )

with rosbag.Bag(rosbag_file_imgs, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=center_image_topic):
        if topic == center_image_topic:
            center_image_struct.append( [msg.header.stamp.to_nsec()] )




# make sure timestamps are in ascending chronological order
def assert_sorted(l):
    assert all(l[i][0] <= l[i+1][0] for i in xrange(len(l)-1))

assert_sorted(gps_struct)
assert_sorted(gps_vel_struct)
# assert_sorted(left_image_struct)
assert_sorted(center_image_struct)
# assert_sorted(right_image_struct)


# https://stackoverflow.com/questions/12141150/from-list-of-integers-get-number-closest-to-a-given-value
def takeClosest(myList, myNumber):
    """
    Assumes myList is sorted. Returns closest value to myNumber.

    If two numbers are equally close, return the smallest number.
    """
    pos = bisect_left(myList, myNumber)
    if pos == 0:
        return 0 #myList[0]
    if pos == len(myList):
        return len(myList)-1 #myList[-1]

    before = myList[pos - 1]
    after = myList[pos]

    if after - myNumber < myNumber - before:
       return pos #after
    else:
       return pos-1 #before


def naive_gps_interpolation(image_struct, gps, gps_vel):
    # Find closest index
    gps_timestamp, _, _, _ = zip(*gps)
    cpt = takeClosest(gps_timestamp, image_struct[0][0])

    # Find first index before initial camera timestamp
    while image_struct[0][0] - gps[cpt][0] < 0 and cpt > 0:
        cpt = cpt-1

    gps_interp = []
    for i in xrange(len(image_struct)):
        current_timestamp = image_struct[i][0]

        while cpt+1 < len(gps) and image_struct[i][0] - gps[cpt+1][0] >= 0 and cpt+1 < len(gps):
            cpt = cpt+1

        # Assume two consecutive GPS timestamps are widder than image timestamp diff, otherwise use two counters?
        if cpt+1 < len(gps):
            diff_timestamp = gps[cpt+1][0] - gps[cpt][0]

            delta_latitude = (gps[cpt+1][1] - gps[cpt][1]) / diff_timestamp
            delta_longitude = (gps[cpt+1][2] - gps[cpt][2]) / diff_timestamp
            delta_vel = (gps_vel[cpt+1] - gps_vel[cpt]) / diff_timestamp

            timestamp_prev = gps[cpt][0]
            latitude_prev = gps[cpt][1]
            longitude_prev = gps[cpt][2]
            vel_prev = gps_vel[cpt]

            diff_timestamp_image = current_timestamp-timestamp_prev
            gps_interp.append( [current_timestamp,
                                latitude_prev + diff_timestamp_image * delta_latitude,
                                longitude_prev + diff_timestamp_image * delta_longitude,
                                vel_prev + diff_timestamp_image * delta_vel] )
        elif cpt-1 >= 0 and image_struct[i][0] - gps[cpt][0] < 0 and image_struct[i][0] - gps[cpt-1][0] >= 0:
            diff_timestamp = gps[cpt][0] - gps[cpt-1][0]

            delta_latitude = (gps[cpt][1] - gps[cpt-1][1]) / diff_timestamp
            delta_longitude = (gps[cpt][2] - gps[cpt-1][2]) / diff_timestamp
            delta_vel = (gps_vel[cpt] - gps_vel[cpt-1]) / diff_timestamp

            timestamp_prev = gps[cpt-1][0]
            latitude_prev = gps[cpt-1][1]
            longitude_prev = gps[cpt-1][2]
            vel_prev = gps_vel[cpt-1]

            diff_timestamp_image = current_timestamp-timestamp_prev
            gps_interp.append( [current_timestamp,
                                latitude_prev + diff_timestamp_image * delta_latitude,
                                longitude_prev + diff_timestamp_image * delta_longitude,
                                vel_prev + diff_timestamp_image * delta_vel] )
        else:
            latitude_end = gps[cpt][1]
            longitude_end = gps[cpt][2]
            vel_end = gps_vel[cpt]
            gps_interp.append( [current_timestamp,
                                latitude_end,
                                longitude_end,
                                vel_end] )

    return gps_interp


def save_gps_csv(gps, name):
    gps_data_file = os.path.join(data_dir, name)

    with open(gps_data_file, "w") as save_file:
        fieldnames = ["timestamp", "latitude", "longitude", "vel"]

        writer = csv.DictWriter(save_file, fieldnames=fieldnames)
        writer.writeheader()

        for i in xrange(0, len(gps)):
            writer.writerow({"timestamp": gps[i][0],
                             "latitude": gps[i][1],
                             "longitude": gps[i][2],
                             "vel": gps[i][3]})


def try_write_image(dir, msg):
    image_name = os.path.join(dir, str(msg.header.stamp.to_nsec()) + ".jpg")
    try:
        cv2.imwrite(image_name, bridge.compressed_imgmsg_to_cv2(msg))
    except CvBridgeError as e:
        print(e)


def get_image_dir(image_type):
    images_dir = os.path.join(data_dir, image_type)
    if not os.path.exists(images_dir):
        os.makedirs(images_dir)
    return images_dir




# Naive linear interpolation for left camera
# left_gps_interp = naive_gps_interpolation(left_image_struct, gps_struct)
# save_gps_csv(left_gps_interp, "gps_data_left_interp.csv")
print (len(gps_vel_struct))
print (len([row[1] for row in gps_vel_struct]))

center_gps_interp = naive_gps_interpolation(center_image_struct, gps_struct, [row[1] for row in gps_vel_struct])
#save_gps_csv(center_gps_interp, "gps_vel_data_center_interp.csv")

center_images_dir = get_image_dir("center")
center_vel_zero_images_dir = get_image_dir("center_vel_zero")

center_gps_interp_filtered = []
count = 0

with rosbag.Bag(rosbag_file_imgs, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=center_image_topic):
        if msg.header.stamp.to_nsec() > 1477007215156377715:
            if (abs(center_gps_interp[count][3]) > 0.0001):
                center_gps_interp_filtered.append(center_gps_interp[count])
                try_write_image(center_images_dir, msg)
            else:
                print ("Skipped frame " + str(center_gps_interp[count][0]))
            	#try_write_image(center_vel_zero_images_dir, msg)
        count = count + 1
save_gps_csv(center_gps_interp_filtered, "gps_vel_data_center_interp_filtered.csv")
