import os
import rosbag
import csv


data_dir = "../data/"
rosbag_file = os.path.join(data_dir, "test.bag")


# GPS structure
gps_topic = '/vehicle/gps/fix'
gps_struct = []

# GPS velocity structure
gps_vel_topic = '/vehicle/gps/vel'
gps_vel_struct = []

# IMU data raw structure

imu_data_raw_topic = '/vehicle/imu/data_raw'
imu_data_raw_struct = []

# Joint state structure
# Should be useless for us
#~ joint_states_topic = '/vehicle/joint_states'
#~ joint_states_struct = []


topics = [gps_topic, gps_vel_topic, imu_data_raw_topic]


with rosbag.Bag(rosbag_file, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == gps_topic:
            gps_struct.append( [msg.header.stamp.to_nsec(), 
                                msg.latitude, 
                                msg.longitude, 
                                msg.altitude] )
        elif topic == gps_vel_topic:
            gps_vel_struct.append( [msg.header.stamp.to_nsec(), 
                                    msg.twist.linear.x,
                                    msg.twist.linear.y,
                                    msg.twist.linear.z,
                                    msg.twist.angular.x,
                                    msg.twist.angular.y,
                                    msg.twist.angular.z] )
        elif topic == imu_data_raw_topic:
            imu_data_raw_struct.append( [msg.header.stamp.to_nsec(),
                                         msg.orientation.x,
                                         msg.orientation.y,
                                         msg.orientation.z,
                                         msg.orientation.w,
                                         msg.angular_velocity.x,
                                         msg.angular_velocity.y,
                                         msg.angular_velocity.z,
                                         msg.linear_acceleration.x,
                                         msg.linear_acceleration.y,
                                         msg.linear_acceleration.z] )



# make sure timestamps are in ascending chronological order
def assert_sorted(l):
    assert all(l[i][0] <= l[i+1][0] for i in xrange(len(l)-1))

assert_sorted(gps_struct)
assert_sorted(gps_vel_struct)
assert_sorted(imu_data_raw_struct)

# TODO: add sort if timestamp is not in ascending chronological order?


# Save to csv file
gps_data_file = os.path.join(data_dir, "gps_data.csv")

with open(gps_data_file, "w") as save_file:
    fieldnames = ["timestamp", "latitude", "longitude", "altitude"]

    writer = csv.DictWriter(save_file, fieldnames=fieldnames)
    writer.writeheader()
    
    for i in xrange(0, len(gps_struct)):
        writer.writerow({"timestamp": gps_struct[i][0],
                         "latitude": gps_struct[i][1],
                         "longitude": gps_struct[i][2],
                         "altitude": gps_struct[i][3]})


# Save to csv file
gps_vel_data_file = os.path.join(data_dir, "gps_vel_data.csv")

with open(gps_vel_data_file, "w") as save_file:
    fieldnames = ["timestamp", "twist linear x", "twist linear y", "twist linear z", "twist angular x", "twist angular y", "twist angular z"]

    writer = csv.DictWriter(save_file, fieldnames=fieldnames)
    writer.writeheader()
    
    for i in xrange(0, len(gps_vel_struct)):
        writer.writerow({"timestamp": gps_vel_struct[i][0],
                         "twist linear x": gps_vel_struct[i][1],
                         "twist linear y": gps_vel_struct[i][2],
                         "twist linear z": gps_vel_struct[i][3],
                         "twist angular x": gps_vel_struct[i][4],
                         "twist angular y": gps_vel_struct[i][5],
                         "twist angular z": gps_vel_struct[i][6]})


# Save to csv file
imu_raw_data_file = os.path.join(data_dir, "imu_raw_data.csv")

with open(imu_raw_data_file, "w") as save_file:
    fieldnames = ["timestamp", "orientation x", "orientation y", "orientation z", "orientation w", 
                               "angular velocity x", "angular velocity y", "angular velocity z", 
                               "linear acceleration x", "linear acceleration y", "linear acceleration z"]

    writer = csv.DictWriter(save_file, fieldnames=fieldnames)
    writer.writeheader()
    
    for i in xrange(0, len(imu_data_raw_struct)):
        writer.writerow({"timestamp": imu_data_raw_struct[i][0],
                         "orientation x": imu_data_raw_struct[i][1],
                         "orientation y": imu_data_raw_struct[i][2],
                         "orientation z": imu_data_raw_struct[i][3],
                         "angular velocity x": imu_data_raw_struct[i][4],
                         "angular velocity y": imu_data_raw_struct[i][5],
                         "angular velocity z": imu_data_raw_struct[i][6],
                         "linear acceleration x": imu_data_raw_struct[i][7],
                         "linear acceleration y": imu_data_raw_struct[i][8],
                         "linear acceleration z": imu_data_raw_struct[i][9]})
