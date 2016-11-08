#extract images from rosbag
$ ~/catkin_ws/src/challenge2/scripts/Ch2-Train$ python dump_images_ch2_train.py

# Extract gps interpolated
$ ~/catkin_ws/src/challenge2/scripts/Ch2-Train$ python 


# Create Sift files for pictures:
$ ls -d $PWD/*.jpg > image_list

$ ./libvot_feature /home/jokla/catkin_ws/src/challenge2/src/test/test_pic/image_list

$ ls -d $PWD/*.sift > sift_list

# Create database:
$ ./image_search /home/jokla/catkin_ws/src/challenge2/src/test/test_pic/sift_list /home/jokla/catkin_ws/src/challenge2/src/test/test_pic/database

# Query one images:
./web_search /home/jokla/catkin_ws/src/challenge2/src/test/test_pic/1476996520856905303.sift /home/jokla/catkin_ws/src/challenge2/src/test/test_pic/image_list /home/jokla/catkin_ws/src/challenge2/src/test/test_pic/database/db.out
