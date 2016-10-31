import os
import csv
import argparse
import shutil
import sys


parser = argparse.ArgumentParser()
parser.add_argument("gps_keyframes_file", help="csv GPS keyframes")
parser.add_argument("image_dir", help="directory that contains the extracted images")
parser.add_argument("keyframe_image_dir", help="directory that will contain the keyframe images")
args = parser.parse_args()


gps_data_keyframes = []
gps_keyframes_file = args.gps_keyframes_file

# load the GPS data keyframes
with open(gps_keyframes_file, 'rb') as csvfile:
  gps_data_reader = csv.reader(csvfile, delimiter=',')
  next(gps_data_reader, None)  # skip the headers

  for index, row in enumerate(gps_data_reader):
    gps_data_keyframes.append( [row[0], float(row[1]), float(row[2]), float(row[3])] )


# directory that contains all the images extracted
image_dir = args.image_dir

# directory to copy the keyframe images
keyframe_image_dir = args.keyframe_image_dir
if not os.path.exists(keyframe_image_dir):
  os.makedirs(keyframe_image_dir)

# copy keyframe images
for row in gps_data_keyframes:
  image_filename = os.path.join(image_dir, row[0] + ".jpeg")
  dst_image_filename = os.path.join(keyframe_image_dir, row[0] + ".jpeg")
  try:
    shutil.copyfile(image_filename, dst_image_filename)
  except:
    print "image_filename=", image_filename, " ; dst_image_filename=", dst_image_filename
    print "Unexpected error:", sys.exc_info()[0]
    raise
