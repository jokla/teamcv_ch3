import os
import csv
from math import cos, asin, sqrt
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("input_gps_interp_file", help="input csv GPS data interpolated")
parser.add_argument("output_gps_keyframes_file", help="output csv GPS keyframes")
parser.add_argument("min_distance_between_keyframes", nargs='?', default=0.1, type=float, help="output csv GPS keyframes")
args = parser.parse_args()

input_gps_interp_file = args.input_gps_interp_file
gps_data_interp = []

# load the GPS data interpolated
with open(input_gps_interp_file, 'rb') as csvfile:
  gps_data_reader = csv.reader(csvfile, delimiter=',')
  next(gps_data_reader, None)  # skip the headers

  for index, row in enumerate(gps_data_reader):
    gps_data_interp.append( [row[0], float(row[1]), float(row[2]), float(row[3])] )


# http://stackoverflow.com/a/21623206
def gps_distance(lat1, lon1, lat2, lon2):
  p = 0.017453292519943295
  a = 0.5 - cos((lat2 - lat1) * p)/2 + cos(lat1 * p) * cos(lat2 * p) * (1 - cos((lon2 - lon1) * p)) / 2
  return 12742 * asin(sqrt(a)) # distance in kilometers


min_distance_between_keyframes = args.min_distance_between_keyframes # in km
print "min_distance_between_keyframes=", min_distance_between_keyframes


def extract_keyframes1(gps_data_interp, gps_data_interp_keyframes):
  # compute the distance between the last keyframe GPS coordinate
  last_lat = 0
  last_lon = 0
  for index, row in enumerate(gps_data_interp):
    if index == 0:
      # add first data
      gps_data_interp_keyframes.append(row)
      last_lat = row[1]
      last_lon = row[2]
    elif index == len(gps_data_interp)-1:
      # add last data
      gps_data_interp_keyframes.append(row)
    elif index < len(gps_data_interp)-2:
      lat_cur  = row[1]
      lon_cur  = row[2]
      lat_next = gps_data_interp[index+1][1]
      lon_next = gps_data_interp[index+1][2]
      distance_cur  = gps_distance(last_lat, last_lon, lat_cur, lon_cur)
      distance_next = gps_distance(last_lat, last_lon, lat_next, lon_next)

      if abs(min_distance_between_keyframes-distance_cur) < abs(min_distance_between_keyframes-distance_next):
        last_lat = row[1]
        last_lon = row[2]
        gps_data_interp_keyframes.append(row)

def extract_keyframes2(gps_data_interp, gps_data_interp_keyframes):
  # compute the distance between each timestamp and sum them
  last_lat = 0
  last_lon = 0
  distance_cur = 0
  distance_next = 0

  for index, row in enumerate(gps_data_interp):
    if index == 0:
      # add first data
      gps_data_interp_keyframes.append(row)
      last_lat = row[1]
      last_lon = row[2]
    elif index == len(gps_data_interp)-1:
      # add last data
      gps_data_interp_keyframes.append(row)
    elif index < len(gps_data_interp)-2:
      lat_cur  = row[1]
      lon_cur  = row[2]
      lat_next = gps_data_interp[index+1][1]
      lon_next = gps_data_interp[index+1][2]
      distance_cur  = distance_cur + gps_distance(last_lat, last_lon, lat_cur, lon_cur) # add current distance
      distance_next = distance_cur + gps_distance(lat_cur, lon_cur, lat_next, lon_next) # add next distance

      if abs(min_distance_between_keyframes-distance_cur) < abs(min_distance_between_keyframes-distance_next):
        distance_cur = 0
        distance_next = 0
        gps_data_interp_keyframes.append(row)

      last_lat = lat_cur
      last_lon = lon_cur


# extract keyframes
gps_data_interp_keyframes = []
extract_keyframes2(gps_data_interp, gps_data_interp_keyframes)


def save_gps_csv(gps, name):
  with open(name, "w") as save_file:
    fieldnames = ["timestamp", "latitude", "longitude", "altitude"]

    writer = csv.DictWriter(save_file, fieldnames=fieldnames)
    writer.writeheader()

    for i in xrange(0, len(gps)):
      writer.writerow({"timestamp": gps[i][0],
                       "latitude": gps[i][1],
                       "longitude": gps[i][2],
                       "altitude": gps[i][3]})


# save GPS keyframes
output_gps_keyframes_file = args.output_gps_keyframes_file
save_gps_csv(gps_data_interp_keyframes, output_gps_keyframes_file)
