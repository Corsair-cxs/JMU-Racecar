#!/usr/bin/env python
import os
map_path = "~/JMU-Racecar/src/jmu_racecar_mapping/maps/"
map_name = "map_runway_corrected_2"

# TODO gmapping
# occupied_thresh = 90
# free_thresh = 10

# TODO cartographer
# occupied_thresh = 55
# free_thresh = 30

occupied_thresh = 55
free_thresh = 30

# print("rosrun map_server map_saver"+" --occ "+str(occupied_thresh)+" --free "+str(free_thresh)+" -f "+map_path+map_name)
os.system("rosrun map_server map_saver"+" --occ "+str(occupied_thresh)+" --free "+str(free_thresh)+" -f "+map_path+map_name)