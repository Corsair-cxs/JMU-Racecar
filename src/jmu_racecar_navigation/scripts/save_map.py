#!/usr/bin/env python
import os
map_path = "~/map_saves/"
map_name = "map_2019_3"
os.system("rosrun map_server map_saver -f "+map_path+map_name)