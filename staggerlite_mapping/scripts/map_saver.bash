#! /bin/bash
map_dir=${1:-../../staggerlite_gazebo/maps/mymap}
rosrun map_server map_saver --occ 65 --free 25 -f ${map_dir} map:=/map