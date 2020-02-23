FN='../maps/arena/wps.txt'
rostopic echo -p /clicked_point  | cut -d ',' -f 5,6 
