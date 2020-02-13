grep T1 $1.txt >$1_black.txt
grep T2 $1.txt >$1_red.txt
grep T3 $1.txt >$1_blue.txt
grep T0 $1.txt|grep -v '0.000 0.000'  >$1_drone.txt 
