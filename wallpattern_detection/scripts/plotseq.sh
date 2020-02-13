n=$(cat aha.txt|grep -n $2|grep T0|sed s/\:.*//|cut -f 1 -d ' ')
echo $n
cat aha.txt|head -n $n |grep T1 >$1_black.txt
cat aha.txt|head -n $n |grep T2 >$1_red.txt
cat aha.txt|head -n $n |grep T3 >$1_blue.txt
cat aha.txt|head -n $n |grep T0|grep -v '0.000 0.000' >$1_drone.txt 
