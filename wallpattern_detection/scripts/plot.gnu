#set terminal fig color 
dat='pos0'
plot [-10:50] [-10:50]\
dat.'_drone.txt' using 4:5 with lines lc 1 title '0',\
dat.'_red.txt' using 4:5 lc 1 title '0',\
dat.'_blue.txt' using 4:5 lc 3 title '0',\
dat.'_black.txt' using 4:5 lc 0 title '0',\
dat='pos1',\
dat.'_drone.txt' using 4:5 with lines title '1' ,\
dat.'_red.txt' using 4:5  title '1' ,\
dat.'_blue.txt' using 4:5 title '1' ,\
dat.'_black.txt' using 4:5 title '1' 
