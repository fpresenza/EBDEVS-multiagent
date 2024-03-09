set title 'Simple Oscillator'

set xlabel "time (sec)"
set ylabel "states"
set xrange [0:10]
set yrange [-2:2]

set key box

set grid
set datafile separator ","

plot  "output.csv" using 1:2 with steps title 'x1',\
      "output.csv" using 1:3 with steps title 'x2'


pause -1
