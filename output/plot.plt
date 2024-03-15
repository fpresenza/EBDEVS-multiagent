set title 'Simple Oscillator'

set xlabel "time (sec)"
set ylabel "states"
set xrange [0:10]
set yrange [-2:2]

set key box

set grid
set datafile separator ","

plot  "output.csv" using 1:2 with steps title 'x1',\
      "output.csv" using 1:3 with steps title 'x2',\
      "output.csv" using 1:4 with steps title 'x3',\
      "output.csv" using 1:5 with steps title 'x4',\
      "output.csv" using 1:6 with steps title 'x5',\
      "output.csv" using 1:7 with steps title 'x6',\
      "output.csv" using 1:8 with steps title 'x7',\
      "output.csv" using 1:9 with steps title 'x8'

pause -1
