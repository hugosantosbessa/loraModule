# Reset all previously set options
reset

# Set terminal up
#set term pngcairo font "FreeSans, 10" size 1024, 768
set term postscript eps color blacktext "FreeSans-Bold" 10
set grid
set xrange [0:80]
set xtics font ",10"
set yrange [0:1]
set ytics font ",10"
set ylabel "Prob. Success (%)" font "Times-Roman-Bold,16"
set xlabel "No. of EndDevices" font "times-Roman-Bold,16"
set output '~/ns-3/TestResult/probSuccessAlarm.pdf'
set key bottom

set multiplot

# Filename of the data
filename1='~/ns-3/TestResult/test0/traffic-60/result-alarm-STAs.dat.dat'


# Plot the data
plot filename1 using 1:3 w lp lw 4 t 'Alarm'

#reset 
#set grid
#set origin 0.425,0.34
#set size 0.4,0.45
#set title 'zoom'
#set xlabel "End-Nodes"
#set ylabel ""
#set xrange [1150:1200]
#set yrange [99.9:100.1]

#plot filename1 using 1:3 w lp lw 4 notitle, filename2 using 1:3 w lp lw 4 notitle, filename3 using 1:3 w lp lw 4 notitle, filename4 using 1:3 w lp lw 4 notitle, filename5 using 1:3 w lp lw 4 notitle, filename6 using 1:3 w lp lw 4 notitle, filename7 using 1:3 w lp lw 4 notitle, filename8 using 1:3 w lp lw 4 notitle, filename9 using 1:3 w lp lw 4 notitle
