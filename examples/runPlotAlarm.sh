#!/bin/bash

# if [ $# -ne 2 ]
# then
# 	echo "parameters missing"
# 	exit 1
# fi

# trial=$1
# interval=$2


echo "##### Start Plot ProbSucc Alarm #####"
gnuplot plot-lora-probSucc-alarm-copy.gnuplot
echo "##### Finish Plot ProbSucc Alarm #####"

echo "##### Start Plot ProbSucc Regular #####"
gnuplot plot-lora-probSucc-regular-copy.gnuplot
echo "##### Finish Plot ProbSucc Regular #####"

echo "##### Start Plot endDevices #####"
# plot endDEvices
for trial in 0
do
	for numSta in {100..1500..100}
	do
			echo "numSTA:$numSta"
			file1="'~/ns-3/TestResult/plotEndDevices$numSta.pdf'"
			file2="'~/ns-3/TestResult/test0/endDevicesReg$numSta.dat'"
			file3="'~/ns-3/TestResult/test0/endDevicesAla$numSta.dat'"
			gnuplot -e "outFile=$file1; filename_edR=$file2; filename_edA=$file3" plot-topologyMTC.gnuplot	

	done
done	
echo "##### Finish Plot endDevices #####"

