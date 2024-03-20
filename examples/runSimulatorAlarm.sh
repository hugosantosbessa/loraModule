#!/bin/bash

if [ $# -ne 8 ]
then
	echo "parameters missing"
	exit 1
fi

gwRing=$2
rad=$3
gwRad=$4
simTime=$5
interval=$6
pEDs=$7
trial=$8

echo "##### Simulation Start #####"

if [ ! -d TestResult/ ]
then
	mkdir TestResult/
fi

RANDOM=$$

#if [ $trial -eq 2 ]
#then
#	gwRing=1
#	gwRad=0
#elif [ $trial -eq 1 ]
#then
#	gwRing=2
#	gwRad=0
#else
#	gwRing=2
#	gwRad=4000
#fi  

if [ ! -d TestResult/test$trial/ ]
	then
	mkdir TestResult/test$trial/
fi

if [ ! -d TestResult/test$trial/traffic-$interval/ ]
	then
	mkdir TestResult/test$trial/traffic-$interval/
fi

if [ $1 -eq 0 ]
then

	# file1="./TestResult/test$trial/traffic-$interval/result-STAs"
	# echo "#numSta, Throughput(Kbps), ProbSucc(%), ProbLoss(%), avgDelay(Seconds)" > ./TestResult/test$trial/traffic-$interval/result-STAs-SF7.dat 
		
	# touch ./TestResult/test$trial/traffic-$interval/mac-STAs-GW-$gwRing.txt
	# file2="./TestResult/test$trial/traffic-$interval/mac-STAs-GW-$gwRing.txt"

	for numSta in {100..1500..100}
	do
			echo "trial:$trial-numSTA:$numSta #"

			if [ ! -d TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/ ]
			then
				mkdir TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/
			fi

			touch TestResult/test$trial/time-record$numSta.txt

			echo "Time: $(date) $interval $numSta" >> TestResult/test$trial/time-record$numSta.txt

		for numSeed in {1..5}
		do
			echo -ne "$numSeed \r"
 			./ns3 run "lorawan-network-test-alarm --nSeed=$RANDOM --nDevices=$numSta --nGateways=$gwRing --radius=$rad --gatewayRadius=$gwRad --simulationTime=$simTime --appPeriod=$interval --print=$pEDs --trial=$trial" > ./TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/record-$numSta.txt 2>&1
		done
	done
fi


echo "##### Simulation finish #####"



