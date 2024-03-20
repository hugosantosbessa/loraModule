/*
 * =====================================================================================
 *
 *       Filename:  lorawan-network-mClass-sim.cc
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  02/06/2020 22:42:04
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Francisco Helder (FHC), helderhdw@gmail.com
 *   Organization:  Federal University of Ceara
 *
 * =====================================================================================
 */

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/random-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h" 
#include "ns3/node.h"
#include <algorithm>
#include <ctime>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <map>
#include <cmath>

using namespace ns3;
using namespace lorawan;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("LorawanNetworkTestAlarm");

#define MAXRTX 4

string fileMetric="./TestResult/test";
string fileData="./TestResult/test";
string endDevFileReg="./TestResult/test";
string endDevFileAla="./TestResult/test";
string gwFile="./TestResult/test";
ofstream myfile;
bool flagRtx=false; //, sizeStatus=0;
uint32_t nSeed=1;
uint8_t trial=1, numClassRegular=0, numClassAlarm=0; 
vector<uint16_t> sfQuantRegular(6,0), sfQuantAlarm(6,0), sfQuantAll(6,0);
vector<double> rtxQuant(4,0);
double packLoss=0, sent=0, received=0, avgDelay=0;
double angle=0, sAngle=M_PI; //, radius1=4200; //, radius2=4900;
double throughput=0, probSucc=0, probLoss=0;
bool connectTraceSoucer = false;
// Network settings
uint16_t nAllDevices = 200, nAlarmDevices, nRegularDevices;
uint16_t nGateways = 1;
double radius = 5600;
double gatewayRadius = 0;
uint16_t simulationTime = 600;
std::map<LoraDeviceAddress, endAlarmFCtn> mapEndAlarm, mapEndRegular;
// Channel model
bool realisticChannelModel = false;

uint16_t appPeriodSeconds = 600;

// Output control
bool printBuildings = false;
bool print = true;

enum SF { SF7=7, SF8, SF9, SF10, SF11, SF12 };

enum printMetric {ALL, REGULAR, ALARM};

bool createDirectory(std::string path) {
	int num = path.find_last_of("/");
    path = path.substr(0, num+1);
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directories(path);
		return true;
	}
	return false;
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  printEndDevices
 *  Description:  
 * =====================================================================================
 */
void PrintEndDevices (NodeContainer regularDevices, NodeContainer alarmDevices, NodeContainer gateways, std::string filename1, std::string filename2, std::string filename3){
  	const char * c = filename1.c_str ();	
	vector<int> countSF(6,0);
  	std::ofstream spreadingFactorFile;
  	spreadingFactorFile.open (c);
  	for (NodeContainer::Iterator j = regularDevices.Begin (); j != regularDevices.End (); ++j){
    	Ptr<Node> object = *j;
      	Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      	NS_ASSERT (position != NULL);
      	Ptr<NetDevice> netDevice = object->GetDevice (0);
      	Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      	NS_ASSERT (loraNetDevice != NULL);
      	Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
      	int sf = int(mac->GetSfFromDataRate(mac->GetDataRate ()));
		countSF[sf-7]++;
		//NS_LOG_DEBUG("sf: " << sf);
      	Vector pos = position->GetPosition ();
      	spreadingFactorFile << pos.x << " " << pos.y << " " << sf << endl;
  	}
	spreadingFactorFile.close ();
	c = filename2.c_str ();
  	spreadingFactorFile.open (c);
	for (NodeContainer::Iterator j = alarmDevices.Begin (); j != alarmDevices.End (); ++j){
    	Ptr<Node> object = *j;
      	Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      	NS_ASSERT (position != NULL);
      	Ptr<NetDevice> netDevice = object->GetDevice (0);
      	Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      	NS_ASSERT (loraNetDevice != NULL);
      	Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
      	int sf = int(mac->GetSfFromDataRate(mac->GetDataRate ()));
		countSF[sf-7]++;
		//NS_LOG_DEBUG("sf: " << sf);
      	Vector pos = position->GetPosition ();
      	spreadingFactorFile << pos.x << " " << pos.y << " " << sf << endl;
  	}
  	spreadingFactorFile.close ();

	c = filename3.c_str ();
  	spreadingFactorFile.open (c);
  	// Also print the gateways
  	for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j){
    	Ptr<Node> object = *j;
      	Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      	Vector pos = position->GetPosition ();
      	spreadingFactorFile << pos.x << " " << pos.y << " GW" << endl;
  	}
  	spreadingFactorFile.close ();
}


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  buildingHandler
 *  Description:  
 * =====================================================================================
 */
void buildingHandler ( NodeContainer regularDevices, NodeContainer gateways ){

	double xLength = 230;
  	double deltaX = 80;
  	double yLength = 164;
  	double deltaY = 57;
 	int gridWidth = 2 * radius / (xLength + deltaX);
  	int gridHeight = 2 * radius / (yLength + deltaY);

  	if (realisticChannelModel == false){
    	gridWidth = 0;
    	gridHeight = 0;
    }
  
	Ptr<GridBuildingAllocator> gridBuildingAllocator;
  	gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
  	gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
  	gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
  	gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
  	gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
  	gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
  	gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
  	gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
  	gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
  	gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
  	gridBuildingAllocator->SetAttribute (
      "MinX", DoubleValue (-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
  	gridBuildingAllocator->SetAttribute (
      "MinY", DoubleValue (-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
  	BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

  	BuildingsHelper::Install (regularDevices);
  	BuildingsHelper::Install (gateways);
    //BuildingsHelper::MakeMobilityModelConsistent ();

  	// Print the buildings
  	if (printBuildings){
    	std::ofstream myfile;
    	myfile.open ("buildings.txt");
      	std::vector<Ptr<Building>>::const_iterator it;
      	int j = 1;
      	for (it = bContainer.Begin (); it != bContainer.End (); ++it, ++j){
			Box boundaries = (*it)->GetBoundaries ();
        	myfile << "set object " << j << " rect from " << boundaries.xMin << "," << boundaries.yMin
                 << " to " << boundaries.xMax << "," << boundaries.yMax << std::endl;
      	}
      	myfile.close ();
    }

}/* -----  end of function buildingHandler  ----- */
 

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getPacketSizeFromSF
 *  Description:  
 * =====================================================================================
 */
uint8_t getPacketSizeFromSF (NodeContainer regularDevices, int j, bool pDiff){

	uint8_t size = 90, sf=0;

	Ptr<Node> object = regularDevices.Get(j);
    Ptr<NetDevice> netDevice = object->GetDevice (0);
    Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
    NS_ASSERT (loraNetDevice != NULL);
    Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
	sf = mac->GetSfFromDataRate(mac->GetDataRate ());

	if (pDiff){
			switch ( sf ) {
					case SF7:
							size = 90;
							break;
					case SF8:
							size = 35;
							break;
					case SF9:
							size = 5;
							break;
					default:	
							break;
			}/* -----  end switch  ----- */
	}
	
	return(size);
}/* -----  end of function getRateSF  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getShiftPosition
 *  Description:  
 * =====================================================================================
 */
Vector getShiftPosition (NodeContainer regularDevices, int j, int base){

	double radius=0, co=0, si=0;	
	Ptr<Node> object = regularDevices.Get(j);
    Ptr<MobilityModel> mobility = object->GetObject<MobilityModel> ();
    NS_ASSERT (mobility != NULL);
    Vector position = mobility->GetPosition ();

	cout << "x: " << position.x << " y: " << position.y <<endl;
	cout << "mod: " << (int)position.x/base << " mod: " << (int)position.y/base <<endl;

	radius = sqrt(pow(position.x, 2) + pow(position.y, 2));
	co = position.x/radius;
	si = position.y/radius;
	
	radius += base - (int)radius/700*700;
	position.x = radius*co;
	position.y = radius*si;

    cout << "x: " << position.x << " y: " << position.y <<endl;
	cout << sqrt(pow(position.x, 2) + pow(position.y, 2)) << endl;
	cout << endl;

	return(position);
}/* -----  end of function getRateSF  ----- */



void metricsComputation(LoraPacketTracker &tracker, enum printMetric deviceType) {
	Time appStopTime = Seconds(simulationTime);
	std::map<LoraDeviceAddress, endAlarmFCtn> mapDevices;
	uint16_t nDevices;
	vector<uint16_t> sfQuant;
	string strDevice;
	string file1 = "./TestResult/test" + to_string(trial) + "/traffic-" + to_string(appPeriodSeconds);
	string file2 = "./TestResult/test" + to_string(trial) + "/traffic-" + to_string(appPeriodSeconds);
	
	switch (deviceType){
	case ALL:
		nDevices = nAllDevices;
		sfQuant = sfQuantAll;	
		strDevice = "all";
		break;
	case REGULAR:
		mapDevices = mapEndRegular;
		nDevices = nRegularDevices;
		sfQuant = sfQuantRegular;
		strDevice = "regular";
		break;
	case ALARM:
		mapDevices = mapEndAlarm;
		nDevices = nAlarmDevices;
		sfQuant = sfQuantAlarm;
		strDevice = "alarm";
		break;
	default:
		return;
	}
	file1 +=  "/result-" + strDevice + "-STAs.dat";
	file2 +=  "/mac-STAs-GW-" + to_string(nGateways) + ".txt";
	createDirectory(file1);
	createDirectory(file2);

	NS_LOG_INFO (endl <<"//////////////////////////////////////////////");
	NS_LOG_INFO("//  METRICS FOR " << strDevice << " DEVICES  //");
	NS_LOG_INFO ("//////////////////////////////////////////////" << endl);
	NS_LOG_INFO("SF Allocation " << strDevice <<" Devices: 6 -> "<< "SF7=" << (unsigned)sfQuant.at(0) << " SF8=" << (unsigned)sfQuant.at(1) << " SF9=" << (unsigned)sfQuant.at(2)
				<< " SF10=" << (unsigned)sfQuant.at(3) << " SF11=" << (unsigned)sfQuant.at(4) << " SF12=" << (unsigned)sfQuant.at(5));		
	for(uint8_t i=SF7; i <= SF12; i++){
		if(sfQuant.at(i-SF7)) {
			NS_LOG_INFO("\n##################################################################");
			NS_LOG_INFO (endl <<"//////////////////////////////////////////////");
			NS_LOG_INFO ("//  Computing SF-"<<(unsigned)i<<" performance metrics  //");
			NS_LOG_INFO ("//////////////////////////////////////////////" << endl);

			if(deviceType == ALL)
				stringstream(tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (2), i)) >> sent >> received;
			else
				stringstream(tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (2), i, mapDevices)) >> sent >> received;

			if(flagRtx){
				stringstream(tracker.CountMacPacketsGloballyCpsr (Seconds (0), appStopTime + Hours (2), i)) >> rtxQuant.at(0) >> rtxQuant.at(1) >> rtxQuant.at(2) >> rtxQuant.at(3);
			}
			
			if(deviceType == ALL)
				stringstream(tracker.CountMacPacketsGloballyDelay (Seconds (0), appStopTime + Hours (1), (unsigned)nAllDevices, (unsigned)nGateways, i)) >> avgDelay;
			else
				stringstream(tracker.CountMacPacketsGloballyDelay (Seconds (0), appStopTime + Hours (1), (unsigned)nAllDevices, (unsigned)nGateways, i, mapDevices)) >> avgDelay;
				
			packLoss = sent - received;
			throughput = received/simulationTime;

			probSucc = received/sent;
			probLoss = packLoss/sent;
			
			NS_LOG_INFO("----------------------------------------------------------------");
			NS_LOG_INFO("nDevices" << "  |  " << "throughput" << "  |  "  << "probSucc"  << "  |  " << "probLoss" <<  "  |  " << "avgDelay"); 
			NS_LOG_INFO(nDevices  << "       |  " << throughput << "    |  " << probSucc << "   |  " << probLoss << "   |  " << avgDelay);
			NS_LOG_INFO("----------------------------------------------------------------"<< endl);

			myfile.open (file1+"-SF"+to_string(i)+".dat", ios::out | ios::app);
			myfile << nDevices << ", " << throughput << ", " << probSucc << ", " <<  probLoss  << ", " << avgDelay << "\n";
			myfile.close();  

			if (flagRtx){
				sort(rtxQuant.begin(),rtxQuant.end(), greater<double>());		
				myfile.open (file1+"-RTX"+to_string(i)+".dat", ios::out | ios::app);
				myfile << nDevices << ", " << sent;
				for(uint8_t i=0; i<rtxQuant.size(); i++)
					myfile << ", " << rtxQuant[i];
				myfile << "\n";
				myfile.close(); 
			}

			NS_LOG_INFO("numDev:" << nDevices << " numGW:" << nGateways << " simTime:" << simulationTime << " throughput:" << throughput);
			NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
			NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:"<< packLoss  << "   delay:" << avgDelay);
			NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);
			NS_LOG_INFO("##################################################################\n");
			myfile.open (file2, ios::out | ios::app);
			myfile << "sent: " << sent << " succ: " << received << " drop: "<< packLoss << "\n";
			myfile << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << "\n";
			myfile << "numDev: " << nDevices << " numGat: " << nGateways << " simTime: " << simulationTime << " throughput: " << throughput<< "\n";
			myfile << "##################################################################" << "\n\n";
			myfile.close();
		} else {
			NS_LOG_INFO("\n##################################################################");
			NS_LOG_INFO (endl <<"//////////////////////////////////////////////");
			NS_LOG_INFO ("//  No devices for SF-"<<(unsigned)i<<"  //");
			NS_LOG_INFO ("//////////////////////////////////////////////" << endl);
			NS_LOG_INFO("##################################################################\n");
		}
	}
  
	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  	NS_LOG_INFO ("Computing system performance metrics");

	if(deviceType == ALL)
		stringstream(tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (1))) >> sent >> received;
	else
		stringstream(tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (1), mapDevices)) >> sent >> received;

	if(deviceType == ALL)
		stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0), appStopTime + Hours(1), (unsigned)nAllDevices, (unsigned)nGateways)) >> avgDelay;
	else
		stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0), appStopTime + Hours(1), (unsigned)nAllDevices, (unsigned)nGateways, mapDevices)) >> avgDelay;

	packLoss = sent - received;
  	throughput = received/simulationTime;

  	probSucc = received/sent;
  	probLoss = packLoss/sent;

	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
   	NS_LOG_INFO("nDevices: " << nDevices); 
	NS_LOG_INFO("throughput: " << throughput); 
	NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc*100 << "%)"); 
	NS_LOG_INFO("probLoss: " <<  probLoss<< " (" << probLoss*100 << "%)"); 
	NS_LOG_INFO("avgDelay: " << avgDelay); 
	NS_LOG_INFO("----------------------------------"<< endl);

	myfile.open (file1+"-RTX"+".dat", ios::out | ios::app);
	myfile << nDevices << ", " << sent;
	myfile << "\n";
	myfile.close(); 
	
	myfile.open (file1+".dat", ios::out | ios::app);
	myfile << nDevices << ", " << throughput << ", " << probSucc << ", " <<  probLoss  << ", " << avgDelay << "\n";
 	myfile.close();  

   	NS_LOG_INFO("numDev:" << nDevices << " numGW:" << unsigned(nGateways) << " simTime:" << simulationTime << " throughput:" << throughput);
  	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  	NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:"<< packLoss  << "   delay:" << avgDelay);
  	NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);
}

void OnReceivePacket(Ptr<const Packet> packet, uint32_t num) {
	std::cout << "Receive Packet: " << packet<< "\n";
	Ptr<Packet> packetCopy = packet->Copy();
   	LorawanMacHeader mHdr;
	LoraFrameHeader fHdr;
	packetCopy->RemoveHeader(mHdr);
	packetCopy->RemoveHeader(fHdr);
	if(mHdr.IsUplink()){
		fHdr.SetAsUplink();
		LoraDeviceAddress address = fHdr.GetAddress ();
		if (mapEndAlarm.find(address) != mapEndAlarm.end()) {
			mapEndAlarm[address].FCtn = fHdr.GetFCnt();
			// NS_LOG_DEBUG("Frame Counter " << fHdr.GetFCnt() << " Node Addr: " << address << "\n");
			std::cout << "Node ID: " << mapEndAlarm[address].id << ", Frame Counter: " << fHdr.GetFCnt() << ", Device Addr: " << address << "\n";
		}
	}
}

void OnReceivePacketMac(Ptr<const Packet> packet) {
	std::cout << "Receive Packet: " << packet<< "\n";
	Ptr<Packet> packetCopy = packet->Copy();
   	LorawanMacHeader mHdr;
	LoraFrameHeader fHdr;
	packetCopy->RemoveHeader(mHdr);
	packetCopy->RemoveHeader(fHdr);
	if(mHdr.IsUplink()){
		fHdr.SetAsUplink();
		LoraDeviceAddress address = fHdr.GetAddress ();
		if (mapEndAlarm.find(address) != mapEndAlarm.end()) {
			mapEndAlarm[address].FCtn = fHdr.GetFCnt();
			// NS_LOG_DEBUG("Frame Counter " << fHdr.GetFCnt() << " Node Addr: " << address << "\n");
			std::cout << "Node ID: " << mapEndAlarm[address].id << ", Frame Counter: " << fHdr.GetFCnt() << ", Device Addr: " << address << "\n";
		}
	}
}

void OnStartingSendPacket(Ptr<const Packet> packet, uint32_t num) {
	Ptr<Packet> packetCopy = packet->Copy();
   	LorawanMacHeader mHdr;
	LoraFrameHeader fHdr;
	packetCopy->RemoveHeader(mHdr);
	packetCopy->RemoveHeader(fHdr);
	if(mHdr.IsUplink()){
		fHdr.SetAsUplink();
		LoraDeviceAddress address = fHdr.GetAddress ();
		if (mapEndAlarm.find(address) != mapEndAlarm.end()) {
			mapEndAlarm[address].FCtn = fHdr.GetFCnt();
			// NS_LOG_DEBUG("Frame Counter " << fHdr.GetFCnt() << " Node Addr: " << address << "\n");
			std::cout << "Node ID: " << mapEndAlarm[address].id << ", Frame Counter: " << fHdr.GetFCnt() << ", Device Addr: " << address << "\n";
		}
	}

}


int main (int argc, char *argv[]){
	CommandLine cmd;
  	cmd.AddValue ("nSeed", "Number of seed to position", nSeed);
  	cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nAllDevices);
  	cmd.AddValue ("nGateways", "Number of gateway rings to include", nGateways);
  	cmd.AddValue ("radius", "The radius of the area to simulate", radius);
  	cmd.AddValue ("gatewayRadius", "The distance between gateways", gatewayRadius);
  	cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  	cmd.AddValue ("appPeriod", "The period in seconds to be used by periodically transmitting applications", appPeriodSeconds);
  	cmd.AddValue ("file1", "files containing result data", fileMetric);
  	cmd.AddValue ("file2", "files containing result information", fileData);
  	cmd.AddValue ("print", "Whether or not to print various informations", print);
  	cmd.AddValue ("trial", "set trial parameter", trial);
  	cmd.Parse (argc, argv);

	endDevFileReg += to_string(trial) + "/endDevicesReg" + to_string(nAllDevices) + ".dat";
	endDevFileAla += to_string(trial) + "/endDevicesAla" + to_string(nAllDevices) + ".dat";
	gwFile += to_string(trial) + "/GWs" + to_string(nGateways) + ".dat";
	
	createDirectory(endDevFileReg);
	createDirectory(endDevFileAla);
	createDirectory(gwFile);


  	// Set up logging
	LogComponentEnable ("LorawanNetworkTestAlarm", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraChannel", LOG_LEVEL_DEBUG);
	// LogComponentEnable("LoraChannel", LOG_PREFIX_TIME);
	// LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
	// LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
	// LogComponentEnable("SimpleEndDeviceLoraPhy", LOG_LEVEL_ALL);
	// LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
	// LogComponentEnable("SimpleGatewayLoraPhy", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
	// LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
	// LogComponentEnable("EndDeviceStatus", LOG_LEVEL_ALL);
	// LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
	// LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
	// LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
	// LogComponentEnable("RandomSenderHelper", LOG_LEVEL_ALL);
	// LogComponentEnable("RandomSender", LOG_LEVEL_ALL);
	// LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
	// LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
	// LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
	// LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
	// LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
	// LogComponentEnable("NetworkController", LOG_LEVEL_ALL);

  	/***********	
   	*  Setup  *
   	***********/
	

  	RngSeedManager::SetSeed(1);
  	RngSeedManager::SetRun(nSeed);

  	// Create the time value from the period
  	Time appPeriod = Seconds (appPeriodSeconds);

  	// Mobility
  	MobilityHelper mobility;
  	mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (radius),
                                 "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
  	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  	/************************
   	*  Create the channel  *
   	************************/

  	// Create the lora channel object
  	Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  	loss->SetPathLossExponent (3.76);
  	loss->SetReference (1, 7.7);

  	if (realisticChannelModel){
      	// Create the correlated shadowing component
      	Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
          	CreateObject<CorrelatedShadowingPropagationLossModel> ();

      	// Aggregate shadowing to the logdistance loss
      	loss->SetNext (shadowing);

      	// Add the effect to the channel propagation loss
      	Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

      	shadowing->SetNext (buildingLoss);
    }

  	Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  	Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  	/************************
   	*  Create the helpers  *
   	************************/

  	// Create the LoraPhyHelper
  	LoraPhyHelper phyHelper = LoraPhyHelper ();
  	phyHelper.SetChannel (channel);

 	// Create the LorawanMacHelper
  	LorawanMacHelper macHelper = LorawanMacHelper ();

  	// Create the LoraHelper
  	LoraHelper helper = LoraHelper ();
  	helper.EnablePacketTracking (); // Output filename
  	// helper.EnableSimulationTimePrinting ();

  	//Create the NetworkServerHelper
  	NetworkServerHelper nsHelper = NetworkServerHelper ();

  	//Create the ForwarderHelper
  	ForwarderHelper forHelper = ForwarderHelper ();

  	/************************
   	*  Create End Devices  *
   	************************/

  	// Create a set of nodes
  	NodeContainer regularDevices;
	NodeContainer alarmDevices;
	NodeContainer allDevices;
	nRegularDevices = nAllDevices*0.95;
	nAlarmDevices = ceil(nAllDevices*0.05);
	NS_LOG_DEBUG("Nº Regular Devices: " << nRegularDevices << ", Nº Alarm Devices: " << nAlarmDevices << 
				", Total Devices: " << nAllDevices << std::endl);
  	regularDevices.Create (nRegularDevices);
	alarmDevices.Create (nAlarmDevices);
	allDevices.Add(regularDevices);
	allDevices.Add(alarmDevices);
  	// Assign a mobility model to each node
	mobility.Install (allDevices);
  	// Make it so that nodes are at a certain height > 0
  	for (NodeContainer::Iterator j = allDevices.Begin (); j != allDevices.End (); ++j){
      	Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      	Vector position = mobility->GetPosition ();
 		position.z = 1.2;
      	mobility->SetPosition (position);
	}

  	// Create the LoraNetDevices of the end devices
  	uint8_t nwkId = 54;
  	uint32_t nwkAddr = 1864;
  	Ptr<LoraDeviceAddressGenerator> addrGen =
    	  CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

 	// Create the LoraNetDevices of the end devices
  	macHelper.SetAddressGenerator (addrGen);
  	phyHelper.SetDeviceType (LoraPhyHelper::ED);
  	macHelper.SetDeviceType (LorawanMacHelper::ED_A);
   	//macHelper.SetRegion (LorawanMacHelper::SingleChannel);
	helper.Install (phyHelper, macHelper, allDevices);
	
	// Now end devices are connected to the channel
  	// Connect trace sources
  	// for (NodeContainer::Iterator j = allDevices.Begin (); j != allDevices.End (); ++j){
    //   	Ptr<Node> node = *j;
    //   	Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
    //   	Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
 
    //   	if (flagRtx){
    //   		Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac>();
	//   		mac->SetMaxNumberOfTransmissions (MAXRTX);
	//   		mac->SetMType (LorawanMacHeader::CONFIRMED_DATA_UP);
	//   	}
    // }
  	

	for (NodeContainer::Iterator j = regularDevices.Begin (); j != regularDevices.End (); ++j){
		Ptr<Node> node = *j;
		Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
		Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac>();
		mapEndRegular[mac->GetDeviceAddress()] = endAlarmFCtn{(uint16_t)node->GetId(), 0};
	}
  	
	for (NodeContainer::Iterator j = alarmDevices.Begin (); j != alarmDevices.End (); ++j){
		Ptr<Node> node = *j;
		Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
		Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac>();
		mapEndAlarm[mac->GetDeviceAddress()] = endAlarmFCtn{(uint16_t)node->GetId(), 0};
	}
	
	if(connectTraceSoucer){
		if(Config::ConnectWithoutContextFailSafe("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Phy/$ns3::LoraPhy/ReceivedPacket", MakeCallback (&OnReceivePacket)))
			NS_LOG_DEBUG("Conect Trace Source LoraPhy/ReceivedPacket!\n"); 
		else
			NS_LOG_DEBUG("Not Conect Trace Source LoraPhy/ReceivedPacket!\n");	
		if(Config::ConnectWithoutContextFailSafe("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Phy/$ns3::LoraPhy/StartSending", MakeCallback (&OnStartingSendPacket)))
			NS_LOG_DEBUG("Conect Trace Source LoraPhy/StartSending!\n"); 
		else
			NS_LOG_DEBUG("Not Conect Trace Source LoraPhy/StartSending!\n");
		if(Config::ConnectWithoutContextFailSafe("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::LorawanMac/ReceivedPacket", MakeCallback (&OnReceivePacketMac)))
			NS_LOG_DEBUG("Conect Trace Source LorawanMac/ReceivedPacket!\n"); 
		else
			NS_LOG_DEBUG("Not Conect Trace Source LorawanMac/ReceivedPacket!\n");	
	}

	

  	/*********************
   	*  Create Gateways  *
   	*********************/

  	// Create the gateway nodes (allocate them uniformely on the disc)
  	NodeContainer gateways;
  	gateways.Create (nGateways);

    sAngle = (2*M_PI)/nGateways;  
   
  	Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  	// Make it so that nodes are at a certain height > 0
  	allocator->Add (Vector (0.0, 0.0, 0.0));
  	mobility.SetPositionAllocator (allocator);
  	mobility.Install (gateways);

  	// Make it so that nodes are at a certain height > 0
  	for (NodeContainer::Iterator j = gateways.Begin ();
    	j != gateways.End (); ++j){
      	Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      	Vector position = mobility->GetPosition ();
		position.x = gatewayRadius * cos(angle); 
  		position.y = gatewayRadius * sin(angle); 
      	position.z = 15;
      	mobility->SetPosition (position);
		angle += sAngle;
	}

  	// Create a netdevice for each gateway
  	phyHelper.SetDeviceType (LoraPhyHelper::GW);
  	macHelper.SetDeviceType (LorawanMacHelper::GW);
  	helper.Install (phyHelper, macHelper, gateways);

	/**********************
   	*  Handle buildings  *
   	**********************/
	buildingHandler(allDevices, gateways);	
 
  	/**********************************************
   	*  Set up the end device's spreading factor  *
   	**********************************************/
    
	sfQuantRegular = macHelper.SetSpreadingFactorsUp (regularDevices, gateways, channel);
	sfQuantAlarm = macHelper.SetSpreadingFactorsUp (alarmDevices, gateways, channel);
	// sfQuant = macHelper.SetSpreadingFactorsUp (alarmDevices, gateways, channel);
	//sfQuant = macHelper.SetSpreadingFactorsEIB (regularDevices, radius);
	//sfQuant = macHelper.SetSpreadingFactorsEAB (regularDevices, radius);
	//sfQuant = macHelper.SetSpreadingFactorsProp (regularDevices, 0.4, 0, radius);
  	//sfQuant = macHelper.SetSpreadingFactorsStrategies (regularDevices, sfQuant, 0.76*nAllDevices, 0*nAllDevices, nAllDevices, LorawanMacHelper::CLASS_TWO);

	for(uint16_t i=0; i<sfQuantRegular.size(); i++)
		sfQuantRegular.at(i)?numClassRegular++:numClassRegular;
	
	for(uint16_t i=0; i<sfQuantAlarm.size(); i++)
		sfQuantAlarm.at(i)?numClassAlarm++:numClassAlarm;

	for(uint16_t i=0; i<sfQuantAll.size(); i++)
		sfQuantAll.at(i) = sfQuantAlarm.at(i) + sfQuantRegular.at(i);


  	NS_LOG_DEBUG ("Completed configuration");

  	/*********************************************
   	*  Install applications on the end devices  *
   	*********************************************/

  	Time appStopTime = Seconds (simulationTime);
 
 	PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  	appHelper.SetPeriod (Seconds (appPeriodSeconds));	
	appHelper.SetPacketSize (19);
  	ApplicationContainer appEndDevicesContainer = appHelper.Install (regularDevices);

	appEndDevicesContainer.Start (Seconds (0));
  	appEndDevicesContainer.Stop (appStopTime);

	RandomSenderHelper appRandomHelper = RandomSenderHelper ();
  	appRandomHelper.SetMean (appPeriodSeconds);
	appRandomHelper.SetBound (appPeriodSeconds);
	appRandomHelper.SetPacketSize (19);
  	ApplicationContainer appAlarmContainer = appRandomHelper.Install (alarmDevices);

	appAlarmContainer.Start (Seconds (0));
  	appAlarmContainer.Stop (appStopTime);

  	/**************************
   	*  Create Network Server  *
   	***************************/

  	// Create the NS node
  	NodeContainer networkServer;
  	networkServer.Create (1);

  	// Create a NS for the network
	nsHelper.SetEndDevices (allDevices);
  	nsHelper.SetGateways (gateways);
  	nsHelper.Install (networkServer);

  	//Create a forwarder for each gateway
 	forHelper.Install (gateways);

 	/**********************
   	* Print output files *
   	*********************/
  	if (print){
    	PrintEndDevices (regularDevices, alarmDevices, gateways, endDevFileReg, endDevFileAla, gwFile);
 	}

  	////////////////
  	// Simulation //
  	////////////////

  	Simulator::Stop (appStopTime + Seconds (10));

  	NS_LOG_INFO ("Running simulation...");
  	Simulator::Run ();

  	Simulator::Destroy ();

  	/////////////////////////////
  	// Metrics results to file //
  	//////////////////////////////

	metricsComputation(helper.GetPacketTracker(), ALARM);
	metricsComputation(helper.GetPacketTracker(), REGULAR);
	metricsComputation(helper.GetPacketTracker(), ALL);
  	return(0);
}
