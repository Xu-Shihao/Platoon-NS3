/*
 *
 *
 *  Created on: Apr 5, 2016
 *      Author: apratim
 */

// Tjhe following code will utilize the waypoint mobility model instead of the constantpositionmobilitymodel as was being used earlier

#include <iostream>
#include <algorithm>
#include <vector>
#include <queue>
#include <map>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/node-container.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/waypoint-mobility-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "PlatoonGroup.h"

//#include "ns3/traced-value.h"
//#include "ns3/constant-velocity-mobility-model.h"
NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");
//using namespace ns3;
//using namespace std;

//namespace ns3
//{

//using namespace ns3;


 //using std::queue;
namespace ns3
{
PlatoonGroup* plat = NULL;
// Function to track if vehicle nodes are changing course
void CourseChange(std::string context, Ptr<const MobilityModel>model)
{

	//cout << "Blah" << endl;
	Vector position = model->GetPosition();
	NS_LOG_UNCOND (context << "  x= "  << position.x << ", y= " <<position.y);

}


// Structure of vehicle nodes
/*struct veh_Nodes{

	Ptr<Node> veh_node;
	Ptr<Socket> veh_socket;
	int veh_number;
	bool in_use;

}; */


// Pause will be responsibe for executing three events, attaching vehicle to nodes, updating positions of existing vehicles, removing vehciels from nodes

static void Pause(TCPStream* stream, std::queue<veh_Nodes>* vehicles, std::map<int, double>* numb_pos_pair)
{
	//cout << "In Pause" << endl;
        size_t length;
        char buffer[4096];
   	int vehicle_number;
	double vehicle_pos;
	double first_data;
	//map<int, double> numb_pos_pair;
        //printf("Inside Pause\n");
	ns3::Time firstSchedule;
	//PlatoonGroup* plat = NULL;
       //The code pauses at the "receive()" function)
	int platoon_size;
	//cout << stream <<endl;
       if (stream != NULL)
            {
                if((length=stream->receive(buffer, sizeof(buffer))) >0)
                {

			//cout << "In here" << endl;
			buffer[length]='\0';
			char *saveptr = buffer;
			char * pch = NULL;
			bool veh_found = 0;
			bool initial = false;
			
			pch = strtok_r(buffer,"-", &saveptr);
			if (Simulator::Now ().GetSeconds () == 0.0)
				{
					//cout<< "In get second = 0" << endl;
					char * sz;
					//first_data = atof(c_str(pch));
					first_data =strtod(pch,&sz);
					firstSchedule = Seconds(first_data);
					pch = strtok_r(NULL, "-", &saveptr);
					platoon_size = atoi(pch);
                			pch = strtok_r(NULL, "-", &saveptr);
				}

			while (pch != NULL)
				{
					//cout << "In separate"<< endl;
					char* vehicle_info = new char[sizeof(pch)+1];
					char* temp = NULL;
					strcpy(vehicle_info, pch);
					pch = strtok_r(NULL, "-", &saveptr);
					temp = strtok(vehicle_info,",");
					vehicle_number = atoi(temp);
					//cout << vehicle_number<< endl;
					//vehicleNumbers.push_back(vehicle_number);
					while (temp != NULL)
					{

						vehicle_pos = atoi(temp);
						//cout << vehicle_pos<< endl;
						// vehicleNumbers.push_back(vehicle_number);
						temp = strtok(NULL,",");

					}

					numb_pos_pair->insert(make_pair(vehicle_number, vehicle_pos));

				/*	for (int j = 1 ; j < 101 ; j++)
					{

						if (vehicle_number == (*vehicles)[j].veh_number)
						{
							Simulator::ScheduleNow(&ns3::SetPosition_next,(*vehicles)[j].veh_node, Vector(vehicle_pos,0.0,0.0), (*vehicles)[j].veh_number);
							veh_found = 1;
							break;
						}
					}

					if (veh_found == 1)
					{
						veh_found = 0;
					}
					else // Assign it to a node
					{

						for (int j = 1 ; j < 101 ; j++)
						{
						if ((*vehicles)[j].in_use == 0)
							{

									if (Simulator::Now ().GetSeconds () == 0.0)
										{
											initial = true;
											Simulator::Schedule(firstSchedule, &attach_vehicle_to_node, stream, &(*vehicles)[j], vehicle_number, vehicle_pos, initial);

										}
									else
										attach_vehicle_to_node(stream, &(*vehicles)[j], vehicle_number, vehicle_pos, initial);

									break;

							}
						}

					}  */


				}

			if (Simulator::Now ().GetSeconds () == 0.0)
			{
				plat = new PlatoonGroup(platoon_size, vehicles, numb_pos_pair, firstSchedule, stream);
				//cout<< "after constructor" << endl;
				size_t length;
				char buffer[1024];
		   		int vehicle_number;
				double vehicle_pos;
				buffer[0]='\0';
		//cout<< "waiting for second waypoint"<<endl;
				// Second set of position arrive for the first vehicle
				if((length=stream->receive(buffer, sizeof(buffer))) >0)
				{


					buffer[length]='\0';
					char *saveptr = buffer;
					char * pch = NULL;

                    /// Continue checking code from here once you return from gym

					pch = strtok_r(buffer,"-", &saveptr);

					while (pch != NULL)
						{

							char* vehicle_info = new char[sizeof(pch)+1];
							char* temp = NULL;
							strcpy(vehicle_info, pch);
							pch = strtok_r(NULL, "-", &saveptr);
							temp = strtok(vehicle_info,",");
							vehicle_number = atoi(temp);
							//cout << vehicle_number<< endl;
							while (temp != NULL)
							{

								vehicle_pos = atoi(temp);
								//cout << vehicle_pos<< endl;
								temp = strtok(NULL,",");

							}
                            				numb_pos_pair->insert(make_pair(vehicle_number, vehicle_pos));

						}
                    			plat -> SetPosition_next_initial(numb_pos_pair, firstSchedule);
					//buffer[0]='\0';
				}

			}
			else
			{

				plat -> SetPosition_next(numb_pos_pair);

			}

			numb_pos_pair->erase ( numb_pos_pair->begin(), numb_pos_pair->end() );
		}
		buffer[0]='\0';
		}

	     //}


	//Schedule a Pause event every 0.1 second since the beginning of the simulations, i.e. since the arrival of the first vehicle in the network


	if (Simulator::Now ().GetSeconds () == 0.0)
	{
		//cout << "next pause schedule" << endl;
		Simulator::Schedule(firstSchedule + Seconds(0.1), &Pause, stream, vehicles, numb_pos_pair);
		
		//plat -> schedule_first_event(firstSchedule, 1000, 1);

		plat -> schedule_first_event(first_data, 500, 1);


	}

	else
	{
		Simulator::Schedule(Seconds(0.1), &Pause, stream, vehicles, numb_pos_pair);
	}

}
}
//using std::queue;
int main(int argc, char* argv[])
{
    
	
	using namespace ns3;
     
    if (argc < 2 || argc > 4) {
        printf("usage: server <port> [<ip>]\n");
        exit(1);
	 }

    TCPStream* stream = NULL;
    TCPAcceptor* acceptor = NULL;

    if (argc == 3) {
        acceptor = new TCPAcceptor(atoi(argv[1]), argv[2]);
    }
    else {
        acceptor = new TCPAcceptor(atoi(argv[1]));
    }

 // Acceptor will not start listening for connections until simulation configuration is complete in NS3

		int numVehicleNodes= 11;


		//numVehicleNodes=atoi(line);
			//x=2;

			printf("Number of nodes are %d\n", numVehicleNodes);

			std::string phyMode ("OfdmRate6MbpsBW10MHz");
			uint32_t packetSize = 500; // bytes
			uint32_t numPackets = 1;
		        double interval = 0.1; // seconds
			bool verbose = false;

			CommandLine cmd;

		  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
		  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
		  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
		  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
		  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
		  cmd.Parse (argc, argv);
		  // Convert to time object

		  Time interPacketInterval = Seconds (interval);
		  NodeContainer c ;
			//NodeContainer c;
		  c.Create (numVehicleNodes);

		 // double txPower = 27;
		//NodeContainer = NodeContainer::GetGlobal();
		  // The below set of helpers will help us to put together the wifi NICs we want
		  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();

		  wifiPhy.Set ("TxPowerStart", DoubleValue (23));
		  wifiPhy.Set ("TxPowerEnd", DoubleValue (23));
		  wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
		  wifiPhy.Set ("TxGain", DoubleValue (0));
		  wifiPhy.Set ("RxGain", DoubleValue (0));
		  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-87.0));
		  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/DcaTxop/Queue/MaxDelay",TimeValue(Seconds(0.1))); //changed to 0.2 sec
 		  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/CcaMode1Threshold",DoubleValue (-90.0));
		  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
		//  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
		//  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");

		  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
   		  wifiChannel.AddPropagationLoss("ns3::ThreeLogDistancePropagationLossModel","Exponent0",DoubleValue (1.9),"Distance0",DoubleValue (10),"Exponent1",DoubleValue (3.8),"Distance1",DoubleValue(80), 							"ReferenceLoss",DoubleValue (61.8));
   		  wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel","m0", DoubleValue(3), "m1", DoubleValue(1.5), "m2", DoubleValue(1),"Distance1",DoubleValue (50),"Distance2",DoubleValue(150)); 
		  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
		  wifiPhy.SetChannel (channel);
		  // ns-3 supports generate a pcap trace
		  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

		  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
		  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
		 
		  if (verbose)
		    {
		      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
		    }

		  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
				                      "DataMode",StringValue (phyMode),
				                      "ControlMode",StringValue (phyMode));
		  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

		  AsciiTraceHelper ascii;
		  wifiPhy.EnableAsciiAll (ascii.CreateFileStream("Interface.tr"));


               // In this case a constant position monbiltiy model will be installed only on the zeroth node representing the RSU while all other nodes will have the waypoint mobility model

		  MobilityHelper mobility;

    		  mobility.SetMobilityModel("ns3::WaypointMobilityModel");

		  mobility.Install (c);

		  InternetStackHelper internet;
		  internet.Install (c);

		  Ipv4AddressHelper ipv4;
		  NS_LOG_INFO ("Assign IP Addresses.");
		  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
		  Ipv4InterfaceContainer ipv4Cont = ipv4.Assign (devices);

		  MobilityHelper::EnableAsciiAll(ascii.CreateFileStream("mobility-trace-example.mob"));

		  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");


		// No application.....socket is created which sends data to a peer
		/*  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (1), tid);
		  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
		  recvSink->Bind (local);
		  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));  */

		 // Node 0 will be the RSU/Traffic light, we attach the socket beforehand, for the vehicles we attach sockets later



		//Pause();


		//Simulator::Schedule(Seconds(1), &Pause);
		 // Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
			//	            Seconds (30.1), &GenerateTraffic,
//				                 source, packetSize, numPackets)	;



		// std::ostringstream oss;

		//Config::Connect(oss2.str(), MakeCallback (&CourseChange));

		// create vector of vehicle node structures
		std::queue<veh_Nodes> vehicles;
		struct veh_Nodes veh;
//cout<< "before node loop"<<endl;
		for ( int i = 0 ; i < numVehicleNodes ; i++)
		{

			veh.veh_node = c.Get(i);
			veh.veh_number = 0;
			veh.in_use = 0;
			veh.veh_socket = 0;
			vehicles.push(veh);
		}
		Config::SetDefault ("ns3::ArpCache::PendingQueueSize", UintegerValue (10));
		std::map<int, double> numb_pos_pair;
		
		
		
		// Code for alternating between control channel and service channel
		Config::SetDefault ("ns3::ChannelCoordinator::CchInterval", TimeValue (MilliSeconds(50)));
		Config::SetDefault ("ns3::ChannelCoordinator::SchInterval", TimeValue (MilliSeconds(50)));
		/*for (uint32_t i = 0; i != devices.GetN (); ++i)

		{
			Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice>(devices.Get(i));
			// Alternating access without immediate channel switch
			const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
			Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, device, schInfo);
		}*/


			//cout<< "before listen starts"<<endl;
			// Now acceptor starts listening
		   if (acceptor->start() == 0)
		    {
			while (1)
			{
			    cout << "Start MATLAB \n" ;
			    stream = acceptor->accept();
			    //PlatoonGroup::streamer = stream;

			    if (stream != NULL)
			    {
				  // Schedule a pause event at the very beginning of simulations to wait for data from MATLAB
				//std::ostringstream oss1;
				
		 		//oss1 <<  "/NodeList/" << c.Get(1)->GetId() << "/$ns3::MobilityModel/CourseChange";
				//cout<<"Over here!!"<<endl;
				// Trace to check if node 1 is moving or not
				//Config::Connect(oss1.str(), MakeCallback (&CourseChange));
				//Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChange));
				Simulator::Schedule(Seconds(0.0), &Pause, stream, &vehicles, &numb_pos_pair);
				//cout <<stream<<endl;
				//Config::Connect ("/NodeList/0/DeviceList/0/$ns3::WifiNetDevice/Phy/State/Tx",MakeBoundCallback (&TxSent, stream));
				 AnimationInterface anim("Interface.xml");

				  Simulator::Stop(Seconds(600.0));
				  //cout << "Before Run" << endl;
				  Simulator::Run ();
				  Simulator::Destroy ();
				  delete stream;
				  return 0;



   	    		    } // End of if (stream != NULL)

		        } // End of while (1)


		    } //End of if (acceptor->start() == 0)


    perror("Could not start the server");
    exit(-1);
} // End of int main(int argc, char* argv[])

//}

