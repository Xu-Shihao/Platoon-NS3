#include "PlatoonGroup.h"

//using namespace std;

namespace ns3
{

PlatoonGroup::PlatoonGroup(int psize, std::queue<veh_Nodes>* vehicles, std::map<int, double>* mymap, ns3::Time first, TCPStream* stream) : size_of_platoon(psize), streamer(stream)  /// What the constructor has to do is to create the vehicles
{
    //constructor  has to pop out the elements from the vehicle and update the platoon_members member of the PlatoonGroup class. Everytime a new element is popped, a socket is attached,
    // vehicle number is updated and its poisition information is added to the node
	//cout << "insude constructor" << endl;
	//cout << size_of_platoon << endl;
	//out<< streamer << endl	;
	std::map<int, double>::iterator it = mymap->begin();
	for (uint32_t i = 0; i < size_of_platoon; i++)
	{
		//cout << "Inside node assignment" << endl;
		struct veh_Nodes platMember= vehicles->front();
		//vehicles->pop();
		platMember.veh_number = it->first;
		platMember.in_use = 1;  ///Continue from the dereferencing

		TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

		// All sockets will both broadcast and recv.....therefore both "setallowbroadcast" and "setrecvcallback" have to be installed on the socket

		//Ptr<Socket> source = Socket::CreateSocket (c.Get (0), tid);
		//InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
		//source->SetAllowBroadcast (true);
		//source->Connect (remote);
		//cout << "B4 socket creation" << endl;
		
			//cout << "Inside socket creation" << endl;
			Ptr<Socket> recvSink = Socket::CreateSocket (platMember.veh_node, tid);
			platMember.veh_socket = recvSink;	
			InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
			recvSink -> SetAllowBroadcast (true);
            		recvSink -> Connect (remote);
			InetSocketAddress local = InetSocketAddress (platMember.veh_node->GetObject<ns3::Ipv4>()->GetAddress(1,0).GetLocal(), 80);
			recvSink -> Bind (local);
			//cout << platMember.veh_number << endl;
			//recvSink -> SetRecvCallback (MakeBoundCallback (&PlatoonGroup::ReceivePacket, streamer, &platMember));
			//Ptr<Node> tempNode = platMember.veh_node;
			//Ptr<NetDevice> tempDevice = platMember.veh_node->GetDevice(0);	
			//int bindSuccess = platMember.veh_socket->Bind (local);
			//platMember.veh_socket->SetRecvCallback (MakeBoundCallback (&PlatoonGroup::ReceivePacket, streamer, &platMember));
			//cout << "Socket Created" << endl;


		//Ptr<WaypointMobilityModel> vehWaypointMobility = DynamicCast<WaypointMobilityModel>( platMember->veh_node->GetObject<MobilityModel>());
		//cout << "B4 Mobility" << endl;
		Ptr<WaypointMobilityModel> vehWaypointMobility = DynamicCast<WaypointMobilityModel>(platMember.veh_node->GetObject<MobilityModel>());
		//cout << "Dynamic cast works" << endl;
		//cout << Vector(it->second,0.0,0.0) << endl;
		//cout << Seconds(first) << endl;
		//cout<< Simulator::Now ().GetSeconds () << endl;
		vehWaypointMobility->AddWaypoint(Waypoint(first,Vector(it->second,0.0,0.0)));
		//cout << "After mobility" << endl;
		// After the data is entered in the struct along with position of the node
		platoon_members.push_back(platMember);
	        vehicles->pop();
		//cout << "Node added to vector" << endl;
		it++;
		//cout << "Iterator incremented" << endl;
	}
    	mymap->erase (mymap->begin(), mymap->end() );
	//std::ostringstream oss1;
	
	// Connnecting the packet sent and receive callback trace sources to the appropriate sinks 
	for (uint32_t i=0; i<size_of_platoon; i++)
	{
		//cout << "Before packet sending sink attach" << endl;
		//cout << platoon_members[i].veh_node -> GetId() << endl;
		std::ostringstream oss1;
		oss1 <<  "/NodeList/" << platoon_members[i].veh_node -> GetId() << "/DeviceList/0/$ns3::WifiNetDevice/Phy/State/Tx";
		Config::Connect (oss1.str(), MakeBoundCallback (&PlatoonGroup::TxSent, streamer, &platoon_members[i]));
		platoon_members[i].veh_socket -> SetRecvCallback (MakeBoundCallback (&PlatoonGroup::ReceivePacket, streamer, &platoon_members[i]));
		
	}
}

long double PlatoonGroup::Uniform(double Schedule)
{
	double min = Schedule+0.000000001;
	double max = Schedule+0.08;
	Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
	x->SetAttribute ("Min", DoubleValue (min));
	x->SetAttribute ("Max", DoubleValue (max));
	// The values returned by a uniformly distributed random
	// variable should always be within the range
	//
	// [min, max) .
	//
	long double value = x->GetValue ();
	return value;

}

int PlatoonGroup::GetSize()
{

    return size_of_platoon;

}

void PlatoonGroup::schedule_first_event(double Sch, uint32_t packetSize,uint32_t numPack)
{
    ///Schedule a broadcast by every vehicle at span of 50 milliseconds
	for (uint32_t i=0; i< size_of_platoon; i++)
	{
		//cout<< "Packet send schedule for " << platoon_members[i].veh_node->GetId ()<< endl;	
		
		//cout << platoon_members[i].veh_socket << endl;

/*		Simulator::ScheduleWithContext (platoon_members[i].veh_node->GetId (),
				            	Seconds(Sch + 0.02*(i+1)), &PlatoonGroup::GenerateTraffic,
				                platoon_members[i].veh_socket, packetSize, numPack);  
		Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice>(devices.Get(platoon_members[i].veh_node->GetId ()));
			// Alternating access without immediate channel switch
			const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
			Simulator::Schedule (Seconds (sch), &WaveNetDevice::StartSch, device, schInfo);*/

		Simulator::ScheduleWithContext (platoon_members[i].veh_node->GetId (),
				            	Seconds(PlatoonGroup::Uniform(Sch)), &PlatoonGroup::GenerateTraffic,
				                platoon_members[i].veh_socket, packetSize, numPack);  

	}

}

void PlatoonGroup::SetPosition_next(std::map<int, double>* mymap)
{
	//cout << this->size_of_platoon << endl;	
	//cout << "Inside set pos next" << endl;
	//cout << streamer << endl;
	std::map<int, double>::iterator iter;
	//cout << size_of_platoon << endl;
	for (uint32_t i=0; i<size_of_platoon; i++)
	{
		//cout << "before 2nd for loop" << endl;
		for ( iter = mymap->begin(); iter!= mymap->end(); iter++)
		{
			//cout<< iter->first << endl;
			if(platoon_members[i].veh_number == iter->first)
			{
				//cout <<"In SetPosition_next" << endl;	
				//cout << "B4 waypoint" << endl;
				
				Ptr<WaypointMobilityModel> vehWaypointMobility = DynamicCast<WaypointMobilityModel>(platoon_members[i].veh_node->GetObject<MobilityModel>());
				vehWaypointMobility->AddWaypoint(Waypoint(Seconds(Simulator::Now ().GetSeconds () + 0.1), Vector((*mymap)[platoon_members[i].veh_number],0.0,0.0)));				
				vehWaypointMobility->Update ();
				mymap->erase(iter);
				break;
			}
		}
	}
}

void PlatoonGroup::SetPosition_next_initial(std::map<int, double>* mymap, ns3::Time first)
{
	std::map<int, double>::iterator iter;
	for (uint32_t i=0; i<size_of_platoon; i++)
	{
		for ( iter = mymap->begin(); iter != mymap->end(); iter++)
		{

			if(platoon_members[i].veh_number == iter->first)
			{
				//cout <<"In SetPosition_next_initial" << endl;				
				Ptr<WaypointMobilityModel> vehWaypointMobility = DynamicCast<WaypointMobilityModel>(platoon_members[i].veh_node->GetObject<MobilityModel>());

				//vehWaypointMobility->AddWaypoint(Waypoint(first + Seconds(1), Vector((*mymap)[platoon_members[i].veh_number],0.0,0.0)));
				// Waypoints will be supplied by MATLAB every 0.1 seconds
				vehWaypointMobility->AddWaypoint(Waypoint(first + Seconds(0.1), Vector(iter -> second,0.0,0.0)));

				///One of the following has to work
				//Simulator::Schedule (Seconds(first), &WaypointMobilityModel::Update, this);
				Simulator::Schedule (first, &WaypointMobilityModel::Update, vehWaypointMobility);

				mymap->erase(iter);
				break;
			}
		}
	}
}

void PlatoonGroup::GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
		                     uint32_t pktCount)

{

        if (pktCount > 0)

            {

             socket->Send (Create<Packet> (pktSize));
             Simulator::Schedule (Seconds(0.1), &PlatoonGroup::GenerateTraffic, socket, pktSize,pktCount);

            }
        else
            {
              socket->Close ();
            }

}

void PlatoonGroup::TxSent (TCPStream *ss1, struct veh_Nodes* plat, std::string context, Ptr<const Packet> p, WifiMode mode,
                           enum WifiPreamble preamble, uint8_t txLevel)
{

     char buf2[20];
   //ssize_t foo;
     sprintf(buf2, "RSU %d:%.6f,", plat->veh_number, Simulator::Now ().GetSeconds ());
     //std::cout<< "Vehicle " << plat->veh_number << " sent packet at " <<  Simulator::Now ().GetSeconds () << endl;
     ss1->send(buf2, sizeof(buf2));

}

void PlatoonGroup::ReceivePacket (TCPStream *ss2, struct veh_Nodes* v, Ptr<Socket> socket)
{
    Ptr<Packet> packet = socket->Recv ();
    //std::cout<< "packet size: "<< packet->GetSize()<<std::endl;
    if(packet->GetSize() == 500)
	{
            char buf[20];
            for (int i=0; i<20 ; i++)
            {
                buf[i] = 0;

            }
            buf[19] = '\0';
	    //cout << v->in_use << endl;
            sprintf(buf, "%d:%.6f,",v->veh_number, Simulator::Now ().GetSeconds ());
            std::cout << "Vehicle " << v->veh_number<< " received packet at "<< Simulator::Now ().GetSeconds () << endl;
            ss2->send(buf, sizeof(buf));
        }
}


PlatoonGroup::~PlatoonGroup()
{
    //dtor
}

} //namespace ns3
