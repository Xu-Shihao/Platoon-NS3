#include "PlatoonGroup.h"

//using namespace std;

namespace ns3
{

int PlatoonGroup::ack1=0;

PlatoonGroup::PlatoonGroup(int psize, std::queue<veh_Nodes>* vehicles, std::map<int, double>* mymap, ns3::Time first,int psize2, std::queue<veh_Nodes>* vehicles2, std::map<int, double>* mymap2, ns3::Time first2, TCPStream* stream) :  streamer(stream)  /// What the constructor has to do is to create the vehicles
{
    //constructor  has to pop out the elements from the vehicle and update the platoon_members member of the PlatoonGroup class. Everytime a new element is popped, a socket is attached,
    // vehicle number is updated and its poisition information is added to the node
	//cout << "insude constructor" << endl;
	//cout << size_of_platoon << endl;
	//out<< streamer << endl	;
	size_of_platoon1=psize;
	size_of_platoon2=psize2;
	size_of_platoon=size_of_platoon1+size_of_platoon2;
	cout << "size_of_platoon: " << endl;	
	cout << size_of_platoon1 << endl;
	cout << size_of_platoon2 << endl;
	//cout << size_of_platoon << endl;
	LeaderVehicle=0;
	std::map<int, double>::iterator it = mymap->begin();
	for (int i = 0; i < size_of_platoon1; i++)
	{
		//cout << "Inside node assignment" << endl;
		struct veh_Nodes platMember= vehicles->front();
		//vehicles->pop();
		platMember.veh_number = it->first;
		platMember.in_use = 1;  ///Continue from the dereferencing
		TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
		Ptr<Socket> recvSink = Socket::CreateSocket (platMember.veh_node, tid);
		platMember.veh_socket = recvSink;	
		InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
		recvSink -> SetAllowBroadcast (true);
    		recvSink -> Connect (remote);
		InetSocketAddress local = InetSocketAddress (platMember.veh_node->GetObject<ns3::Ipv4>()->GetAddress(1,0).GetLocal(), 80);
		recvSink -> Bind (local);
		Ptr<WaypointMobilityModel> vehWaypointMobility = DynamicCast<WaypointMobilityModel>(platMember.veh_node->GetObject<MobilityModel>());
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
	std::map<int, double>::iterator it2 = mymap2->begin();
	for (int i = 0; i < size_of_platoon2; i++)
	{
		//cout << "Inside node assignment" << endl;
		struct veh_Nodes platMember= vehicles2->front();
		//vehicles->pop();
		platMember.veh_number = it2->first;
		platMember.in_use = 1;  ///Continue from the dereferencing
		TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
		Ptr<Socket> recvSink = Socket::CreateSocket (platMember.veh_node, tid);
		platMember.veh_socket = recvSink;	
		InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
		recvSink -> SetAllowBroadcast (true);
    		recvSink -> Connect (remote);
		InetSocketAddress local = InetSocketAddress (platMember.veh_node->GetObject<ns3::Ipv4>()->GetAddress(1,0).GetLocal(), 80);
		recvSink -> Bind (local);
		Ptr<WaypointMobilityModel> vehWaypointMobility = DynamicCast<WaypointMobilityModel>(platMember.veh_node->GetObject<MobilityModel>());
		vehWaypointMobility->AddWaypoint(Waypoint(first2,Vector(it2->second,0.0,0.0)));
		//cout << "After mobility" << endl;
		// After the data is entered in the struct along with position of the node
		platoon_members.push_back(platMember);
	        vehicles2->pop();
		//cout << "Node added to vector" << endl;
		it2++;
		//cout << "Iterator incremented" << endl;
	}
	mymap2->erase (mymap2->begin(), mymap2->end() );
	// Connnecting the packet sent and receive callback trace sources to the appropriate sinks 
	for (int i=0; i<size_of_platoon; i++)
	{
		//cout << "Before packet sending sink attach" << endl;
		//cout << platoon_members[i].veh_node -> GetId()<<" IP: "<< platoon_members[i].veh_ip_address<< endl;
		std::ostringstream oss1;
		oss1 <<  "/NodeList/" << platoon_members[i].veh_node -> GetId() << "/DeviceList/0/$ns3::WifiNetDevice/Phy/State/Tx";
		Config::Connect (oss1.str(), MakeBoundCallback (&PlatoonGroup::TxSent, streamer, &platoon_members[i]));
		platoon_members[i].veh_socket -> SetRecvCallback (MakeBoundCallback (&PlatoonGroup::ReceivePacket,streamer,&platoon_members[i],&platoon_members));
	
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
void PlatoonGroup::SendPacket (Ptr<Socket> socket, uint32_t pktSize)
{ 
      socket->Send (Create<Packet> (pktSize));
      //cout <<"Complete Send merge packet" << endl;
}
int PlatoonGroup::GetSize()
{

    return size_of_platoon;

}
void PlatoonGroup::schedule_merge_event(double Sch, uint32_t packetSize,uint32_t numPack)
{
	
	//	
	//Ptr<WifiNetDevice>  sender1 = platoon_members[0].veh_device;
	//Ptr<WifiNetDevice>  receiver1 = platoon_members[7].veh_device;
	//platoon2 leader send socket to the leader of platoon1 
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> source = Socket::CreateSocket (platoon_members[7].veh_node, tid);
	InetSocketAddress remote = InetSocketAddress (platoon_members[0].veh_ip_address, 80);
	//Set socket options, it is also possible to set the options after the socket has been created/connected.
	  if (ipTos > 0)
	    {
	      source->SetIpTos (ipTos);
	    }

	  if (ipTtl > 0)
	    {
	      source->SetIpTtl (ipTtl);
	    }
	source->Connect (remote);	
	//source->Send (Create<Packet> (packetSize));
	Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (Sch), &SendPacket, 
                                  source, packetSize);
	//cout <<"Complete Send merge socket" << endl;
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
     if(p->GetSize() == 564)
	{
     char buf2[20];
     //std::cout<< "One packet sent:  "<<p->GetSize()<<std::endl;
     sprintf(buf2, "RSU %d:%.6f,", plat->veh_number, Simulator::Now ().GetSeconds ());
     //std::cout<< "Vehicle " << plat->veh_number << " sent packet at " <<  Simulator::Now ().GetSeconds () << endl;
     ss1->send(buf2, sizeof(buf2));
	}
}


void PlatoonGroup::ReceivePacket (TCPStream* ss2, struct veh_Nodes* v,std::vector<veh_Nodes>* platoon_members, Ptr<Socket> socket)
{

    Ptr<Packet> packet = socket->Recv ();
    //std::cout<< "packet size: "<< packet->GetSize()<< "at "<<Simulator::Now ().GetSeconds ()<<std::endl;

    struct veh_Nodes receiver=*v;
    if(packet->GetSize() == 500)
	{
	    std::cout<< " reveiver is: "<<v->veh_number<<" at " <<  Simulator::Now ().GetSeconds () <<" ack1 = "<<ack1<<std::endl;
            char buf[20];
            for (int i=0; i<20 ; i++)
            {
                buf[i] = 0;

            }
            buf[19] = '\0';
	    //cout << v->in_use << endl;
            sprintf(buf, "%d:%.6f,",v->veh_number, Simulator::Now ().GetSeconds ());
            //std::cout << "Vehicle " << v->veh_number<< " received packet at "<< Simulator::Now ().GetSeconds () << endl;
            ss2->send(buf,sizeof(buf));
	    
        }
    else if(packet->GetSize() == 120)
	{
	   std::cout << "get in callback, ack1 = "<<ack1 << endl;
	   // 1. Send merge_accept by leader
	   if (receiver.veh_number == (*platoon_members)[0].veh_number && ack1 == 0)
	   {
		struct veh_Nodes sender=(*platoon_members)[7];
		PlatoonGroup::SendAcknowledgement (receiver, sender); //reply 1
		std::cout<< "Merge Starts" << std::endl;
		TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
		Ptr<Socket> source = Socket::CreateSocket (receiver.veh_node, tid);
		InetSocketAddress remote = InetSocketAddress (sender.veh_ip_address, 80);
		source->Connect (remote);	
		//source->Send (Create<Packet> (120));
		Simulator::ScheduleWithContext (source->GetNode ()->GetId (),Seconds (1.0), &SendPacket,source, 120);
	  	//ack++;
	   }
	   if (receiver.veh_number == (*platoon_members)[7].veh_number && ack1==1)
	   {
		struct veh_Nodes sender=(*platoon_members)[0];
		PlatoonGroup::SendAcknowledgement (receiver, sender);
		ack1++;
		std::cout<< "Received merging permission"<< std::endl;
	   }
	   if (receiver.veh_number == (*platoon_members)[7].veh_number && ack1==0)
	   {
		ack1++;
		std::cout<< "Get in ack1="<< ack1 << std::endl;
		
	   }
	   
	   if (receiver.veh_number != (*platoon_members)[7].veh_number && receiver.veh_number != (*platoon_members)[0].veh_number)
	   {
		struct veh_Nodes sender=(*platoon_members)[7];	
		PlatoonGroup::SendAcknowledgement (receiver, sender);
		ack1++;
		std::cout << "when last vehicles change leader, the ack1 = "<< ack1 << std::endl; 
	   }


	    // Change_PL multicast to all the vehicles after vehicle 7
	    if (receiver.veh_number == (*platoon_members)[0].veh_number && ack1 == 2)
	   {
	      		std::cout<<"Sending vehicles after Vehicle 7" <<std::endl;
			struct veh_Nodes sender=(*platoon_members)[7];
	   		for (std::vector<veh_Nodes>::iterator it = (*platoon_members).begin() + 8 ; it != (*platoon_members).end(); it++)
	  		{
			     
	  		     TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
			     Ptr<Socket> source = Socket::CreateSocket (sender.veh_node, tid);
			     InetSocketAddress remote = InetSocketAddress (it->veh_ip_address, 80);
			     source->Connect (remote);	
			     Simulator::ScheduleWithContext (source->GetNode ()->GetId (),Seconds (0.1), &SendPacket,source, 120);
	  		}
	   }

	   if (ack1 == 4)
	   {
	   char buf[20];
            for (int i=0; i<20 ; i++)
            {
                buf[i] = 0;

            }
            buf[19] = '\0';
	    //cout << v->in_use << endl;
            sprintf(buf, "Merge:%.6f,",Simulator::Now ().GetSeconds ());
	    std::cout << "Merge send to MATLAB" << std::endl;
            ss2->send(buf, sizeof(buf));
	    ack1=5;
	   }
	}
}

void PlatoonGroup::SendAcknowledgement (struct veh_Nodes sender, struct veh_Nodes receiver)
{
	//Ptr<WifiNetDevice>  sender1 = sender.veh_device;
	//Ptr<WifiNetDevice>  receiver1 = receiver.veh_device;
	//platoon2 leader send socket to the leader of platoon1 
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> source = Socket::CreateSocket (sender.veh_node, tid);
	InetSocketAddress remote = InetSocketAddress (receiver.veh_ip_address, 80);
	//Set socket options, it is also possible to set the options after the socket has been created/connected.
	source->Connect (remote);	
	source->Send (Create<Packet> (120));
	
}

PlatoonGroup::~PlatoonGroup()
{
    //dtor
}

} //namespace ns3
