#include "PlatoonGroup.h"
//#include "ns3/waypoint-mobility-model.h"

//using namespace std;

namespace ns3
{

PlatoonGroup::PlatoonGroup(int psize, std::queue<veh_Nodes>* vehicles, std::map<int, double>* mymap, ns3::Time first, TCPStream* stream) : size_of_platoon(psize), streamer(stream), ack (0)  /// What the constructor has to do is to create the vehicles
{
    //constructor  has to pop out the elements from the vehicle and update the platoon_members member of the PlatoonGroup class. Everytime a new element is popped, a socket is attached,
    // vehicle number is updated and its poisition information is added to the node
	//cout << "insude constructor" << endl;
	//cout << size_of_platoon << endl;
	//out<< streamer << endl	;
	std::map<int, double>::iterator it = mymap->begin();
	for (int i = 0; i < size_of_platoon; i++)
	{
		// Pulling out the first vehicle form the qof vehicle nodes
		struct veh_Nodes platMember= vehicles->front();
		platMember.veh_number = it->first;
		platMember.in_use = 1;  ///Continue from the dereferencing
		Ptr<WaypointMobilityModel> vehWaypointMobility = DynamicCast<WaypointMobilityModel>(platMember.veh_node->GetObject<MobilityModel>());
		vehWaypointMobility->AddWaypoint(Waypoint(first,Vector(it->second,0.0,0.0)));
		platoon_members.push_back(platMember);
	        vehicles->pop();
		it++;
		
	}
    	mymap->erase (mymap->begin(), mymap->end() );
	
	// Connnecting the packet sent and receive callback trace sources to the appropriate sinks 
	for (int i=0; i<size_of_platoon; i++)
	{
	
		std::ostringstream oss1;
		oss1 <<  "/NodeList/" << platoon_members[i].veh_node -> GetId() << "/DeviceList/0/$ns3::WaveNetDevice/Phy/State/Tx";
		Config::Connect (oss1.str(), MakeBoundCallback (&PlatoonGroup::TxSent, streamer, &platoon_members[i]));
		//platoon_members[i].veh_device -> SetReceiveCallback (MakeBoundCallback (&PlatoonGroup::ReceivePacket, streamer));
    		platoon_members[i].veh_device -> SetReceiveCallback (MakeCallback  (&PlatoonGroup::ReceivePacket, this));	
	}
}


/*void PlatoonGroup::CourseChange1(std::string context, Ptr<const MobilityModel>model)
{
  Vector position = model->GetPosition();
  NS_LOG_UNCOND (context << "  x= "  << position.x << ", y= " <<position.y);

}*/


long double PlatoonGroup::Uniform(double Schedule)
{
	double min = Schedule+0.000000001;
	double max = Schedule+0.08;
	Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
	x->SetAttribute ("Min", DoubleValue (min));
	x->SetAttribute ("Max", DoubleValue (max));
	long double value = x->GetValue ();
	return value;

}

int PlatoonGroup::GetSize()
{

    return size_of_platoon;

}

void PlatoonGroup::schedule_split_event(double Sch, uint32_t packetSize,uint32_t numPack)
{
	
    ///Schedule a broadcast by every vehicle at span of 50 milliseconds
	Ptr<WaveNetDevice>  sender = platoon_members[0].veh_device;
  	Ptr<WaveNetDevice>  receiver = platoon_members[6].veh_device;
  	const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
  //Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
     const Address sendAdd = sender->GetAddress ();
    const Address dest = receiver->GetAddress ();
    std::cout << "Sender: " << sendAdd << "and" << "Receiver: " << dest << std::endl;
    const TxInfo txInfo = TxInfo (CCH, 7, WifiMode("OfdmRate6MbpsBW10MHz"),  1);
  Ptr<Packet> wsaPacket  = Create<Packet> (packetSize);
  //SeqTsHeader seqTs;
  //seqTs.SetSeq (seq);
  //wsaPacket->AddHeader (seqTs);
  //sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
  Simulator::Schedule (Seconds (Sch),  &WaveNetDevice::SendX, sender, wsaPacket, dest, WSMP_PROT_NUMBER, txInfo);

}

void PlatoonGroup::SetPosition_next(std::map<int, double>* mymap)
{
	//cout << this->size_of_platoon << endl;	
	//cout << "Inside set pos next" << endl;
	//cout << streamer << endl;
	cout << "SetPosition_next time: " <<Simulator::Now ().GetSeconds ()<< " a="<< ack <<endl;
	std::map<int, double>::iterator iter;
	//cout << size_of_platoon << endl;
	for (int i=0; i<size_of_platoon; i++)
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
	for (int i=0; i<size_of_platoon; i++)
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
     std::cout<< "Vehicle " << plat->veh_number << " sent packet at " <<  Simulator::Now ().GetSeconds () << endl;
     ss1->send(buf2, sizeof(buf2));

}

// ReceivePacket earlier was simply checking for packet reception on the socket and printing and sending 
// this "data received" information back to MATLAB

// Now the packet contents will have to be read, especially the MAC address and the packet contents 
// ( Split_req and ACK ) and based on that an action will be required to be taken. For split there has to 
// be a differnet set of schedule that will have to be taken 

// This receive packet will be responsible for a few things....the first one is to send the ACK as soon as it is received


// The following function has been connected to the trace source....now the only thing to do is to write the logic

//bool PlatoonGroup::ReceivePacket (TCPStream *ss2, Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
bool PlatoonGroup::ReceivePacket (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
     SeqTsHeader seqTs;
     pkt->PeekHeader (seqTs);
     Ptr<WaveNetDevice> receiver = DynamicCast<WaveNetDevice> (dev);
     const Address recvAdd = receiver->GetAddress();
     std::cout << "receive a packet from " << sender <<" to "<< recvAdd<< std::endl
            << "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s," << std::endl
            << "  recvTime = " << Now ().GetSeconds () << "s," << std::endl ;
    //std::cout << "Sender: " << sender << " and " << "Receiver: " << recvAdd  <<std::endl;
    // First duty is to send an ACK to the leader/followera as soon as the packet is received by any of the node
    // Function to send packet back to 
   if (recvAdd != platoon_members[0].node_mac_address)
   {
	PlatoonGroup::SendAcknowledgement (sender, receiver);   		
	//ack = ack + 1;
      //std::cout << ack << std::endl; 
   }

   if (receiver == platoon_members[0].veh_device && pkt->GetSize () == 80)
   {

      ack=ack+1;
      //std::cout << ack << std::endl; 
   }

   // Send split_accept by platoon number 6
   if (receiver == platoon_members[6].veh_device && ack == 0)
   {
      std::cout<< "Hello" << std::endl;
      Ptr<WaveNetDevice>  sender1 = platoon_members[6].veh_device;
      Ptr<WaveNetDevice>  receiver1 = platoon_members[0].veh_device;
      const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
      //Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
      const Address dest = receiver1->GetAddress ();
      const TxInfo txInfo = TxInfo (CCH, 7, WifiMode("OfdmRate6MbpsBW10MHz"),  1);
      Ptr<Packet> wsaPacket  = Create<Packet> (100);
  		  
      // Schedule a send event 1 seconds from now where the message conveyed is split_accept
    Simulator::Schedule (Seconds (1.0),  &WaveNetDevice::SendX, sender1, wsaPacket, dest, WSMP_PROT_NUMBER, txInfo);
  		  return true;
   }

   // platoon leader Sends Change_pl to platoon 6
   if (receiver == platoon_members[0].veh_device && ack == 1 && pkt->GetSize () != 80)
   {

	   Ptr<WaveNetDevice>  sender2 = platoon_members[0].veh_device;
	   Ptr<WaveNetDevice>  receiver2 = platoon_members[6].veh_device;
	   const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
	  //Mac48Address bssWildcard = Mac48Address::GetBroadcast ();
	   const Address dest = receiver2->GetAddress ();
	   const TxInfo txInfo = TxInfo (CCH, 7, WifiMode("OfdmRate6MbpsBW10MHz"),  1);
	   Ptr<Packet> wsaPacket  = Create<Packet> (100);
	  		
	   Simulator::Schedule (Seconds (0.5),  &WaveNetDevice::SendX, sender2, wsaPacket, dest, WSMP_PROT_NUMBER, txInfo);
	   std::cout<< "Due to ack = 1" << std::endl;
	   ack++;
	   //std::cout << ack << std::endl; 
	   return true;	  
   }

    // Change_PL multicast to all the vehicles after vehicle 6
    if (receiver == platoon_members[0].veh_device && ack == 3)
   {
      		std::cout<<"Inside ack=3" <<std::endl;
   		Ptr<WaveNetDevice>  sender3 = platoon_members[0].veh_device;
   		const static uint16_t WSMP_PROT_NUMBER = 0x88DC;

   		for (std::vector<veh_Nodes>::iterator it = platoon_members.begin() + 7 ; it != platoon_members.end(); it++)
  		{
  		  Ptr<WaveNetDevice>  receiver3 = it->veh_device;
    		  const Address dest = receiver3->GetAddress ();
    		  const TxInfo txInfo = TxInfo (CCH, 7, WifiMode("OfdmRate6MbpsBW10MHz"),  1);
  		  Ptr<Packet> wsaPacket  = Create<Packet> (100);
  		
  		  Simulator::Schedule (Seconds (0.1),  &WaveNetDevice::SendX, sender3, wsaPacket, dest, WSMP_PROT_NUMBER, txInfo);

  		}
  		return true;
   }

   if (ack == 6)
   {

            char buf[20];
            for (int i=0; i<20 ; i++)
            {
                buf[i] = 0;

            }
            buf[19] = '\0';
	    //cout << v->in_use << endl;
            sprintf(buf, "%.6f", Simulator::Now ().GetSeconds () - 178);
            //std::cout << "Vehicle " << v->veh_number<< " received packet at "<< Simulator::Now ().GetSeconds () << endl;
            streamer->send(buf, sizeof(buf));

   }
   return true;

}

// Method for sending an acknowledgement by any of the follower members
void PlatoonGroup::SendAcknowledgement (const Address &sender, Ptr<WaveNetDevice> recv )
{

  //Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  //Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
  
   const TxInfo txInfo = TxInfo (CCH, 7, WifiMode("OfdmRate6MbpsBW10MHz"),  1);
  Ptr<Packet> wsaPacket  = Create<Packet> (80);
  //SeqTsHeader seqTs;
  //seqTs.SetSeq (seq);
  //wsaPacket->AddHeader (seqTs);
  ////sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
  //Simulator::Schedule (Seconds (Sch),  &WaveNetDevice::SendX, sender, wsaPacket, dest, WSMP_PROT_NUMBER, txInfo);
  std::cout << "Inside SendAcknowledgement" << std::endl;
  std::cout << "Sender: " << recv->GetAddress () << "and" << "Receiver: " << sender << std::endl;
  recv->SendX  (wsaPacket, sender, WSMP_PROT_NUMBER, txInfo);

}


PlatoonGroup::~PlatoonGroup()
{
    //dtor
}

} //namespace ns3
