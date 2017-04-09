#ifndef PLATOONGROUP_H
#define PLATOONGROUP_H

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
#include "ns3/nstime.h"
#include "ns3/seq-ts-header.h"
//#include "TCPAcceptor.h"
//using namespace std;
struct veh_Nodes{

	ns3::Ptr<ns3::Node> veh_node;
    ns3::Address node_mac_address;
	//ns3::Ptr<ns3::Socket> veh_socket;
    ns3::Ptr<ns3::WaveNetDevice> veh_device;
	int veh_number;
	bool in_use;

};

namespace ns3
{

class PlatoonGroup
{
    public:
        //static void CourseChange1(std::string context, Ptr<const MobilityModel>model);
        PlatoonGroup(int psize, std::queue<veh_Nodes>* vehicles, std::map<int, double>* mymap, ns3::Time first, TCPStream* stream);
        int GetSize();
        //void RemoveVehicle();
        //void AddVehicle();
        void SetPosition_next(std::map<int, double>* mymap);
        void SetPosition_next_initial(std::map<int, double>* mymap, ns3::Time first);
        ///Static function since every platoon will use them to send data back to MATLAB
        void schedule_split_event(double Sch, uint32_t packetSize,uint32_t numPack);
        static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
		                     uint32_t pktCount);
        static void TxSent (TCPStream *str, struct veh_Nodes* plat, std::string context, Ptr<const Packet> p, WifiMode mode,
                            enum WifiPreamble preamble, uint8_t txLevel);
	static long double Uniform (double Schedule);
       // static void attach_vehicle_to_node(TCPStream *str, struct veh_Nodes* vehicle,
       //                                    int number,double position, bool init)
        static void SendAcknowledgement (const Address &sender, Ptr<WaveNetDevice> recv);
        //bool ReceivePacket (TCPStream *ss2,  Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
        bool ReceivePacket (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
        virtual ~PlatoonGroup();

    protected:

    private:
        std::vector<veh_Nodes> platoon_members;
        //std::<ns3::Mac48Address> node_mac_address;
        int size_of_platoon;
        int LeaderVehicle ;
	    TCPStream* streamer; 
        int ack ;
};

#endif // PLATOONGROUP_H

}

