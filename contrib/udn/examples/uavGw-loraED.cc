/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2022 Universidade Federal de Goiás - UFG
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *   Author Rogério Sousa rogeriosousa@discente.ufg.br
 */

/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices.  The metric of interest for this script is the throughput of the
 * network.
*/

#include "ns3/end-device-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/netanim-module.h"
#include <fstream>
#include <iostream>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Network settings
int nDevices = 100;
int nGateways = 1;
double simulationTime = 600;
bool verbose = false;

/* Area definition
** The longest distance between a node and the gateway may be >=7.5Km,
** because it is the maximum distance the GW and Node can communicate using SF 12.
** In case of Rectangle, use the "areaRectSide", in case of Disc, use "radius"
*/
double areaRectSide = 7500;
double radius = 7500;

// Channel model
bool realisticChannelModel = false;

int appPeriodSeconds = 600;

int main(int argc, char *argv[]) {

    CommandLine cmd;
    cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
    cmd.AddValue("simulationTime", "The time for which to simulate", simulationTime);
    cmd.AddValue("appPeriod",
                 "The period in seconds to be used by periodically transmitting applications",
                 appPeriodSeconds);
    cmd.AddValue("print", "Whether or not to print various informations", verbose);
    cmd.Parse(argc, argv);

    // Set up logging
    if (verbose) {
        LogComponentEnable("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);
        LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
        LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
        LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
        LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
        LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
        LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
        LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
        LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
        LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
        LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
        LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
        LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
        LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
        LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
        LogComponentEnable("NetworkController", LOG_LEVEL_ALL);
    }
    /***********
     *  Setup  *
     ***********/

    // Create the time value from the period
    Time appPeriod = Seconds(appPeriodSeconds);

    // Mobility
    MobilityHelper mobilityNodes;
    ObjectFactory pos;

//    // Node placement area :: Rectangle Area [areaRectSide x areaRectSide]
//    // Mobility :: none :: ConstantPositionMobilityModel
//    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
//    std::stringstream ssX, ssY;
//    ssX << "ns3::UniformRandomVariable[Min=0.0|Max=" << areaRectSide << "]";
//    ssY << "ns3::UniformRandomVariable[Min=0.0|Max=" << areaRectSide << "]";
//    pos.Set("X", StringValue(ssX.str()));
//    pos.Set("Y", StringValue(ssY.str()));
//    Ptr <PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();
//
//    mobilityNodes.SetPositionAllocator(taPositionAlloc);
//    mobilityNodes.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // Node placement area :: Circular Area [areaRectSide x areaRectSide]
    // Mobility :: none :: ConstantPositionMobilityModel
    mobilityNodes.SetPositionAllocator("ns3::UniformDiscPositionAllocator", "rho", DoubleValue(radius),
                                       "X", DoubleValue(areaRectSide), "Y", DoubleValue(areaRectSide));
    /*
     ** Modelo de mobilidade
     **** Nodes Lorawan :: RandomWalk // ConstantPositionMobilityModel
     **** Drones :: ConstantPositionMobilityModel
     **** gNBs :: ConstantPositionMobilityModel
     **/
    mobilityNodes.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//    mobilityNodes.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
//                                   "Bounds",StringValue("0|0|15000|15000"));

    /************************
     *  Create the channel  *
     ************************/

    // Create the lora channel object
    Ptr <LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    if (realisticChannelModel) {
        // Create the correlated shadowing component
        Ptr <CorrelatedShadowingPropagationLossModel> shadowing =
                CreateObject<CorrelatedShadowingPropagationLossModel>();

        // Aggregate shadowing to the logdistance loss
        loss->SetNext(shadowing);
    }

    Ptr <PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();

    Ptr <LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    /************************
     *  Create the helpers  *
     ************************/

    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper();

    // Create the LoraHelper
    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking(); // Output filename
//    helper.EnableSimulationTimePrinting (appPeriod);

    //Create the NetworkServerHelper
    NetworkServerHelper nsHelper = NetworkServerHelper();

    //Create the ForwarderHelper
    ForwarderHelper forHelper = ForwarderHelper();

    /************************
     *  Create End Devices  *
     ************************/

    // Create a set of nodes
    NodeContainer endDevices;
    endDevices.Create(nDevices);

    // Assign a mobility model to each node
    mobilityNodes.Install(endDevices);

    // Make it so that nodes are at a certain height > 0
    Ptr <UniformRandomVariable> z = CreateObject<UniformRandomVariable>();
    z->SetAttribute("Min", DoubleValue(0));
    z->SetAttribute("Max", DoubleValue(1.2));
    for (NodeContainer::Iterator j = endDevices.Begin(); j != endDevices.End(); ++j) {
        Ptr <MobilityModel> mobilityNode = (*j)->GetObject<MobilityModel>();
        Vector position = mobilityNode->GetPosition();
        position.z = z->GetValue();
//        std::cout << "x: " << position.x << "y: " << position.y << "z: " << position.z << std::endl;
        mobilityNode->SetPosition(position);
    }

    // Create the LoraNetDevices of the end devices
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr <LoraDeviceAddressGenerator> addrGen =
            CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    // Create the LoraNetDevices of the end devices
    macHelper.SetAddressGenerator(addrGen);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    helper.Install(phyHelper, macHelper, endDevices);

    // Now end devices are connected to the channel

    // Connect trace sources
    for (NodeContainer::Iterator j = endDevices.Begin(); j != endDevices.End(); ++j) {
        Ptr <Node> node = *j;
        Ptr <LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr <LoraPhy> phy = loraNetDevice->GetPhy();
        //std::cout << "Phy: " << phy << std::endl;
    }

    // Set message type (Default is unconfirmed)
    //    Devices CLASSE A
    Ptr<LorawanMac> edMac1 = endDevices.Get (1)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ();
    Ptr<ClassAEndDeviceLorawanMac> edLorawanMac1 = edMac1->GetObject<ClassAEndDeviceLorawanMac> ();
    edLorawanMac1->SetMType (LorawanMacHeader::CONFIRMED_DATA_UP);


    /*********************
     *  Create Gateways  *
     *********************/

    // Create the gateway nodes
    NodeContainer gateways;
    gateways.Create(nGateways);
    MobilityHelper mobilityGateways;

    Ptr <ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    // Make it so that nodes are at a certain height > 0
    allocator->Add(Vector(7500.0, 7500.0, 25.0));  // adding a centralized gateway
    mobilityGateways.SetPositionAllocator(allocator);
    mobilityGateways.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityGateways.Install(gateways);

    // Create a netdevice for each gateway
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    // Print the nodes positions
    if (verbose) {
        std::ofstream myfile;
        myfile.open("./sim-res/nodes#100.txt");
        for (NodeContainer::Iterator itGW = gateways.Begin(); itGW != gateways.End(); ++itGW) {
            Ptr <MobilityModel> mobilityGW = (*itGW)->GetObject<MobilityModel>();
            for (NodeContainer::Iterator itN = endDevices.Begin(); itN != endDevices.End(); ++itN) {
                Ptr <MobilityModel> mobilityNode = (*itN)->GetObject<MobilityModel>();
                Vector position = mobilityNode->GetPosition();
                double distToGW = mobilityNode->GetDistanceFrom(mobilityGW);
                myfile << "Node" << (*itN)->GetId() << "; Distance " << distToGW << "Pos " << position.x
                       << "," << position.y << "," << position.z << std::endl;
                std::cout << (*itN)->GetId() << std::endl;
            }
        }
        myfile.close();
    }

    /**********************************************
     *  Set up the end device's spreading factor  *
     **********************************************/

    macHelper.SetSpreadingFactorsUp(endDevices, gateways, channel);

    NS_LOG_DEBUG("Completed configuration");

    /*********************************************
     *  Install applications on the end devices  *
     *********************************************/

    Time appStopTime = Seconds(simulationTime);
    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod(Seconds(appPeriodSeconds));
    appHelper.SetPacketSize(23);
//    Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
//            "Min", DoubleValue (0), "Max", DoubleValue (10));
    ApplicationContainer appContainer = appHelper.Install(endDevices);

    appContainer.Start(Seconds(0));
    appContainer.Stop(appStopTime);


    /**************************
     *  Create Network Server  *
     ***************************/

    // Create the NS node
    NodeContainer networkServer;
    networkServer.Create(1);

    // Create a NS for the network
    nsHelper.SetEndDevices(endDevices);
    nsHelper.SetGateways(gateways);
    nsHelper.Install(networkServer);

    //Create a forwarder for each gateway
    forHelper.Install(gateways);

    ////////////////
    // Simulation //
    ////////////////

    Simulator::Stop(appStopTime + Hours(1));

    NS_LOG_INFO("Running simulation...");
    AnimationInterface anim("./sim-res/animationGlobecom.xml");
//    uint32_t loraImgResId = anim.AddResource("../img/LoRa.png");
//    uint32_t uavImgResId = anim.AddResource("../img/uav.png");
    anim.EnablePacketMetadata(true);
    std::cout << "N. de Gateways: " << gateways.GetN() << std::endl << "N. de Devices: " << endDevices.GetN() << std::endl;
    for (uint32_t i = 0; i < gateways.GetN(); i++) {
        anim.UpdateNodeDescription(gateways.Get(i), "GW");
        anim.UpdateNodeColor(gateways.Get(i), 0, 255, 0);
//        anim.UpdateNodeImage(i, uavImgResId);
    }
    for (uint32_t i = 0; i < endDevices.GetN(); i++) {
//        anim.UpdateNodeImage(i, loraImgResId);
        anim.UpdateNodeDescription(endDevices.Get(i), "LoRa");
        anim.UpdateNodeColor(endDevices.Get(i), 255, 0, 0);
    }

    Simulator::Run();
    Simulator::Destroy();

    ///////////////////////////
    // Print results to file //
    ///////////////////////////
    NS_LOG_INFO ("Computing performance metrics...");

    LoraPacketTracker &tracker = helper.GetPacketTracker ();
    std::cout << "MAC Packets: " << tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (1)) << std::endl;

    return 0;
}