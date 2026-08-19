/****************************************************************/
/*   NAME: Douglas Lima                                         */
/*   FILE: TrafficSim_Info.cpp                                  */
/*   DATE: 2026                                                 */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "TrafficSim_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pTrafficSim simulates boat traffic around the USV. Every      ");
  blk("  contact is published twice: as a NODE_REPORT, so the helm and ");
  blk("  pContactMgrV20 can act on it, and as !AIVDM sentences sent by ");
  blk("  UDP to a Signal K server, so a chart plotter sees it as a     ");
  blk("  genuine AIS target rather than a bespoke overlay.             ");
  blk("                                                                ");
  blk("  Vessel names are derived from the MMSI (AIS<mmsi>) so a client ");
  blk("  holding only an MMSI can name a contact for BHV_Trail without ");
  blk("  any lookup table.                                             ");
  blk("                                                                ");
  blk("  The app is INACTIVE unless enable_simulation=true, so that it ");
  blk("  can never inject fake AIS traffic on a real vessel by accident.");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pTrafficSim file.moos [OPTIONS]                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pTrafficSim with the given process name.           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pTrafficSim.                   ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pTrafficSim Example MOOS Configuration                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pTrafficSim                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  enable_simulation = true    // required, off by default       ");
  blk("                                                                ");
  blk("  origin_lat = 29.71970895316288   // defaults to LatOrigin     ");
  blk("  origin_lon = -91.14705165281887  // defaults to LongOrigin    ");
  blk("                                                                ");
  blk("  aivdm_host = 127.0.0.1      // Signal K NMEA0183 UDP listener ");
  blk("  aivdm_port = 10112                                            ");
  blk("  mmsi_base  = 366999000      // clearly fake MMSI block        ");
  blk("                                                                ");
  blk("  position_interval = 1       // seconds between AIS message 1  ");
  blk("  static_interval   = 30      // seconds between AIS message 5  ");
  blk("                                                                ");
  blk("  // (a) scripted contacts - deterministic and repeatable       ");
  blk("  contact = name=TUG_ALPHA, type=tug, x=300, y=200, hdg=270, spd=4");
  blk("                                                                ");
  blk("  // (b) random traffic around the USV                          ");
  blk("  random_contacts = 6                                           ");
  blk("  random_radius   = 1500                                        ");
  blk("  random_spd_min  = 2                                           ");
  blk("  random_spd_max  = 9                                           ");
  blk("                                                                ");
  blk("  // (c) closed-loop patrols - predictable targets to follow    ");
  blk("  loop = name=WORKBOAT_ECHO, type=cargo, spd=5, pts=0,0:600,0:600,600:0,600");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pTrafficSim INTERFACE                                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X, NAV_Y   Ownship position, used to place and recycle    ");
  blk("                 the random traffic around the USV.             ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT    One per contact per iteration, in the standard ");
  blk("                 NAME/TYPE/TIME/X/Y/SPD/HDG/YAW/DEPTH/LENGTH/   ");
  blk("                 MODE form.                                     ");
  blk("                                                                ");
  blk("  Also emits !AIVDM sentences over UDP - these leave the        ");
  blk("  community entirely and do not appear in the MOOSDB.           ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pTrafficSim", "gpl");
  exit(0);
}
