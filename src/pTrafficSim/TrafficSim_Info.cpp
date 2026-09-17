/****************************************************************/
/*   NAME: DL                                                   */
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
  blk("  spawn_mmsi_base = 366999900 // separate block for TRAFFIC_SPAWN ");
  blk("  max_runtime_contacts = 8    // TRAFFIC_SPAWN targets at once     ");
  blk("  spawn_max_range = 10000     // metres; further away is refused   ");
  blk("  aim_lock_range  = 400       // an at_ownship target corrects its ");
  blk("                              // intercept until this range, then  ");
  blk("                              // holds course for the encounter    ");
  blk("  expunge_var = XYZ_EXPUNGE_TARGET // pContactMgrV20 expunge_var  ");
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
  blk("                                                                ");
  blk("  // (d) contacts placed at runtime arrive on TRAFFIC_SPAWN -    ");
  blk("  //     nothing to configure here, see the INTERFACE section.   ");
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
  blk("  NAV_HEADING, NAV_SPEED                                        ");
  blk("                 Ownship course and speed, so hdg=at_ownship    ");
  blk("                 can solve for an intercept rather than aim at  ");
  blk("                 where the boat already was.                    ");
  blk("                                                                ");
  blk("  TRAFFIC_SPAWN  Place one contact at runtime. Same grammar as  ");
  blk("                 the config 'contact' line, plus a heading that ");
  blk("                 can be aimed to intercept the USV:            ");
  blk("                   x=-6000,y=800,hdg=at_ownship,spd=2.5,        ");
  blk("                   type=cargo,length=25,beam=9                  ");
  blk("                 hdg=at_ownship solves for the course that     ");
  blk("                 meets the boat given its present course and    ");
  blk("                 speed, falling back to pure pursuit when the   ");
  blk("                 contact is too slow to catch it.               ");
  blk("                 The heading is resolved once, at spawn: the    ");
  blk("                 contact then holds course and speed, which is  ");
  blk("                 the steady-contact assumption every COLREGS    ");
  blk("                 behaviour is built around. Ignored             ");
  blk("                 unless enable_simulation is true.              ");
  blk("                                                                ");
  blk("  TRAFFIC_CLEAR  all | name=AIS366999901 | mmsi=366999901       ");
  blk("                 Removes runtime-spawned contacts only; the     ");
  blk("                 mission file's own contacts are never touched. ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT    One per contact per iteration, in the standard ");
  blk("                 NAME/TYPE/TIME/X/Y/SPD/HDG/YAW/DEPTH/LENGTH/   ");
  blk("                 MODE form.                                     ");
  blk("                                                                ");
  blk("  TRAFFIC_SPAWNED  Ack for TRAFFIC_SPAWN, carrying the assigned ");
  blk("                 identity and the position in both frames:      ");
  blk("                   name=,mmsi=,x=,y=,lat=,lon=,hdg=,spd=,type=, ");
  blk("                   utc=                                         ");
  blk("                                                                ");
  blk("  <expunge_var>  On TRAFFIC_CLEAR, the cleared contact's name,  ");
  blk("                 so pContactMgrV20 tells the helm to end the    ");
  blk("                 behaviours running against it straight away.   ");
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
