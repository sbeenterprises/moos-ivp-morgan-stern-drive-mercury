/****************************************************************/
/*   NAME: D Lima                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: WebSocket_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "WebSocket_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iWebSocket application is the link between the MOOS-IvP  ");
  blk("  community on the boat and the Swiftships-UI ashore: commands  ");
  blk("  from the UI become MOOS postings, telemetry goes back as JSON. ");
  blk("  If nothing is heard from any UI for comms_loss_timeout        ");
  blk("  seconds, the boat returns to the base (RETURN in pHelmIvP)    ");
  blk("  and holds there (RETURN_HOLD) - whenever the helm is in DRIVE ");
  blk("  or the boat is underway; with the helm in PARK and the boat   ");
  blk("  idle only if comms_loss_always is set.                        ");
  blk("  Every contact the helm is steering around (an avdcol_ instance ");
  blk("  in IVPHELM_BHV_ACTIVE) is sent to the UI's Alert List as      ");
  blk("  COLAV_ALERTS, with its heading, speed, range, bearing, CPA and ");
  blk("  TCPA.                                                         ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iWebSocket file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iWebSocket with the given process name         ");
  blk("      rather than iWebSocket.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iWebSocket.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iWebSocket Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iWebSocket                              ");
  blk("{                                                               ");
  blk("  AppTick   = 20                                                ");
  blk("  CommsTick = 20                                                ");
  blk("                                                                ");
  blk("  port = 8000                  // websocket port of the UI link ");
  blk("                                                                ");
  blk("  comms_loss_return         = true  // return to base on a loss ");
  blk("  comms_loss_timeout        = 30    // s without a UI frame     ");
  blk("  comms_loss_speed          = 5     // return speed (knots)     ");
  blk("  comms_loss_always         = false // also in PARK when idle   ");
  blk("  comms_loss_underway_speed = 1.0   // NAV_SPEED, adrift counts ");
  blk("                                                                ");
  blk("  base_lat  = 29.72282          // default base                 ");
  blk("  base_lon  = -91.21539                                         ");
  blk("  base_file = base_position.txt // keeps a base set from the UI ");
  blk("                                                                ");
  blk("  nav_speed_units  = knots  // NAV_SPEED: knots (GPS) or mps    ");
  blk("  colav_alert_hold = 15     // s an alert stays once avoided    ");
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
  blu("iWebSocket INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS (comms-loss failsafe):                            ");
  blk("------------------------------------                            ");
  blk("  BASE_COORDS = 29.722820,-91.215390 (lat,lon; also from the UI)");
  blk("  RETURN, RETURN_HOLD, WAYPOINT, STATIONING, TRAILING,          ");
  blk("  DESIRED_THRUST, NAV_SPEED                                     ");
  blk("                                                                ");
  blk("PUBLICATIONS (comms-loss failsafe):                             ");
  blk("------------------------------------                            ");
  blk("  COMMS_LOSS         = true/false                               ");
  blk("  RETURN             = true/false                               ");
  blk("  RETURN_HOLD        = false (true is the return's endflag)     ");
  blk("  RETURN_UPDATE      = point=x,y # speed=5.00                   ");
  blk("  RETURN_HOLD_UPDATE = station_pt=x,y                           ");
  blk("  BASE_COORDS        = lat,lon                                  ");
  blk("  VIEW_MARKER        = type=diamond,x=..,y=..,label=comms_loss_base");
  blk("                       (the base, drawn in pMarineViewer)         ");
  blk("                                                                ");
  blk("SUBSCRIPTIONS (collision alerts):                              ");
  blk("------------------------------------                            ");
  blk("  IVPHELM_BHV_ACTIVE = waypt_survey,avdcol_AIS366999901          ");
  blk("  IVPHELM_STATE      = DRIVE (the list is only believed in DRIVE)");
  blk("  COLAV_BHV_TRACE    = cn=..,mode=giveway:stern,.. (avdcol_ flag)");
  blk("  NODE_REPORT, NAV_X, NAV_Y, NAV_HEADING, NAV_SPEED             ");
  blk("                                                                ");
  blk("PUBLICATIONS (collision alerts):                                ");
  blk("------------------------------------                            ");
  blk("  COLAV_ALERTS = [{\"cn\":\"AIS366999901\",\"mmsi\":366999901,      ");
  blk("                 \"active\":true,\"for\":42,\"mode\":\"giveway:stern\",");
  blk("                 \"hdg\":218.7,\"spd\":2.57,\"range\":412.3,         ");
  blk("                 \"bng\":35.2,\"relBng\":356.5,\"cpa\":38.1,           ");
  blk("                 \"tcpa\":61.4,\"lat\":..,\"lon\":..,\"len\":25,      ");
  blk("                 \"type\":\"cargo\",\"age\":0.6}]                    ");
  blk("    m, degrees true, m/s, s; -1 unknown. Posted on change, at   ");
  blk("    most once a second; also sent to the UI on every tick.      ");
  blk("                                                                ");
  blk("  Commands from the UI are posted as their MOOS variables; see  ");
  blk("  the Swiftships-UI README for the list.                        ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iWebSocket", "gpl");
  exit(0);
}

