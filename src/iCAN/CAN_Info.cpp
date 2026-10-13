/****************************************************************/
/*   NAME: DL                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: CAN_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "CAN_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  iCAN sends rudder, gear and throttle commands on CAN. The CAN   ");
  blk("  IDs, byte layout and scaling come from a per-boat profile file  ");
  blk("  (profiles/<boat>.prf); none are built into the program.        ");
  blk("  Only the current control owner may command the drive: SS UI    ");
  blk("  (iWebSocket) in manual, planchaPID in autonomy. Manual settings ");
  blk("  are held until changed; if the UI link is lost in manual, the   ");
  blk("  rudder and throttle go to the profile's lost-link values (0).   ");
  blk("  An invalid profile leaves iCAN running but never transmitting.  ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iCAN file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iCAN with the given process name         ");
  blk("      rather than iCAN.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iCAN.        ");
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
  blu("iCAN Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iCAN                                            ");
  blk("{                                                               ");
  blk("  AppTick   = 50              // frames are scheduled per message ");
  blk("  CommsTick = 50                                                ");
  blk("                                                                ");
  blk("  profile_file = ../../profiles/mercury_sterndrive.prf  (required)");
  blk("  can_interface = slcan0      // default: from the profile        ");
  blk("  manual_sources   = iWebSocket                                   ");
  blk("  autonomy_sources = planchaPID, pHelmIvP, iWebSocket             ");
  blk("  trim_sources     = iWebSocket                                   ");
  blk("  link_stale_secs  = 3        // UI_LINK_OK older than this = lost");
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
  blu("iCAN INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  DESIRED_RUDDER       = % of steering travel, + = starboard     ");
  blk("  DESIRED_THRUST       = %, + ahead / - astern (sign sets gear)  ");
  blk("  MOOS_MANUAL_OVERRIDE = true (SS UI owns the drive) / false     ");
  blk("  UI_LINK_OK           = true/false, posted by iWebSocket        ");
  blk("  CAN_SEND             = true/false, transmit enable             ");
  blk("  TRIM                 = 1 up pulse, -1 down pulse, 0 stop       ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  CAN_STATUS           = INHIBITED | NO_BUS | STANDBY | ACTIVE   ");
  blk("  CAN_CONTROL_OWNER    = MANUAL | AUTONOMY                       ");
  blk("  CAN_LOST_LINK        = true/false                              ");
  blk("  CAN_CONFLICT         = none | names of messages not sent       ");
  blk("                         because another node sends the same ID ");
  blk("  CAN_TX_RUDDER_PCT, CAN_TX_THRUST_PCT, CAN_TX_GEAR,             ");
  blk("  CAN_TRIM_STATE (up | down | idle)                              ");
  blk("                       = what is actually being sent             ");
  blk("  CAN_RUDDER_MISMATCH  = true when rudder feedback disagrees     ");
  blk("  CAN_PROFILE, CAN_PROFILE_HASH                                  ");
  blk("  plus every rx_field the profile defines (e.g.                  ");
  blk("  CAN_RUDDER_ACTUAL_PCT on USV-722)                              ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iCAN", "gpl");
  exit(0);
}

