/****************************************************************/
/*   FILE: NavGuard_Info.cpp                                    */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "NavGuard_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pNavGuard checks that the navigation data can be trusted by   ");
  blk("  comparing independent sources with each other:                ");
  blk("    heading  NAV_HEADING vs the track the GPS positions make    ");
  blk("             (straight running above min_speed)                 ");
  blk("    speed    NAV_SPEED vs the speed the GPS positions make      ");
  blk("    frozen   NAV_SPEED / NAV_COG / NAV_HEADING not changing while");
  blk("             the GPS positions show they should                 ");
  blk("    stale    no new GPS position for stale_secs                 ");
  blk("    jump     GPS position steps faster than jump_speed          ");
  blk("    gps_flag the receiver reports no fix (RMC status V)         ");
  blk("    follow   in autonomy the track stays follow_tol off the     ");
  blk("             commanded course for follow_secs (not turning in)  ");
  blk("  NAV_STATUS: warn while a check is failing, fault once it has  ");
  blk("  lasted its time; a fault holds until all is clean clear_secs. ");
  blk("  In a fault while the helm drives it posts NAV_GUARD_STOP and  ");
  blk("  iWebSocket ends the mission as the UI's Stop does. Manual     ");
  blk("  control is never affected.                                    ");
}

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pNavGuard file.moos [OPTIONS]                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pNavGuard with the given process name.            ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pNavGuard.                     ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pNavGuard Example MOOS Configuration                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pNavGuard                                       ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  nav_speed_units     = knots  // NAV_SPEED units: knots | mps  ");
  blk("  stop_on_fault       = true   // post NAV_GUARD_STOP in autonomy");
  blk("  nmea_var            = MSG_UDP  // raw NMEA (RMC status); none = off");
  blk("  min_speed           = 3      // nav_speed_units, checks above it");
  blk("  speed_tol           = 2      // nav_speed_units               ");
  blk("  jump_speed          = 60     // nav_speed_units               ");
  blk("  heading_tol         = 45     // deg, heading vs GPS track     ");
  blk("  heading_secs        = 10                                      ");
  blk("  speed_secs          = 8                                       ");
  blk("  freeze_secs         = 5      // speed / course                ");
  blk("  heading_freeze_secs = 10                                      ");
  blk("  stale_secs          = 3                                       ");
  blk("  jump_min_dist       = 15     // m                             ");
  blk("  gps_flag_secs       = 2                                       ");
  blk("  follow_tol          = 60     // deg                           ");
  blk("  follow_secs         = 20                                      ");
  blk("  clear_secs          = 10                                      ");
  blk("  window_secs         = 4      // GPS track / speed window      ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pNavGuard INTERFACE                                             ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_LAT, NAV_LONG, NAV_SPEED, NAV_COG, NAV_HEADING            ");
  blk("  DESIRED_HEADING, IVPHELM_ALLSTOP (clear = autonomy)           ");
  blk("  <nmea_var> (MSG_UDP: $--RMC status field)                     ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NAV_STATUS      = ok | warn | fault (on change and 1 Hz)      ");
  blk("  NAV_STATUS_MSG  = why, readable                               ");
  blk("  NAV_GUARD_STOP  = reason; every 2 s while in fault and the    ");
  blk("                    helm drives (iWebSocket stops autonomy)     ");
  blk("                                                                ");
  exit(0);
}

void showReleaseInfoAndExit()
{
  showReleaseInfo("pNavGuard", "gpl");
  exit(0);
}
