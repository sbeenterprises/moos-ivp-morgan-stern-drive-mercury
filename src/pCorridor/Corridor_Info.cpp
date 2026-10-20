/****************************************************************/
/*   FILE: Corridor_Info.cpp                                    */
/*   DATE: 2026                                                 */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "Corridor_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pCorridor keeps a waypoint mission inside a route corridor     ");
  blk("  drawn by the operator: a centreline polyline plus a full       ");
  blk("  width. Each iteration it measures the cross-track error (the   ");
  blk("  distance from NAV_X/NAV_Y to the nearest centreline segment)   ");
  blk("  and posts a zone the corridor_* behaviours in alpha.bhv act on:");
  blk("                                                                ");
  blk("    in          xte <= slow_frac * half                         ");
  blk("    slow        up to half the width     -> corridor_slow       ");
  blk("    recovering  up to stop_factor * half -> corridor_recover    ");
  blk("    stop        beyond that              -> corridor_stop       ");
  blk("    none        corridor off, no mission or no valid geometry   ");
  blk("                                                                ");
  blk("  Getting worse is immediate; getting better needs the xte to    ");
  blk("  clear the boundary by `hysteresis` metres.                     ");
  blk("  Active only while CORRIDOR=true and WAYPOINT=true.             ");
  blk("  A boat in 'stop' stays stopped until the operator acts         ");
  blk("  (CORRIDOR=off, a wider corridor, or manual control).           ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pCorridor file.moos [OPTIONS]                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pCorridor with the given process name.             ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pCorridor.                     ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pCorridor Example MOOS Configuration                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pCorridor                                       ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  slow_frac       = 0.5   // 'slow' beyond this share of half   ");
  blk("  stop_factor     = 2.0   // 'stop' beyond this many half-widths");
  blk("  hysteresis      = 2     // metres, to return to a better zone ");
  blk("  recover_lead    = 20    // metres ahead of the closest point  ");
  blk("  status_interval = 1     // seconds between repeat STATUS posts");
  blk("  recover_move    = 1     // metres the recover point must move ");
  blk("                          // before it is posted again          ");
  blk("  turn_slow_dist  = 60    // CORRIDOR_TURN within this of a vertex");
  blk("  turn_min_angle  = 60    // ...turning by at least this (degrees)");
  blk("  turn_allowance  = 0     // metres of turning room at sharp corners");
  blk("  slow_speed      = (unset) // corridor speeds in helm units, each  ");
  blk("  turn_speed      = (unset) // capped at the mission speed; unset =  ");
  blk("  recover_speed   = (unset) // keep the speed in alpha.bhv         ");
  blk("  stop_action     = stop  // or recover: steer back instead of stop");
  blk("  tighten_following       = true  // WPT_UPDATE on CORRIDOR edges");
  blk("  corridor_capture_radius = 15    // while a corridor is on      ");
  blk("  corridor_slip_radius    = 30                                    ");
  blk("  corridor_lead           = 60                                    ");
  blk("  route_capture_radius    = 60    // restored when it goes off   ");
  blk("  route_slip_radius       = 90    // (match waypt_survey)        ");
  blk("  route_lead              = 400                                   ");
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
  blu("pCorridor INTERFACE                                             ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X, NAV_Y      Ownship position, local metres.             ");
  blk("  CORRIDOR          true/false - corridor on (from iWebSocket). ");
  blk("  CORRIDOR_UPDATE   width=<w>,pts={x1,y1:x2,y2:...}             ");
  blk("                    Full width (both sides) and centreline.     ");
  blk("  WAYPOINT          true/false - waypoint mission running.      ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  CORRIDOR_STATUS   none | in | slow | recovering | stop        ");
  blk("                    On change and every status_interval.        ");
  blk("  XTE_NOW           Cross-track error in metres, every iteration");
  blk("                    while active; 0 once on deactivation.       ");
  blk("  CORRIDOR_RECOVER_UPDATES  points=x,y  While 'recovering': the ");
  blk("                    centreline point recover_lead metres ahead  ");
  blk("                    of the closest point (clamped to the end).  ");
  blk("                    Posted before CORRIDOR_STATUS.              ");
  blk("  CORRIDOR_SLOW_UPDATES / CORRIDOR_TURN_UPDATES  speed=<v>      ");
  blk("                    (and speed=<v> on CORRIDOR_RECOVER_UPDATES) ");
  blk("                    when configured, capped at the mission speed");
  blk("  CORRIDOR_TURN     true/false - near a sharp corridor vertex;  ");
  blk("                    corridor_turn slows the boat for the turn.  ");
  blk("  WPT_UPDATE        capture_radius / slip_radius / lead for     ");
  blk("                    waypt_survey: tight when CORRIDOR turns on, ");
  blk("                    route values when it turns off (only after  ");
  blk("                    a corridor was on; never otherwise).        ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pCorridor", "gpl");
  exit(0);
}
