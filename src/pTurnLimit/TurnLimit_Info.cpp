/****************************************************************/
/*   FILE: TurnLimit_Info.cpp                                   */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "TurnLimit_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pTurnLimit holds the boat to turn_speed while it turns hard,   ");
  blk("  so turns stay tight instead of wide arcs at speed. It never    ");
  blk("  steers; it posts TURN_LIMIT, which switches on the             ");
  blk("  turn_max_speed cap (BHV_MaxSpeed) in alpha.bhv. Limited when:  ");
  blk("    corner : within corner_dist of the route waypoint the boat   ");
  blk("             is heading for, and the route turns there by        ");
  blk("             corner_angle or more (with or without a corridor)   ");
  blk("    heading: DESIRED_HEADING is heading_trigger or more off      ");
  blk("             NAV_HEADING (region turn-away, avoidance, follow);  ");
  blk("             released at heading_release                        ");
  blk("  held for hold_secs afterwards; only while WAYPOINT, TRAILING   ");
  blk("  or STATIONING drives the boat and not in manual control.       ");
}

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pTurnLimit file.moos [OPTIONS]                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pTurnLimit with the given process name.            ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pTurnLimit.                    ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pTurnLimit Example MOOS Configuration                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pTurnLimit                                      ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  turn_speed         = 5    // helm units (knots on the boat)   ");
  blk("  heading_trigger    = 45   // deg off the desired heading      ");
  blk("  heading_release    = 15                                       ");
  blk("  corner_dist        = 100  // m to a sharp route corner        ");
  blk("  corner_angle       = 60   // deg the route turns there        ");
  blk("  hold_secs          = 3                                        ");
  blk("  desired_stale_secs = 3                                        ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pTurnLimit INTERFACE                                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X, NAV_Y, NAV_HEADING, DESIRED_HEADING                    ");
  blk("  WAYPOINT, TRAILING, STATIONING, MOOS_MANUAL_OVERRIDE          ");
  blk("  WPT_UPDATE (route: polygon=/points=/xpoints=, loop_start=,    ");
  blk("              repeat=), WPT_INDEX                               ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  TURN_LIMIT         = true/false (turn_max_speed condition)    ");
  blk("  TURN_LIMIT_REASON  = corner | heading | hold | (empty)        ");
  blk("  TURN_LIMIT_UPDATES = max_speed=<turn_speed>                   ");
  blk("                                                                ");
  exit(0);
}

void showReleaseInfoAndExit()
{
  showReleaseInfo("pTurnLimit", "gpl");
  exit(0);
}
