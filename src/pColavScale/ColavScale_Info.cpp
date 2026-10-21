/****************************************************************/
/*   FILE: ColavScale_Info.cpp                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "ColavScale_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pColavScale sets each contact's collision-avoidance distances  ");
  blk("  from how fast it is closing, so faster encounters start        ");
  blk("  avoiding earlier. It never steers. For every running avdcol_   ");
  blk("  instance (IVPHELM_LIFE_EVENT) it posts, on CONTACT_INFO,       ");
  blk("    pwt_outer_dist = clamp(outer_secs * v, outer_min, outer_max) ");
  blk("    pwt_inner_dist = clamp(inner_secs * v, inner_min, inner_max) ");
  blk("  v: the contact's speed relative to ours while the range closes ");
  blk("  (0 while it opens -> minimums). It also keeps pContactMgrV20's ");
  blk("  alert range at outer_max (the helm sets it to the template's   ");
  blk("  300 m at start-up). The UI's COLAV_DIST become the minimums.   ");
}

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pColavScale file.moos [OPTIONS]                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pColavScale with the given process name.           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pColavScale.                   ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pColavScale Example MOOS Configuration                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pColavScale                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  nav_speed_units     = knots  // NAV_SPEED: knots boat, mps sim ");
  blk("  contact_speed_units = mps    // NODE_REPORT SPD                ");
  blk("  outer_secs = 60              // start avoiding this early      ");
  blk("  inner_secs = 30              // full priority this early       ");
  blk("  outer_min  = 150                                              ");
  blk("  inner_min  = 75                                               ");
  blk("  outer_max  = 1400            // also the alert range           ");
  blk("  inner_max  = 700                                              ");
  blk("  cpa_range  = 1500            // keep >= avdcol_ completed_dist ");
  blk("  change_frac = 0.1            // re-post on a 10 % change ...   ");
  blk("  change_min  = 20             // ... of at least 20 m           ");
  blk("  contact_max_age   = 30                                        ");
  blk("  alert_repost_secs = 15                                        ");
  blk("  behavior_prefix   = avdcol_                                   ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pColavScale INTERFACE                                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X, NAV_Y, NAV_HEADING, NAV_SPEED, NODE_REPORT,            ");
  blk("  IVPHELM_LIFE_EVENT, COLAV_DIST, BCM_ALERT_REQUEST (others')   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  CONTACT_INFO      = name=<contact> # pwt_outer_dist=.. #      ");
  blk("                      pwt_inner_dist=..                         ");
  blk("  BCM_ALERT_REQUEST = id=avdcol_, alert_range=<outer_max>, ...  ");
  blk("  COLAV_SCALE       = <contact>=<outer>/<inner>,...             ");
  blk("                                                                ");
  exit(0);
}

void showReleaseInfoAndExit()
{
  showReleaseInfo("pColavScale", "gpl");
  exit(0);
}
