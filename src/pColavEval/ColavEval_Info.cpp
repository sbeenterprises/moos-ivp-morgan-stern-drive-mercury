/****************************************************************/
/*   NAME: DL                                                   */
/*   FILE: ColavEval_Info.cpp                                   */
/*   DATE: 2026                                                 */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "ColavEval_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pColavEval measures what the COLREGS avoidance behaviour       ");
  blk("  actually did, one record per encounter.                        ");
  blk("                                                                ");
  blk("  It listens to two things and keeps them apart on purpose:      ");
  blk("    - the behaviour's own view, through the COLAV_BHV_* flags    ");
  blk("      configured on the avdcol_ template: when the instance was  ");
  blk("      created, when it first influenced the helm, which COLREGS  ");
  blk("      mode it chose, when it saw its own CPA, when it ended;     ");
  blk("    - ground truth, tracked here from NODE_REPORT and NAV_*.     ");
  blk("                                                                ");
  blk("  The achieved CPA is taken from ground truth. The behaviour     ");
  blk("  runs on an extrapolated contact position, so its own range is  ");
  blk("  not the number to report as a result.                          ");
  blk("                                                                ");
  blk("  Nothing is measured unless a waypoint mission is running:      ");
  blk("  avdcol_ carries condition = WAYPOINT = true, and an idle       ");
  blk("  behaviour posts no contact flags.                              ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pColavEval file.moos [OPTIONS]                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pColavEval with the given process name.            ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pColavEval.                    ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pColavEval Example MOOS Configuration                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pColavEval                                      ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  // Verdict thresholds, on the achieved closest range.         ");
  blk("  collision_range = 20                                          ");
  blk("  near_miss_range = 60                                          ");
  blk("                                                                ");
  blk("  report_interval = 0.5   // seconds between COLAV_METRICS      ");
  blk("  max_records     = 6     // encounters kept and reported       ");
  blk("                                                                ");
  blk("  // Contacts named AIS<mmsi> inside this block are treated as  ");
  blk("  // targets the operator placed, not as scenario traffic.      ");
  blk("  // Must match pTrafficSim's spawn_mmsi_base.                  ");
  blk("  test_mmsi_low  = 366999900                                    ");
  blk("  test_mmsi_high = 366999999                                    ");
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
  blu("pColavEval INTERFACE                                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NAV_X, NAV_Y, NAV_HEADING, NAV_SPEED, DESIRED_HEADING         ");
  blk("                 Ownship ground truth.                          ");
  blk("  NODE_REPORT    Contact ground truth - the achieved closest    ");
  blk("                 range is measured from this, not from the      ");
  blk("                 behaviour's extrapolated range.                ");
  blk("  WPT_INDEX      Latched at detect. If it changes mid-encounter ");
  blk("                 the deflection figure is marked not valid, so  ");
  blk("                 a waypoint turn is never reported as an        ");
  blk("                 avoidance manoeuvre.                           ");
  blk("  WAYPOINT       Whether a mission is running at all.           ");
  blk("  COLAV_TEST     Test mode on/off, from the UI.                 ");
  blk("  TRAFFIC_SPAWNED  Ack from pTrafficSim; gives the moment a     ");
  blk("                 target was placed, so the detection delay can  ");
  blk("                 be separated from the reaction delay.          ");
  blk("  COLAV_BHV_SPAWN / _ACTIVE / _INACTIVE / _CPA / _END / _TRACE  ");
  blk("                 Flags configured on the avdcol_ template in    ");
  blk("                 alpha.bhv. Without them nothing is measured.   ");
  blk("  IVPHELM_LIFE_EVENT  Authoritative death notice, and the only  ");
  blk("                 source of the behaviour's post_mortem.         ");
  blk("  UCD_REPORT     Optional. uFldCollisionDetect's independent    ");
  blk("                 measurement of the same CPA, reported next to  ");
  blk("                 ours so a disagreement is visible.             ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  COLAV_METRICS  JSON array of the live and recent encounters,  ");
  blk("                 latest value only. A browser that reconnects   ");
  blk("                 gets the whole picture from the next post.     ");
  blk("  COLAV_ENCOUNTER  One flat key=value record per completed      ");
  blk("                 encounter, for the log.                        ");
  blk("  ENCOUNTER_SUMMARY  The same result in the stock spec, so      ");
  blk("                 alogcd and alogeplot can read the run.         ");
  blk("  COLAV_TEST_ACK  Echo of COLAV_TEST - proves to the UI that an ");
  blk("                 evaluator is actually present.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pColavEval", "gpl");
  exit(0);
}
