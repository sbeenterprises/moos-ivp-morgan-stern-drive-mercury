/****************************************************************/
/*   FILE: EngineWatch_Info.cpp                                 */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "EngineWatch_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pEngineWatch tells whether the engine is running and raises   ");
  blk("  engine alerts on the boat, with or without a UI open (same    ");
  blk("  rules as the SS UI engine monitor):                           ");
  blk("    run state  running (>= 300 rpm), stopped, unknown (no rpm   ");
  blk("               reading for 10 s)                                ");
  blk("    shutOffInMission  emergency: stopped 3 s while helm DRIVE   ");
  blk("    noDataInMission   warn: engine data lost while DRIVE        ");
  blk("    overheat     warn >= 95 C, alarm >= 100 C (clears < 93 C)   ");
  blk("    lowVoltage   warn < 12.0 V, alarm < 11.5 V (clears 12.2 V)  ");
  blk("    notCharging  warn: running, alternator < 12.8 V for 30 s    ");
  blk("    lowFuel      warn < 20 %, alarm < 10 % (clears 22 %)        ");
  blk("  Engine data comes from iCANRec (engine_source = moos) or the  ");
  blk("  Signal K server (engine_source = signalk, propulsion.<id>).   ");
  blk("  Alert only: it never commands the boat.                       ");
}

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pEngineWatch file.moos [OPTIONS]                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pEngineWatch with the given process name.          ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pEngineWatch.                  ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pEngineWatch Example MOOS Configuration                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pEngineWatch                                    ");
  blk("{                                                               ");
  blk("  AppTick   = 2                                                 ");
  blk("  CommsTick = 2                                                 ");
  blk("                                                                ");
  blk("  engine_source = signalk         // moos (iCANRec) | signalk   ");
  blk("  signalk_http  = 127.0.0.1:3000  // host:port, no scheme       ");
  blk("  signalk_token = <jwt>           // only if reads need a login ");
  blk("  engine_id     = port            // propulsion id; default first");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pEngineWatch INTERFACE                                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  IVPHELM_STATE (DRIVE = mission)                               ");
  blk("  engine_source = moos: ENGINE_SPEED, ENGINE_COOLANT_TEMP,      ");
  blk("    ENGINE_BATTERY_VOLTAGE, ENGINE_FUEL_LEVEL                   ");
  blk("  engine_source = signalk: GET signalk_http vessels/self, 1 Hz  ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  ENGINE_RUN_STATE = running | stopped | unknown (on change and ");
  blk("                     every 5 s)                                 ");
  blk("  ENGINE_ALERT     = key=<key>,level=<warn|alarm|emergency|     ");
  blk("                     clear>,msg=<text> (on each level change)   ");
  blk("                                                                ");
  exit(0);
}

void showReleaseInfoAndExit()
{
  showReleaseInfo("pEngineWatch", "gpl");
  exit(0);
}
