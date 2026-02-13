/****************************************************************/
/*   NAME: Abdul Aziz                                           */
/*   ORGN: MIT Cambridge MA                                     */
/*   FILE: Multisensorfusion_Info.cpp                           */
/*   DATE: 2025                                                 */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "Multisensorfusion_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  Multi-sensor fusion for USV collision avoidance in rivers    ");
  blk("  and coastal waters. Integrates:                              ");
  blk("    - Radar plots/tracks (position measurements)               ");
  blk("    - AIS reports (MMSI, position, velocity)                   ");
  blk("    - Vision detections (bearing-only + classification)        ");
  blk("                                                                ");
  blk("  Key Design: Vision is bearing-only by default. Monocular     ");
  blk("  camera range estimates are unreliable for distant marine     ");
  blk("  targets. Vision provides classification and bearing          ");
  blk("  refinement for existing radar/AIS tracks.                    ");
  blk("                                                                ");
  blk("  Features:                                                     ");
  blk("    - IMM Kalman Filter tracker (CV + CT models)               ");
  blk("    - Bearing-only EKF for vision updates                      ");
  blk("    - Bayesian classification fusion                           ");
  blk("    - MMSI tagging from AIS                                    ");
  blk("    - Global Nearest Neighbor data association                 ");
  blk("    - Track management (tentative/confirmed/deleted)           ");
}

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pMultisensorfusion file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pMultisensorfusion with the given process name     ");
  blk("      rather than pMultisensorfusion.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pMultisensorfusion.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pMultisensorfusion Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pMultisensorfusion                              ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  // Geodesy (required for AIS lat/lon conversion)             ");
  blk("  lat_origin = 42.3601                                          ");
  blk("  lon_origin = -71.0589                                         ");
  blk("                                                                ");
  blk("  // Radar parameters                                           ");
  blk("  radar_noise_x = 5.0        // Position uncertainty (m)        ");
  blk("  radar_noise_y = 5.0                                           ");
  blk("                                                                ");
  blk("  // AIS parameters                                             ");
  blk("  ais_pos_noise = 10.0       // Position uncertainty (m)        ");
  blk("  ais_vel_noise = 0.5        // Velocity uncertainty (m/s)      ");
  blk("                                                                ");
  blk("  // Vision parameters (CRITICAL: bearing-only)                 ");
  blk("  vision_bearing_noise_deg = 2.0  // Angular noise (deg)        ");
  blk("  vision_can_initiate_tracks = false  // No vision-only tracks  ");
  blk("  vision_only_max_scans = 10          // Max scans if created   ");
  blk("                                                                ");
  blk("  // Camera calibration (Mode 2: pixel to bearing)             ");
  blk("  camera_fx = 1000.0         // Focal length (pixels)           ");
  blk("  camera_cx = 640.0          // Principal point                 ");
  blk("  camera_yaw_offset_deg = 0.0  // Mounting angle                ");
  blk("                                                                ");
  blk("  // Process noise                                              ");
  blk("  process_noise_cv = 0.5     // Constant velocity (m/s^2)       ");
  blk("                                                                ");
  blk("  // Data association                                           ");
  blk("  gate_threshold = 9.0       // Mahalanobis for radar/AIS       ");
  blk("  bearing_gate_threshold = 10.0  // Angular for vision (deg)    ");
  blk("                                                                ");
  blk("  // Track management                                           ");
  blk("  confirm_min_hits = 3                                          ");
  blk("  confirm_min_scans = 5                                         ");
  blk("  delete_max_misses = 5                                         ");
  blk("                                                                ");
  blk("  // Publishing                                                 ");
  blk("  publish_rate_hz = 2.0                                         ");
  blk("  publish_debug = true                                          ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pMultisensorfusion INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  RADAR_TARGETS  = Format: x=...,y=...,vx=...,label=...;...    ");
  blk("                   Radar detections/tracks in local coords     ");
  blk("  AIS_REPORT     = Format: mmsi=...,lat=...,lon=...,sog=...#...");
  blk("                   AIS target reports                           ");
  blk("  VISION_REPORT  = Format: class=...,confidence=...,bearing=...;");
  blk("                   Vision detections (bearing + class)          ");
  blk("  NAV_X          = Ownship X position (m, local)                ");
  blk("  NAV_Y          = Ownship Y position (m, local)                ");
  blk("  NAV_LAT        = Ownship latitude (deg)                       ");
  blk("  NAV_LON        = Ownship longitude (deg)                      ");
  blk("  NAV_HEADING    = Ownship heading (deg)                        ");
  blk("  NAV_SPEED      = Ownship speed (m/s)                          ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT    = Standard MOOS contact format                 ");
  blk("                   NAME=MMSI_xxx or RADAR_xx,X=...,Y=...,      ");
  blk("                   SPD=...,HDG=...,TYPE=...,SOURCE=RAV          ");
  blk("  FUSION_DEBUG   = Debug statistics (if publish_debug=true)     ");
  blk("                   tracks=N,confirmed=M,radar=...,ais=...,etc   ");
  blk("                                                                ");
  blk("NODE_REPORT is consumed by pContactMgr for collision avoidance  ");
  blk("                                                                ");
  exit(0);
}

void showReleaseInfoAndExit()
{
  showReleaseInfo("pMultisensorfusion", "mit");
  exit(0);
}
