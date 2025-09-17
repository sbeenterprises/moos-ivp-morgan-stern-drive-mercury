/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CANRec.cpp                                      */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <sstream>
#include <cstring>
#include <cerrno>

// SocketCAN / system headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>

#include "MBUtils.h"
#include "ACTable.h"
#include "CANRec.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

CANRec::CANRec()
{
  m_can_socket = -1;
  m_can_interface = "slcan0";

  // Initialize engine data
  m_accelerator_pedal_position = 0.0;
  m_percent_load = 0.0;
  m_engine_speed = 0.0;
  m_engine_percent_torque = 0.0;
  m_engine_coolant_temp = 0.0;
  m_oil_temp = 0.0;
  m_oil_pressure = 0.0;
  m_fuel_rate = 0.0;
  m_intake_manifold_pressure = 0.0;
  m_battery_potential = 0.0;

  // Initialize message counters
  m_eec2_count = 0;
  m_eec1_count = 0;
  m_dm1_count = 0;
  m_et1_count = 0;
  m_eflp1_count = 0;
  m_lfe_count = 0;
  m_ic1_count = 0;
  m_vep_count = 0;
}

//---------------------------------------------------------
// Destructor

CANRec::~CANRec()
{
  if(m_can_socket != -1) {
    close(m_can_socket);
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool CANRec::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "FOO") {
      cout << "great!";
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool CANRec::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool CANRec::Iterate()
{
  AppCastingMOOSApp::Iterate();
  readCANMessages();           // non-blocking; safe in Iterate()
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool CANRec::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "can_interface") {
      m_can_interface = value;
      handled = true;
    }
    else if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(!initCAN()) {
    reportConfigWarning("Failed to initialize CAN interface");
    return(false);
  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void CANRec::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool CANRec::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "App: CANRec - Mercury Engine Data" << endl;
  m_msgs << "============================================" << endl;

  string can_status = (m_can_socket != -1) ? "CONNECTED" : "DISCONNECTED";
  m_msgs << "CAN Interface (" << m_can_interface << "): " << can_status << endl;
  m_msgs << "CAN Socket ID: " << m_can_socket << endl;
  m_msgs << endl;

  m_msgs << "Engine Data:" << endl;
  m_msgs << "  Accelerator Pedal Position: " << m_accelerator_pedal_position << "%" << endl;
  m_msgs << "  Engine Load: " << m_percent_load << "%" << endl;
  m_msgs << "  Engine Speed: " << m_engine_speed << " RPM" << endl;
  m_msgs << "  Engine Torque: " << m_engine_percent_torque << "%" << endl;
  m_msgs << "  Coolant Temperature: " << m_engine_coolant_temp << " °C" << endl;
  m_msgs << "  Oil Temperature: " << m_oil_temp << " °C" << endl;
  m_msgs << "  Oil Pressure: " << m_oil_pressure << " kPa" << endl;
  m_msgs << "  Fuel Rate: " << m_fuel_rate << " L/h" << endl;
  m_msgs << "  Intake Manifold Pressure: " << m_intake_manifold_pressure << " kPa" << endl;
  m_msgs << "  Battery Voltage: " << m_battery_potential << " V" << endl;
  m_msgs << endl;

  m_msgs << "Active DTCs: " << m_active_dtcs.size() << endl;
  for(auto& dtc : m_active_dtcs) {
    m_msgs << "  SPN " << dtc.first << ": " << dtc.second << endl;
  }
  m_msgs << endl;

  ACTable actab(2);
  actab << "PGN | Message Count";
  actab.addHeaderLines();
  actab << "EEC2 (61443)" << m_eec2_count;
  actab << "EEC1 (61444)" << m_eec1_count;
  actab << "DM1 (65226)" << m_dm1_count;
  actab << "ET1 (65262)" << m_et1_count;
  actab << "EFL/P1 (65263)" << m_eflp1_count;
  actab << "LFE (65266)" << m_lfe_count;
  actab << "IC1 (65270)" << m_ic1_count;
  actab << "VEP (65271)" << m_vep_count;
  m_msgs << actab.getFormattedString();

  return(true);
}

//---------------------------------------------------------
// Helper: make socket non-blocking

static bool make_nonblocking(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  if(flags < 0) return false;
  if(fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return false;
  return true;
}

//---------------------------------------------------------
// Procedure: initCAN()

bool CANRec::initCAN()
{
  m_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(m_can_socket < 0) {
    reportConfigWarning("socket(PF_CAN) failed");
    return false;
  }

  if(!make_nonblocking(m_can_socket)) {
    reportConfigWarning("fcntl(O_NONBLOCK) failed");
    close(m_can_socket);
    m_can_socket = -1;
    return false;
  }

  memset(&m_ifr, 0, sizeof(m_ifr));
  memset(&m_can_addr, 0, sizeof(m_can_addr));

  // Ensure we don't overflow ifr_name
  strncpy(m_ifr.ifr_name, m_can_interface.c_str(), IFNAMSIZ - 1);
  m_ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if(ioctl(m_can_socket, SIOCGIFINDEX, &m_ifr) < 0) {
    reportConfigWarning("ioctl(SIOCGIFINDEX) failed for iface=" + m_can_interface);
    close(m_can_socket);
    m_can_socket = -1;
    return false;
  }

  m_can_addr.can_family  = AF_CAN;
  m_can_addr.can_ifindex = m_ifr.ifr_ifindex;

  if(bind(m_can_socket, (struct sockaddr *)&m_can_addr, sizeof(m_can_addr)) < 0) {
    reportConfigWarning("bind(AF_CAN) failed");
    close(m_can_socket);
    m_can_socket = -1;
    return false;
  }

  // Optional: Only receive data frames; we can also configure filters later if desired.
  // (No filter set means accept all frames)

  return true;
}

//---------------------------------------------------------
// Helper: Extract J1939 PGN from a 29-bit extended CAN ID.
// Returns true if the frame is a valid extended data frame and sets pgn_out.
// Handles PDU1 vs PDU2 (zero PS for PDU1).

bool CANRec::extractPGN(const struct can_frame& frame, uint32_t& pgn_out) const
{
  // Reject error or RTR frames
  if(frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG))
    return false;

  // Require extended (29-bit) identifier
  if(!(frame.can_id & CAN_EFF_FLAG))
    return false;

  uint32_t id = frame.can_id & CAN_EFF_MASK; // strip flags, leave 29-bit id

  uint8_t PF = (id >> 16) & 0xFF;
  uint8_t PS = (id >> 8)  & 0xFF;
  // uint8_t SA = id & 0xFF; // source address if you need it later

  if(PF >= 240) { // PDU2
    pgn_out = (static_cast<uint32_t>(PF) << 8) | PS;
  } else {        // PDU1
    pgn_out = (static_cast<uint32_t>(PF) << 8); // PS zeroed
  }

  return true;
}

//---------------------------------------------------------
// Procedure: readCANMessages()
// Non-blocking: read as many frames as are ready, then return.

void CANRec::readCANMessages()
{
  if(m_can_socket == -1) {
    return;
  }

  struct can_frame frame;
  int nbytes;

  while(true) {
    nbytes = read(m_can_socket, &frame, sizeof(struct can_frame));

    if(nbytes < 0) {
      if(errno == EWOULDBLOCK || errno == EAGAIN) {
        // No more frames to read right now
        break;
      } else {
        // Some other read error; don’t spam warnings though
        break;
      }
    }

    if(nbytes < (int)sizeof(struct can_frame)) {
      // Partial frame; ignore
      continue;
    }

    uint32_t pgn = 0;
    if(!extractPGN(frame, pgn)) {
      // Not a valid 29-bit J1939 data frame; ignore
      continue;
    }

    switch(pgn) {
      case 61443: // EEC2
        parseEEC2(frame);
        break;
      case 61444: // EEC1
        parseEEC1(frame);
        break;
      case 65226: // DM1
        parseDM1(frame);
        break;
      case 65262: // ET1
        parseET1(frame);
        break;
      case 65263: // EFL/P1
        parseEFLP1(frame);
        break;
      case 65266: // LFE
        parseLFE(frame);
        break;
      case 65270: // IC1
        parseIC1(frame);
        break;
      case 65271: // VEP
        parseVEP(frame);
        break;
      default:
        // Optionally: track unknown/other PGNs here
        break;
    }
  }
}

//---------------------------------------------------------
// Procedure: parseEEC2() - Accelerator Pedal Position & Engine Load

void CANRec::parseEEC2(const can_frame& frame)
{
  m_eec2_count++;

  if(frame.can_dlc >= 8) {
    // SPN 91 - Accelerator Pedal Position (byte 2)
    uint8_t accel_raw = frame.data[1];
    if(accel_raw != 0xFF) { // 0xFF = not available
      m_accelerator_pedal_position = accel_raw * 0.4; // 0.4% per bit
      Notify("ENGINE_ACCELERATOR_PEDAL_POSITION", m_accelerator_pedal_position);
    }

    // SPN 92 - Percent Load (byte 3)
    uint8_t load_raw = frame.data[2];
    if(load_raw != 0xFF) { // 0xFF = not available
      m_percent_load = load_raw * 1.0; // 1% per bit
      Notify("ENGINE_PERCENT_LOAD", m_percent_load);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseEEC1() - Engine Speed & Torque

void CANRec::parseEEC1(const can_frame& frame)
{
  m_eec1_count++;

  if(frame.can_dlc >= 8) {
    // SPN 190 - Engine Speed (bytes 4-5)
    uint16_t speed_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[3];
    if(speed_raw != 0xFFFF) { // 0xFFFF = not available
      m_engine_speed = speed_raw * 0.125; // 0.125 rpm per bit
      Notify("ENGINE_SPEED", m_engine_speed);
    }

    // SPN 513 - Engine Percent Torque (byte 2)
    uint8_t torque_raw = frame.data[1];
    if(torque_raw != 0xFF) { // 0xFF = not available
      m_engine_percent_torque = static_cast<int>(torque_raw) - 125; // Offset -125%
      Notify("ENGINE_PERCENT_TORQUE", m_engine_percent_torque);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseDM1() - Diagnostic Trouble Codes

void CANRec::parseDM1(const can_frame& frame)
{
  m_dm1_count++;

  if(frame.can_dlc >= 8) {
    // DM1 format: lamps bytes 0-1, then 4-byte DTCs: SPN (19 bits), FMI (5 bits), OC (7 bits), CM(1)
    for(int i = 2; i + 3 < frame.can_dlc; i += 4) {
      uint32_t spn = ((static_cast<uint32_t>(frame.data[i+2] & 0xE0) << 11) |
                      (static_cast<uint32_t>(frame.data[i+1]) << 8) |
                       static_cast<uint32_t>(frame.data[i]));
      uint8_t fmi = frame.data[i+2] & 0x1F;
      // uint8_t oc  = frame.data[i+3] & 0x7F; // occurrence count, optional

      if(spn != 0 && spn != 0x3FFFF) {
        stringstream dtc_desc;
        dtc_desc << "FMI " << (int)fmi;
        m_active_dtcs[spn] = dtc_desc.str();
      }
    }

    Notify("ENGINE_ACTIVE_DTCS", (double)m_active_dtcs.size());
  }
}

//---------------------------------------------------------
// Procedure: parseET1() - Engine Temperatures

void CANRec::parseET1(const can_frame& frame)
{
  m_et1_count++;

  if(frame.can_dlc >= 8) {
    // SPN 110 - Engine Coolant Temperature (byte 1)
    uint8_t coolant_raw = frame.data[0];
    if(coolant_raw != 0xFF) { // 0xFF = not available
      m_engine_coolant_temp = static_cast<int>(coolant_raw) - 40; // Offset -40°C
      Notify("ENGINE_COOLANT_TEMP", m_engine_coolant_temp);
    }

    // SPN 175 - Oil Temperature (byte 3)
    uint8_t oil_temp_raw = frame.data[2];
    if(oil_temp_raw != 0xFF) { // 0xFF = not available
      m_oil_temp = static_cast<int>(oil_temp_raw) - 40; // Offset -40°C
      Notify("ENGINE_OIL_TEMP", m_oil_temp);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseEFLP1() - Engine Oil Pressure

void CANRec::parseEFLP1(const can_frame& frame)
{
  m_eflp1_count++;

  if(frame.can_dlc >= 8) {
    // SPN 100 - Engine Oil Pressure (byte 4)
    uint8_t pressure_raw = frame.data[3];
    if(pressure_raw != 0xFF) { // 0xFF = not available
      m_oil_pressure = pressure_raw * 4.0; // 4 kPa per bit
      Notify("ENGINE_OIL_PRESSURE", m_oil_pressure);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseLFE() - Engine Fuel Rate

void CANRec::parseLFE(const can_frame& frame)
{
  m_lfe_count++;

  if(frame.can_dlc >= 8) {
    // SPN 183 - Engine Fuel Rate (bytes 1-2)
    uint16_t fuel_raw = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[0];
    if(fuel_raw != 0xFFFF) { // 0xFFFF = not available
      m_fuel_rate = fuel_raw * 0.05; // 0.05 L/h per bit
      Notify("ENGINE_FUEL_RATE", m_fuel_rate);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseIC1() - Intake Manifold Pressure

void CANRec::parseIC1(const can_frame& frame)
{
  m_ic1_count++;

  if(frame.can_dlc >= 8) {
    // SPN 102 - Intake Manifold #1 Pressure (byte 2)
    uint8_t pressure_raw = frame.data[1];
    if(pressure_raw != 0xFF) { // 0xFF = not available
      m_intake_manifold_pressure = pressure_raw * 2.0; // 2 kPa per bit
      Notify("ENGINE_INTAKE_MANIFOLD_PRESSURE", m_intake_manifold_pressure);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseVEP() - Battery Potential

void CANRec::parseVEP(const can_frame& frame)
{
  m_vep_count++;

  if(frame.can_dlc >= 8) {
    // SPN 168 - Battery Potential / Power Input 1 (bytes 6-7, little-endian)
    uint16_t voltage_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[5];
    if(voltage_raw != 0xFFFF) { // 0xFFFF = not available
      m_battery_potential = voltage_raw * 0.05; // 0.05 V per bit
      Notify("ENGINE_BATTERY_VOLTAGE", m_battery_potential);
    }
  }
}
