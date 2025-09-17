/************************************************************/
/*    NAME: D Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CANReceiver.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cstring>
#include <iomanip>
#include <sstream>
#include "MBUtils.h"
#include "ACTable.h"
#include "CANReceiver.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

CANReceiver::CANReceiver()
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

CANReceiver::~CANReceiver()
{
  if(m_can_socket != -1) {
    close(m_can_socket);
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool CANReceiver::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
   }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool CANReceiver::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool CANReceiver::Iterate()
{
  AppCastingMOOSApp::Iterate();

  readCANMessages();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool CANReceiver::OnStartUp()
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

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(!initCAN()) {
    reportConfigWarning("Failed to initialize CAN interface: " + m_can_interface);
    return(false);
  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void CANReceiver::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
}


//------------------------------------------------------------
// Procedure: buildReport()

bool CANReceiver::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: iCANReceiver - Mercury Engine Data   " << endl;
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
// Procedure: initCAN()

bool CANReceiver::initCAN()
{
  m_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(m_can_socket < 0) {
    return false;
  }

  strcpy(m_ifr.ifr_name, m_can_interface.c_str());
  ioctl(m_can_socket, SIOCGIFINDEX, &m_ifr);

  m_can_addr.can_family = AF_CAN;
  m_can_addr.can_ifindex = m_ifr.ifr_ifindex;

  if(bind(m_can_socket, (struct sockaddr *)&m_can_addr, sizeof(m_can_addr)) < 0) {
    close(m_can_socket);
    m_can_socket = -1;
    return false;
  }

  return true;
}

//---------------------------------------------------------
// Procedure: readCANMessages()

void CANReceiver::readCANMessages()
{
  if(m_can_socket == -1) {
    return;
  }

  struct can_frame frame;
  int nbytes;

  while((nbytes = read(m_can_socket, &frame, sizeof(struct can_frame))) > 0) {
    if(nbytes < sizeof(struct can_frame)) {
      continue;
    }

    uint32_t pgn = (frame.can_id >> 8) & 0x3FFFF;

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
    }
  }
}

//---------------------------------------------------------
// Procedure: parseEEC2() - Accelerator Pedal Position & Engine Load

void CANReceiver::parseEEC2(const can_frame& frame)
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

void CANReceiver::parseEEC1(const can_frame& frame)
{
  m_eec1_count++;

  if(frame.can_dlc >= 8) {
    // SPN 190 - Engine Speed (bytes 4-5)
    uint16_t speed_raw = (frame.data[4] << 8) | frame.data[3];
    if(speed_raw != 0xFFFF) { // 0xFFFF = not available
      m_engine_speed = speed_raw * 0.125; // 0.125 rpm per bit
      Notify("ENGINE_SPEED", m_engine_speed);
    }

    // SPN 513 - Engine Percent Torque (byte 2)
    uint8_t torque_raw = frame.data[1];
    if(torque_raw != 0xFF) { // 0xFF = not available
      m_engine_percent_torque = torque_raw - 125; // Offset -125%
      Notify("ENGINE_PERCENT_TORQUE", m_engine_percent_torque);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseDM1() - Diagnostic Trouble Codes

void CANReceiver::parseDM1(const can_frame& frame)
{
  m_dm1_count++;

  if(frame.can_dlc >= 8) {
    // DM1 message contains active DTCs
    // For simplicity, we'll track the presence of active DTCs
    bool has_active_dtcs = false;
    for(int i = 2; i < frame.can_dlc; i += 4) {
      if(i + 3 < frame.can_dlc) {
        uint32_t spn = ((frame.data[i+2] & 0xE0) << 11) | (frame.data[i+1] << 8) | frame.data[i];
        uint8_t fmi = frame.data[i+2] & 0x1F;

        if(spn != 0 && spn != 0x3FFFF) { // Valid SPN
          stringstream dtc_desc;
          dtc_desc << "FMI " << (int)fmi;
          m_active_dtcs[spn] = dtc_desc.str();
          has_active_dtcs = true;
        }
      }
    }

    Notify("ENGINE_ACTIVE_DTCS", (double)m_active_dtcs.size());
  }
}

//---------------------------------------------------------
// Procedure: parseET1() - Engine Temperatures

void CANReceiver::parseET1(const can_frame& frame)
{
  m_et1_count++;

  if(frame.can_dlc >= 8) {
    // SPN 110 - Engine Coolant Temperature (byte 1)
    uint8_t coolant_raw = frame.data[0];
    if(coolant_raw != 0xFF) { // 0xFF = not available
      m_engine_coolant_temp = coolant_raw - 40; // Offset -40°C
      Notify("ENGINE_COOLANT_TEMP", m_engine_coolant_temp);
    }

    // SPN 175 - Oil Temperature (byte 3)
    uint8_t oil_temp_raw = frame.data[2];
    if(oil_temp_raw != 0xFF) { // 0xFF = not available
      m_oil_temp = oil_temp_raw - 40; // Offset -40°C
      Notify("ENGINE_OIL_TEMP", m_oil_temp);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseEFLP1() - Engine Oil Pressure

void CANReceiver::parseEFLP1(const can_frame& frame)
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

void CANReceiver::parseLFE(const can_frame& frame)
{
  m_lfe_count++;

  if(frame.can_dlc >= 8) {
    // SPN 183 - Engine Fuel Rate (bytes 1-2)
    uint16_t fuel_raw = (frame.data[1] << 8) | frame.data[0];
    if(fuel_raw != 0xFFFF) { // 0xFFFF = not available
      m_fuel_rate = fuel_raw * 0.05; // 0.05 L/h per bit
      Notify("ENGINE_FUEL_RATE", m_fuel_rate);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseIC1() - Intake Manifold Pressure

void CANReceiver::parseIC1(const can_frame& frame)
{
  m_ic1_count++;

  if(frame.can_dlc >= 8) {
    // SPN 102 - Boost Pressure (byte 2)
    uint8_t pressure_raw = frame.data[1];
    if(pressure_raw != 0xFF) { // 0xFF = not available
      m_intake_manifold_pressure = pressure_raw * 2.0; // 2 kPa per bit
      Notify("ENGINE_INTAKE_MANIFOLD_PRESSURE", m_intake_manifold_pressure);
    }
  }
}

//---------------------------------------------------------
// Procedure: parseVEP() - Battery Potential (SPN 168)
// PGN 65271 (0xFEF7) - VEP1
// Escala: 0.05 V/bit, LSB-first em bytes 1–2

void CANReceiver::parseVEP(const can_frame& frame)
{
  m_vep_count++;

  // Para SPN 168 precisamos ao menos dos dois primeiros bytes
  if (frame.can_dlc >= 2) {
    // SPN 168: bytes 1–2 (índices 0 e 1), LSB-first
    uint16_t raw = static_cast<uint16_t>(frame.data[0])
                 | (static_cast<uint16_t>(frame.data[1]) << 8);

    // 0xFFFF = Not Available (J1939). Também descartamos 0x0000 (0.00 V) e ruídos fora da faixa.
    if (raw != 0xFFFF && raw != 0x0000) {
      double volts = raw * 0.05;  // 0.05 V/bit

      // Sanity check: manter apenas tensões plausíveis (10–30 V cobre sistemas 12/24 V)
      if (volts >= 10.0 && volts <= 30.0) {
        m_battery_potential = volts;
        Notify("ENGINE_BATTERY_VOLTAGE", m_battery_potential);
      }
    }
  }
}
