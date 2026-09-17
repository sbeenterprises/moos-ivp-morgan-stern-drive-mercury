/************************************************************/
/*    NAME: DL                                                        */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DivisorNMEA.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef DivisorNMEA_HEADER
#define DivisorNMEA_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "AisDecode.h"

#include <boost/asio.hpp>
#include <map>
#include <string>
#include <iostream>


#ifdef _WIN32
  #include "MOOS/libMOOS/Utils/MOOSNTSerialPort.h"
#else
  #include "MOOS/libMOOS/Utils/MOOSLinuxSerialPort.h"
#endif



class DivisorNMEA : public AppCastingMOOSApp
{
 public:
   DivisorNMEA();
   ~DivisorNMEA();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

   /** Handle one complete NMEA0183 sentence taken off the UDP socket. */
   void processSentence(const std::string& sentence);

   /** Post one NODE_REPORT per known AIS contact, rate limited to
    *  m_report_interval, dropping contacts not heard from in a while. */
   void postContactReports();

 private: // Configuration variables
    int serial_port;
    std::string   endereco_porta_serial;
    char* conteudo_porta_serial;
    int           baudrate;

    /** UDP port the Signal K udp-nmea-sender plugin forwards sentences to. */
    int           m_udp_port;
    /** Our own MMSI - never turned into a contact, or the USV chases itself. */
    unsigned int  m_ownship_mmsi;
    /** Publish NAV_LAT / NAV_LONG / NAV_SPEED / NAV_HEADING from the incoming
     *  GPS sentences. Set false when a simulator such as uSimMarineV22 owns
     *  NAV_* instead, so this app only contributes AIS contacts. */
    bool          m_publish_nav;
    double        m_report_interval;     // seconds between NODE_REPORTs
    double        m_default_length;      // metres, when AIS gives no dimensions
    double        m_contact_stale_secs;  // forget a contact after this silence

 private: // AIS contact state
    /** Position and static data accumulate separately: type 1 carries where a
     *  vessel is, type 5 carries what it is called, and they arrive minutes
     *  apart. One entry per MMSI merges them. */
    struct AisContact {
      unsigned int mmsi;
      std::string  shipname;
      std::string  callsign;
      int          shiptype;
      double       length;
      double       lat, lon;
      double       sog_ms;      // metres/second, already converted from knots
      double       hdg;         // degrees true
      double       last_heard;  // MOOSTime of the last position report
      double       last_posted; // MOOSTime of the last NODE_REPORT

      AisContact() : mmsi(0), shiptype(0), length(0), lat(0), lon(0),
                     sog_ms(0), hdg(0), last_heard(0), last_posted(0) {}
    };

    AisDecoder                        m_ais;
    std::map<unsigned int, AisContact> m_contacts;
    unsigned int                      m_node_reports_sent;
    unsigned int                      m_datagrams_seen;


 private: // State variables
    #ifdef _WIN32
      CMOOSNTSerialPort porta_serial;
    #else
      CMOOSLinuxSerialPort porta_serial;
    #endif 
    char read_buf [2048];
    char nmea_data [2048]; //Declarei pela divisão
    CMOOSGeodesy m_geodesy;
    
};

#endif 
