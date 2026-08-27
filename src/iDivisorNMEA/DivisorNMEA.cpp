/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DivisorNMEA.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <stdio.h>
#include <string.h>


// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"

//Includes do libais
#include "libais/ais.h"

//Includes do divisor NMEA

#include "DivisorNMEA.h"
#include "AisDecode.h"
#include "NMEAParserLib/NMEAParser.h"
#include "NMEAParserLib/NMEAParser.cpp"
#include "NMEAParserLib/NMEAParserData.h"
#include "NMEAParserLib/NMEAParserPacket.h"
#include "NMEAParserLib/NMEAParserPacket.cpp"
#include "NMEAParserLib/NMEASentenceBase.h"
#include "NMEAParserLib/NMEASentenceBase.cpp"
#include "NMEAParserLib/NMEASentenceGGA.h"
#include "NMEAParserLib/NMEASentenceGGA.cpp"
#include "NMEAParserLib/NMEASentenceGSA.h"
#include "NMEAParserLib/NMEASentenceGSA.cpp"
#include "NMEAParserLib/NMEASentenceGSV.h"
#include "NMEAParserLib/NMEASentenceGSV.cpp"
#include "NMEAParserLib/NMEASentenceRMC.cpp"
#include "NMEAParserLib/NMEASentenceRMC.h"

//Includes do libais
#include <memory>
#include <string>
#include "libais/ais.h"
#include "libais/ais.cpp"
#include "libais/ais_bitset.cpp"
#include "libais/ais1_2_3.cpp"

//Include do udpclient
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h> /* close() */
#include <string.h> /* memset() */

#define LOCAL_SERVER_PORT 10111 //Settings do recebedor UDP
#define MAX_MSG 1000 // Tamanho da mensagem UDP a ser recebida


//Include do geodesy
//Serve para fazer a conversão do LAT/LON recebido pelo AIS para Coordenadas Locais 
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "../MOOSGeodesy.cpp" // adiciono a funcao de referencia
using namespace std;

//Variáveis globais para usar no programa e debugs

double lat_gps;
double long_gps;
double speed_gps;
double heading_gps;
double wind_angle;
double wind_speed;
double nav_depth;
int contador_ais;
double heading_giro;
double angulo_leme;
// Set only from inside ProcessRxCommand(), i.e. not until a sentence has
// actually been parsed. They must not start as null: buildReport() builds
// std::strings from them, and constructing a std::string from null throws.
// The old blocking recvfrom() hid this by guaranteeing a sentence had arrived
// before the first appcast; a non-blocking read does not.
char* saida_pCmd  = (char*) "";
char* saida_pData = (char*) "";
int msg_debug;
int current_gear;

//Variáveis globais para o receiver UDP
int sd, rc, n;
socklen_t cliLen;
struct sockaddr_in cliAddr, servAddr;
char msg[MAX_MSG];


///
/// \class MyParser
/// \brief child class of CNMEAParser which will redefine notification calls from the parent class.
///
class MyNMEAParser : public CNMEAParser {

	///
	/// \brief This method is called whenever there is a parsing error.
	///
	/// Redefine this method to capture errors.
	///o
	/// \param pCmd Pointer to NMEA command that caused the error. Please note that this parameter may be NULL of not completely defined. Use with caution.
	///
	virtual void OnError(CNMEAParserData::ERROR_E nError, char *pCmd) {
		printf("ERROR for Cmd: %s, Number: %d\n", pCmd, nError);
	}

protected:
	///
	/// \brief This method is redefined from CNMEAParserPacket::ProcessRxCommand(char *pCmd, char *pData)
	///
	/// Here we are capturing the ProcessRxCommand to print out status. We also are looking for
	/// the GPGGA message and displaying some data from it.
	///
	/// \param pCmd Pointer to the NMEA command string
	/// \param pData Comma separated data that belongs to the command
	/// \return Returns CNMEAParserData::ERROR_OK If successful
	///
	virtual CNMEAParserData::ERROR_E ProcessRxCommand(char *pCmd, char *pData) {

		// Call base class to process the command
		try {
      CNMEAParser::ProcessRxCommand(pCmd, pData);
    }
    catch (std::system_error& e)
    {
      std::cout << e.what();
    }
    
    // Coloquei para debug
    saida_pCmd = pCmd;
    saida_pData = pData;
		printf("Cmd: %s\nData: %s\n", pCmd, pData);

		// Check if this is the GPGGA command. If it is, then display some data
		if (strstr(pCmd, "GPGGA") != NULL) {
			CNMEAParserData::GGA_DATA_T ggaData;
			if (GetGPGGA(ggaData) == CNMEAParserData::ERROR_OK) {
				//printf("GPGGA Parsed!\n");
				//printf("   Time:                %02d:%02d:%02d\n", ggaData.m_nHour, ggaData.m_nMinute, ggaData.m_nSecond);
				//lat_gps = ggaData.m_dLatitude;
        //long_gps = ggaDa"55.70,T";ta.m_dLongitude;
        //printf("   Latitude:            %f\n", ggaData.m_dLatitude);
				//printf("   Longitude:           %f\n", ggaData.m_dLongitude);
				//printf("   Altitude:            %.01fM\n", ggaData.m_dAltitudeMSL);
				//printf("   GPS Quality:         %d\n", ggaData.m_nGPSQuality);
				//printf("   Satellites in view:  %d\n", ggaData.m_nSatsInView);
				//printf("   HDOP:                %.02f\n", ggaData.m_dHDOP);
				//printf("   Differential ID:     %d\n", ggaData.m_nDifferentialID);
				//printf("   Differential age:    %f\n", ggaData.m_dDifferentialAge);
				//printf("   Geoidal Separation:  %f\n", ggaData.m_dGeoidalSep);
				//printf("   Vertical Speed:      %.02f\n", ggaData.m_dVertSpeed);
			}
		} else if (strstr(pCmd, "GPRMC") != NULL) {
      CNMEAParserData::RMC_DATA_T rmcdata;
      if (GetGPRMC(rmcdata) == CNMEAParserData::ERROR_OK) {
        lat_gps = rmcdata.m_dLatitude; // Latitude recebida
        long_gps = rmcdata.m_dLongitude; // Longitude recebida 
        //speed_gps = rmcdata.m_dSpeedKnots; // SOG do GPS
        //heading_gps = rmcdata.m_dTrackAngle; // Marcação vinda do GPS - tirei esse e coloquei o heading vindo da giro

      }
    } else if (strstr(pCmd, "GPHDT") != NULL) { //fix for boat 
      //Take the value from $GPHDT string and put into heading_giro
      std::string input = pData;
      std::stringstream ss(input);
      double number;
      char comma;

      ss >> number >> comma; // Read the number and discard ',T'

      heading_giro = number;

    }
    
  
    

		return CNMEAParserData::ERROR_OK;
	}
};

//---------------------------------------------------------
// Constructor()

DivisorNMEA::DivisorNMEA()
{
  //Configs para receber dados da serial
  //endereco_porta_serial = "/dev/pts/3"; // Porta para testes
  //baudrate = 9600;

  // Port the Signal K udp-nmea-sender plugin forwards every received NMEA0183
  // sentence to (~/.signalk/plugin-config-data/udp-nmea-sender.json).
  m_udp_port           = LOCAL_SERVER_PORT;
  m_ownship_mmsi       = 710400014;   // the launch's own transponder
  m_publish_nav        = true;
  m_report_interval    = 1.0;
  m_default_length     = 30.0;
  m_contact_stale_secs = 120.0;

  m_node_reports_sent  = 0;
  m_datagrams_seen     = 0;

  memset(read_buf,  0, sizeof(read_buf));
  memset(nmea_data, 0, sizeof(nmea_data));
}

//---------------------------------------------------------
// Destructor

DivisorNMEA::~DivisorNMEA()
{
    //close(serial_port);
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool DivisorNMEA::OnNewMail(MOOSMSG_LIST &NewMail)
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

     if(key == "FOO") 
       cout << "great!";

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool DivisorNMEA::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool DivisorNMEA::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // The socket is non-blocking (see OnStartUp), so drain everything that has
  // arrived since the last tick instead of taking one datagram per Iterate.
  // Blocking here - which is what this app used to do - freezes the whole
  // process the moment the sender goes quiet, and caps throughput at AppTick
  // sentences per second even when it does not.
  const int max_per_tick = 200;

  for(int i = 0; i < max_per_tick; i++) {
    memset(msg, 0x0, MAX_MSG);
    cliLen = sizeof(cliAddr);

    n = recvfrom(sd, msg, MAX_MSG - 1, 0,
                 (struct sockaddr *) &cliAddr, &cliLen);

    if(n < 0) {
      if((errno == EAGAIN) || (errno == EWOULDBLOCK))
        break;                       // nothing left to read this tick
      reportRunWarning("recvfrom failed: " + string(strerror(errno)));
      break;
    }
    if(n == 0)
      continue;

    msg[n] = '\0';
    m_datagrams_seen++;
    processSentence(string(msg));
  }

  m_ais.expireFragments(MOOSTime());
  postContactReports();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: processSentence()
//   One complete NMEA0183 sentence, as forwarded by the Signal K
//   udp-nmea-sender plugin (one sentence per datagram).

void DivisorNMEA::processSentence(const std::string& msg_string)
{
  Notify("MSG_UDP", msg_string); //Declarar uma variável pro MOOSDB

    //Parser de msg caso seja profundidade
  // Create a string stream to split the input string by commas
  std::stringstream ss(msg_string);
  std::string token;

  // Read the first token (should be "$SDDPT")
  std::getline(ss, token, ',');

  if (token == "$SDDPT") {
      // Read the second token and convert it to a floating-point number
      std::getline(ss, token, ',');
      nav_depth = std::stof(token);

      // Print the extracted nav_depth
      Notify("NAV_DEPTH", nav_depth);
  }

  // Crio um objeto para parse da msg NMEA
  MyNMEAParser NMEAParser;

  //Processo a sentença
  try {
    NMEAParser.ProcessNMEABuffer((char *) msg_string.c_str(),
                                 (int) msg_string.size());
  }
  catch (std::system_error& e)
  {
    std::cout << e.what();
  }

  // NAV_X / NAV_Y are not published here: pNodeReporter cross-fills them from
  // NAV_LAT / NAV_LONG (cross_fill_policy = global) and pXYpublisher puts them
  // back on the DB. The geodesy origin is set once in OnStartUp from the
  // mission's LatOrigin / LongOrigin - it used to be re-initialised from a
  // hard-coded literal on every single sentence.
  if(m_publish_nav) {
    //Atualizo variáveis necessárias para o movimento do navio
    Notify("NAV_LAT", lat_gps);
    Notify("NAV_LONG", long_gps);
    Notify("NAV_SPEED", speed_gps);
    Notify("NAV_HEADING", heading_giro);  // Notify NAV_HEADING

    //para testes
    Notify("GPS_HEADING", heading_gps);
  }

  NMEAParser.ResetData(); //Reseto dados NMEA
  
  // ---- AIS -------------------------------------------------------------
  // Decoded by AisDecode.{cpp,h}, the mirror of pTrafficSim's AisEncode. The
  // libais copy vendored here only ever built ais1_2_3.cpp, so it could not see
  // type 5 ship names or Class B at all.
  if((msg_string.compare(0, 6, "!AIVDM") == 0) ||
     (msg_string.compare(0, 6, "!AIVDO") == 0)) {

    AisReport rep;
    if(!m_ais.handleSentence(msg_string, rep, MOOSTime()))
      return;                        // buffered fragment, or not for us

    // Our own transponder echo must never become a contact, or the USV ends up
    // avoiding - or following - itself.
    if(rep.mmsi == m_ownship_mmsi)
      return;

    AisContact& c = m_contacts[rep.mmsi];
    c.mmsi = rep.mmsi;

    if(rep.has_position) {
      c.lat = rep.lat;
      c.lon = rep.lon;

      // AIS speed is knots; NODE_REPORT wants metres/second. Skipping this
      // conversion makes every contact appear to move at twice its real speed
      // and quietly poisons every CPA the helm computes.
      if(rep.sog >= 0)
        c.sog_ms = rep.sog / 1.94384449;

      // Small craft very often report heading as 511 (not available); course
      // over ground is the usable stand-in.
      if((rep.hdg >= 0) && (rep.hdg < 360))
        c.hdg = rep.hdg;
      else if(rep.cog >= 0)
        c.hdg = rep.cog;

      c.last_heard = MOOSTime();
    }

    if(rep.has_static) {
      if(!rep.shipname.empty())
        c.shipname = rep.shipname;
      if(!rep.callsign.empty())
        c.callsign = rep.callsign;
      if(rep.shiptype > 0)
        c.shiptype = rep.shiptype;
      if((rep.dim_a + rep.dim_b) > 0)
        c.length = rep.dim_a + rep.dim_b;
    }

    msg_debug = rep.msg_type;
  } //fechamento do if 
  //Parser do ângulo do leme
  else if (msg_string.substr(0,6) == "$AGRSA") {
    try {
      angulo_leme = stod(libais::GetNthField(msg_string,1,",")); //Pega o segundo campo da string NMEA
    }
    catch (std::system_error& e)
    {
      std::cout << e.what();
    }
    double ang = angulo_leme;
    Notify("ANGULO_LEME", ang);
    if(m_publish_nav)
      Notify("NAV_YAW", ang); //Variável de retorno do valor do ângulo do leme para o PID
  }

  //Parser do Rumo Verdadeiro dado pela Giro
  /*else if (msg_string.substr(0,6) == "$GPHDT") {
    try {
      heading_giro = stod(libais::GetNthField(msg_string,1,",")); //Pega o primeiro campo da string NMEA
      Notify("NAV_HEADING", heading_giro);      
    }
    catch (std::system_error& e)
    {
      std::cout << e.what();
    }
  }*/

  // Parser da Velocidade dada pelo GPS
  else if (msg_string.rfind("$GPRMC", 0) == 0) {  // rfind evita substr + ==  
    std::string sog_field = libais::GetNthField(msg_string, 7, ",");
    std::string cog_field = libais::GetNthField(msg_string, 8, ",");

    if (!sog_field.empty()) {
        try {
            speed_gps = std::stod(sog_field);
            if(m_publish_nav)
              Notify("NAV_SPEED", speed_gps);
        } catch (const std::invalid_argument&) {
            reportRunWarning("SOG inválido no GPRMC: '" + sog_field + "'");
        }
    } else {
        // Campo vazio -> GPS sem fix (Status = 'V') ou dado indisponível
        reportRunWarning("GPRMC recebido sem SOG – dado ignorado");
    }

    if (!cog_field.empty()) {
        try {
            heading_gps = std::stod(cog_field);   // graus verdadeiros 0-359.9
            if(m_publish_nav)
              Notify("NAV_COG", heading_gps);        // publique como preferir
        } catch (const std::invalid_argument&) {
            reportRunWarning("COG inválido no GPRMC: '" + cog_field + "'");
        }
    } else {
        reportRunWarning("GPRMC recebido sem COG – dado ignorado");
    }
}
  else if (msg_string.rfind("$VNYMR", 0) == 0) {
    std::string gyro_x = libais::GetNthField(msg_string, 9, ",");
    std::string gyro_y = libais::GetNthField(msg_string, 10, ",");
    std::string gyro_z = libais::GetNthField(msg_string, 11, ",");

    if (!gyro_x.empty()) {
        try {
            double gyro_x_value = std::stod(gyro_x);
            Notify("GYRO_X", gyro_x_value);
        } catch (const std::invalid_argument&) {
            reportRunWarning("Gyro X inválido no VNYMR: '" + gyro_x + "'");
        }
    } else {
        // Campo vazio -> GPS sem fix (Status = 'V') ou dado indisponível
        reportRunWarning("VNYMR recebido sem Gyro X – dado ignorado");
    }

    if (!gyro_y.empty()) {
        try {
            double gyro_y_value = std::stod(gyro_y);
            Notify("GYRO_Y", gyro_y_value);
        } catch (const std::invalid_argument&) {
            reportRunWarning("Gyro Y inválido no VNYMR: '" + gyro_y + "'");
        }
    } else {
        // Campo vazio -> GPS sem fix (Status = 'V') ou dado indisponível
        reportRunWarning("VNYMR recebido sem Gyro Y – dado ignorado");
    }

    if (!gyro_z.empty()) {
        try {
            double gyro_z_value = std::stod(gyro_z);
            Notify("GYRO_Z", gyro_z_value);
        } catch (const std::invalid_argument&) {
            reportRunWarning("Gyro Z inválido no VNYMR: '" + gyro_z + "'");
        }
    } else {
        // Campo vazio -> GPS sem fix (Status = 'V') ou dado indisponível
        reportRunWarning("VNYMR recebido sem Gyro Z – dado ignorado");
    }
  }

  //Parser da leitura do Anemômetro
  else if (msg_string.substr(0,6) == "$WIMWV") {
    try {
      wind_angle = stod(libais::GetNthField(msg_string,1,",")); //Pega o primeiro campo da string NMEA
      Notify("WIND_ANGLE", wind_angle);
      wind_speed = stold(libais::GetNthField(msg_string,3,",")); //Pega o terceiro campo da string NMEA
      Notify("WIND_SPEED", wind_speed);
    }
    catch (std::system_error& e) {
      std::cout << e.what();
    }
    catch (...) {
      std::cout << "\nThere is an error with the $WIMWV message! \n";
    }
  }

  # if 0
    else if (msg_string.substr(0,13) == "$MXPGN,01F205") {
      try {
        size_t lastCommaPos = msg_string.rfind(",");
        size_t asteriskPos = msg_string.find("*");

        if (lastCommaPos != std::string::npos && asteriskPos != std::string::npos && lastCommaPos < asteriskPos) {
            // Extract the substring between the last comma and the asterisk

            std::string hexSubstring = msg_string.substr(lastCommaPos + 1, asteriskPos - lastCommaPos - 1);

            // Convert the hexadecimal substring to binary
            std::stringstream ss;
            ss << std::hex << hexSubstring;
            unsigned long int intValue;
            ss >> intValue;

            // Convert to binary string
            std::bitset<64> binaryValue(intValue); 
            std::string binaryString = binaryValue.to_string();
            // Get the 9th and 10th bits
            char ninthBit = binaryString[8]; // 0-based indexing
            char tenthBit = binaryString[9];

            // Convert the bits to an integer value
            current_gear = (ninthBit - '0') * 2 + (tenthBit - '0');  
            Notify("CURRENT_GEAR", current_gear);    
        } //endif
      } //try
      catch (std::system_error& e) {
        std::cout << e.what();
      } //catch
    } //else if MXPGN
  #endif
  

  //Reseto os dados
  //NMEAParser.ResetData();
}

//---------------------------------------------------------
// Procedure: postContactReports()
//   Field list matches pTrafficSim exactly - which is already proven to be
//   what pContactMgrV20 accepts. Local X/Y only, no LAT/LON: pContactMgrV20
//   runs with contact_local_coords = force_lat_lon and fills those in itself.

void DivisorNMEA::postContactReports()
{
  double now = MOOSTime();

  std::map<unsigned int, AisContact>::iterator p = m_contacts.begin();
  while(p != m_contacts.end()) {
    AisContact& c = p->second;

    if((now - c.last_heard) > m_contact_stale_secs) {
      m_contacts.erase(p++);         // gone quiet - stop reporting it
      continue;
    }

    // Only report what we have actually heard. Re-posting the last known
    // position with a fresh TIME would make a contact that has stopped
    // transmitting look permanently current to pContactMgrV20, defeating its
    // own aging and leaving a ghost in the helm's contact picture.
    if((c.last_heard <= 0) ||        // static data only, never a position
       (c.last_heard <= c.last_posted) ||
       ((now - c.last_posted) < m_report_interval)) {
      ++p;
      continue;
    }

    // pNodeReporter cross-fills ownship's x/y from lat/lon with LatLong2LocalUTM
    // (cross_fill_policy = global), so contacts have to use the same projection
    // or they land in a frame ~100 m away from ownship and every CPA the helm
    // computes is wrong. See USE_UTM in the top-level CMakeLists.
    double nav_x = 0, nav_y = 0;
#ifdef USE_UTM
    m_geodesy.LatLong2LocalUTM(c.lat, c.lon, nav_y, nav_x);
#else
    m_geodesy.LatLong2LocalGrid(c.lat, c.lon, nav_y, nav_x);
#endif

    // vname is AIS<mmsi>, the same convention pTrafficSim uses. The MMSI is
    // the join key between MOOS and the UI, so the "Follow AIS contact"
    // right-click resolves these contacts with no UI change.
    double length = (c.length > 0) ? c.length : m_default_length;

    string report = "NAME=AIS" + to_string(c.mmsi) + ",";
    report += "TYPE=ship,";
    // The time the position was received, not the time we got round to
    // posting it - pContactMgrV20 ages contacts off this field.
    report += "TIME=" + doubleToString(c.last_heard, 2) + ",";
    report += "X=" + doubleToString(nav_x, 2) + ",";
    report += "Y=" + doubleToString(nav_y, 2) + ",";
    report += "SPD=" + doubleToString(c.sog_ms, 2) + ",";
    report += "HDG=" + doubleToString(c.hdg, 2) + ",";
    report += "YAW=" + doubleToString(c.hdg, 2) + ",";
    report += "DEPTH=0.0,";
    report += "LENGTH=" + doubleToString(length, 1) + ",";
    report += "MODE=AIS";

    Notify("NODE_REPORT", report);
    m_node_reports_sent++;
    c.last_posted = now;
    ++p;
  }
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool DivisorNMEA::OnStartUp()
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
    if(param == "udp_port") {
      m_udp_port = atoi(value.c_str());
      handled = (m_udp_port > 0);
    }
    else if(param == "ownship_mmsi") {
      m_ownship_mmsi = (unsigned int) atol(value.c_str());
      handled = true;
    }
    else if(param == "publish_nav") {
      handled = setBooleanOnString(m_publish_nav, value);
    }
    else if(param == "report_interval") {
      m_report_interval = atof(value.c_str());
      handled = (m_report_interval >= 0);
    }
    else if(param == "default_length") {
      m_default_length = atof(value.c_str());
      handled = (m_default_length > 0);
    }
    else if(param == "contact_stale_secs") {
      m_contact_stale_secs = atof(value.c_str());
      handled = (m_contact_stale_secs > 0);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Geodesy origin comes from the mission's global LatOrigin / LongOrigin, so
  // AIS contacts and ownship land on the same grid. The old code used a
  // Morgan City literal for ownship and a Rio de Janeiro one for AIS, which
  // would have put contacts thousands of kilometres away had the conversion
  // not also been commented out.
  double lat_origin = 0, lon_origin = 0;
  if(!m_MissionReader.GetValue("LatOrigin", lat_origin) ||
     !m_MissionReader.GetValue("LongOrigin", lon_origin)) {
    reportConfigWarning("LatOrigin / LongOrigin missing from the mission file");
    return(false);
  }
  if(!m_geodesy.Initialise(lat_origin, lon_origin)) {
    reportConfigWarning("Geodesy failed to initialise");
    return(false);
  }

  /* socket creation */
  sd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sd < 0) {
    reportConfigWarning("cannot open UDP socket");
    return(false);
  }

  /* bind local server port */
  servAddr.sin_family = AF_INET;
  servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servAddr.sin_port = htons(m_udp_port);
  rc = bind(sd, (struct sockaddr *) &servAddr, sizeof(servAddr));
  if(rc < 0) {
    reportConfigWarning("cannot bind UDP port " + intToString(m_udp_port) +
                        ": " + string(strerror(errno)));
    return(false);
  }

  // Non-blocking is what lets Iterate() drain the socket and move on. Left
  // blocking, this app stalls the instant the sender pauses - which reads on
  // the appcast exactly like Signal K having stopped forwarding.
  int flags = fcntl(sd, F_GETFL, 0);
  if((flags < 0) || (fcntl(sd, F_SETFL, flags | O_NONBLOCK) < 0)) {
    reportConfigWarning("cannot set the UDP socket non-blocking");
    return(false);
  }

  registerVariables();	

  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void DivisorNMEA::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("LINHA_NMEA", 0);
  Register("GYRO_X", 0);
  Register("GYRO_Y", 0);
  Register("GYRO_Z", 0);
  Register("NAV_COG", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool DivisorNMEA::buildReport() 
{
  #if 0
    m_msgs << "============================================" << endl;
    m_msgs << "File:                                       " << endl;
    m_msgs << "============================================" << endl;

    ACTable actab(4);
    actab << "Alpha | Bravo | Charlie | Delta";
    actab.addHeaderLines();
    actab << "one" << "two" << "three" << "four";
    m_msgs << actab.getFormattedString();
  #endif

  ACTable actab(4);
  actab << "msg_debug | Saida_PCmd | Saida_PData | String Recebida ";
  actab.addHeaderLines();
  actab << msg_debug
        << (saida_pCmd  ? saida_pCmd  : "")
        << (saida_pData ? saida_pData : "")
        << read_buf;
  m_msgs << actab.getFormattedString();
  m_msgs << endl << "============================================" << endl;

  ACTable actab2(5);
  actab2 << "Speed_GPS | Heading_GPS | Heading_Giro | Lat_Recebida_GPS | Long_Recebida_GPS ";
  actab2.addHeaderLines();
  actab2 << speed_gps << heading_gps << heading_giro << lat_gps << long_gps;
  m_msgs << actab2.getFormattedString();
  m_msgs << endl << "============================================" << endl;

  ACTable actab3(2);
  actab3 << "Wind_Speed | Wind_Angle";
  actab3.addHeaderLines();
  actab3 << wind_speed << wind_angle;
  m_msgs << actab3.getFormattedString();
  m_msgs << endl << "============================================" << endl;

  m_msgs << "UDP port:        " << m_udp_port
         << "   datagrams: "    << m_datagrams_seen << endl;
  m_msgs << "Ownship MMSI:    " << m_ownship_mmsi
         << "   publish NAV_*: " << (m_publish_nav ? "yes" : "no") << endl;
  m_msgs << "AIVDM sentences: " << m_ais.sentencesSeen()
         << "   decoded: "      << m_ais.messagesDecoded()
         << "   rejected: "     << m_ais.badSentences() << endl;
  m_msgs << "NODE_REPORTs:    " << m_node_reports_sent << endl;
  m_msgs << "============================================" << endl;

  ACTable actab4(6);
  actab4 << "MMSI | Name | Type | Len | Spd(m/s) | Hdg";
  actab4.addHeaderLines();
  std::map<unsigned int, AisContact>::const_iterator q;
  for(q = m_contacts.begin(); q != m_contacts.end(); ++q) {
    const AisContact& c = q->second;
    actab4 << c.mmsi
           << (c.shipname.empty() ? "-" : c.shipname)
           << c.shiptype
           << ((c.length > 0) ? c.length : m_default_length)
           << c.sog_ms
           << c.hdg;
  }
  m_msgs << actab4.getFormattedString();
  m_msgs << endl;

  return(true);
}
