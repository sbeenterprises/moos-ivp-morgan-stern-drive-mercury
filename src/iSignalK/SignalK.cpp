/************************************************************/
/*    NAME: D Lima                                          */
/*    ORGN: MIT, Cambridge MA                                */
/*    FILE: SignalK.cpp                                      */
/*    DATE: December 29th, 1963                              */
/************************************************************/

#include <iterator>
#include <iostream>
#include <sstream>
#include <iomanip>         // Para std::setprecision, std::fixed
#include <curl/curl.h>     // Biblioteca libcurl
#include <jsoncpp/json/json.h>     // Biblioteca jsoncpp
#include "MBUtils.h"
#include "ACTable.h"
#include "SignalK.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "../MOOSGeodesy.cpp" 

using namespace std;

//-----------------------------------------------------------------
// Função auxiliar para receber dados do curl (chamada de callback)
//-----------------------------------------------------------------
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

//---------------------------------------------------------
// Constructor()

SignalK::SignalK()
{
}

//---------------------------------------------------------
// Destructor

SignalK::~SignalK()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool SignalK::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Exemplo de como extrair dados
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

bool SignalK::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            acontece AppTick vezes por segundo

bool SignalK::Iterate()
{
  AppCastingMOOSApp::Iterate();

  //-----------------------------------------------------------------
  // 1) Faz a requisição GET ao servidor SignalK
  //-----------------------------------------------------------------
  std::string readBuffer;
  
  // Inicializa sessão CURL
  CURL* curl = curl_easy_init();
  if(curl) {
    curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:3000/signalk/v2/api/vessels/self/navigation/course");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    // Se necessário, descomente para ignorar certificados SSL (apenas em ambientes de teste):
    // curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    // curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);

    CURLcode res = curl_easy_perform(curl);
    if(res != CURLE_OK) {
      reportRunWarning("Falha na requisição HTTP: " + string(curl_easy_strerror(res)));
    }
    curl_easy_cleanup(curl);
  }
  else {
    reportRunWarning("Não foi possível inicializar CURL.");
  }

  //-----------------------------------------------------------------
  // 2) Tenta interpretar o JSON recebido
  //-----------------------------------------------------------------
  if(!readBuffer.empty()) {
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    
    std::stringstream ss(readBuffer);
    bool parsingSuccessful = Json::parseFromStream(builder, ss, &root, &errs);

    
    if(!parsingSuccessful) {
      reportRunWarning("Falha ao interpretar JSON: " + errs);
    } 
    else {
      // Verifica se nextPoint está definido
      if(!root["nextPoint"].isNull()) {
        // Pega latitude e longitude
        double latitude  = root["nextPoint"]["position"]["latitude"].asDouble();
        double longitude = root["nextPoint"]["position"]["longitude"].asDouble();

        //Conversão de global para local para posterior envio para o MOOS

        // Morgan City
        double lat_origin = 29.71970895316288; //ALTERAR AQUI SE MUDAR A CARTA NÁUTICA !!!
        double lon_origin = -91.14705165281887;
        
        //Valores locais
        double nav_x = 0.0;
        double nav_y = 0.0;

        m_geodesy.Initialise(lat_origin, lon_origin);

        m_geodesy.LatLong2LocalUTM(latitude, longitude, nav_y, nav_x);

        // Converte para 1 dígito decimal
        std::ostringstream lat_str, lon_str;
        lat_str << std::fixed << std::setprecision(1) << nav_y;
        lon_str << std::fixed << std::setprecision(1) << nav_x;

        // Monta a string final, por exemplo: "polygon=29.731080,-91.145880"
        std::string wpt_update_str = "polygon=" + lon_str.str() + "," + lat_str.str();

        //-----------------------------------------------------------------
        // 3) Publica no MOOSDB
        //-----------------------------------------------------------------
        // Publica WPT_UPDATE
        Notify("WPT_UPDATE", wpt_update_str);

        // Publica as variáveis desejadas
        Notify("DEPLOY", "true");
        Notify("MOOS_MANUAL_OVERRIDE", "false");
        Notify("RETURN", "false");
      }
      else {
        // nextPoint == null -> não faz nada
        Notify("RETURN", "true");
      }
    }
  }

  //-----------------------------------------------------------------
  // Continua com o resto do ciclo
  //-----------------------------------------------------------------
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            acontece antes da conexão ser aberta

bool SignalK::OnStartUp()
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
    if(param == "foo") {
      // Exemplo de parâmetro "foo"
      handled = true;
    }
    else if(param == "bar") {
      // Exemplo de parâmetro "bar"
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void SignalK::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Exemplo: Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool SignalK::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: SignalK.cpp                           " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}
