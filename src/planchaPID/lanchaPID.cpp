/************************************************************/
/*    NAME: DL                                            */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: lanchaPID.cpp                                        */
/*    DATE: September, 2023                          */
/************************************************************/

#include "MBUtils.h"
#include "ACTable.h"
#include "lanchaPID.h"
//#include "PIDParameterReader.h"
#include <cmath>
#include <string>
#include <vector>
#include <iterator>

using namespace std;

//Variáveis globais
double nav_heading;  // Rumo atual do navio
double desired_heading;  // Rumo desejado do navio
double nav_speed;  // Velocidade atual do navio
double desired_speed;  // Velocidade desejada do navio
double nav_cog; // COG do navio
double nav_speed_filt = 0;  // filtered speed actually used by the code
bool   nav_speed_init = false;
double speed_alpha = 0.1;          //  nav_speed filter coefficient 10 % new sample, 90 % history
double heading_alpha     = 0.15;   // 15 % new sample, 85 % history
double nav_heading_filt  = 0.0;    // filtered heading (deg)
bool   nav_heading_init  = false;  // first-sample flag


double desired_thrust;  // Saída do controle de velocidade
double desired_rudder;  // Saída do controle de rumo

double speed_kp = 0.0; //kp da veloc
double speed_ki = 0.0; //ki da veloc
double speed_kd = 0.0; //kd da veloc
int thrust_map = 1;  // set to 1 to use thrust_map speed control

double heading_kp = 0.3; //kp do heading - Modified value for USV
double heading_ki = 0.0; //ki do heading
double heading_kd = 0.06; //kd do heading

double dt = 0.0;
double output = 0;
std::string deploy; //deploy
std::string moos_manual_overide;
std::string moos_manual_override;

std::string constant_heading; //Variável para verificar se o rumo constante é verdadeiro
double setpoint_heading; //Variável em que eu coloco o setpoint de rumo

//Check when autonomous control is active
std::string waypoint; //Variável para verificar se o controle autônomo é verdadeiro
std::string return_var; //

//Cria os objetos para o controle PID com valores iniciais de kp, ki e kd lidos do arquivo
//PIDController speedController("/home/dueiras/vsnt/moos-ivp-vsnt/src/planchaPID/speed_pid_parameters.txt");
//PIDController headingController("/home/dueiras/vsnt/moos-ivp-vsnt/src/planchaPID/heading_pid_parameters.txt");
PIDController speedController(speed_kp, speed_ki, speed_kd);
PIDController headingController(heading_kp, heading_ki, heading_kd);

//---------------------------------------------------------
// Constructor()

PIDController::PIDController(double kp, double ki, double kd)
{
  kp          = kp;
  ki          = ki;
  kd          = kd;
  upper_bound = 50; //Modified value for USV
  lower_bound = -50;
  integral    = 0;
  prevError   = 0;
  eff_kp      = 0;
  eff_ki      = 0;
  eff_kd      = 0;

}

// CalculateSpeed() – gain-scheduled PID + feed-forward
// thrust output is in the same 0–40 % scale you use elsewhere
double PIDController::CalculateSpeed(double desired, double current, double dt)
{
    // ------- 1. Feed-forward (static map) -----------------
    static const std::vector<double> ff_speed  = {0,1,2,3,4,6,8,10,12,14,18,20,22,24,26,28};
    static const std::vector<double> ff_thrust = {0,1,6,14,20,30,32,33,35,30,33,36,41,45,50,55};


    // linear interpolation
    double ff = ff_thrust.back();                 // default (highest bin)
    for(size_t i=1; i<ff_speed.size(); ++i){
        if(desired <= ff_speed[i]){
            double w = (desired - ff_speed[i-1])/(ff_speed[i]-ff_speed[i-1]);
            ff = ff_thrust[i-1] + w*(ff_thrust[i]-ff_thrust[i-1]);
            break;
        }
    }

    // ------- 2. PID feedback (gain-scheduled) -------------
    double error      = desired - current;
    integral         += error * dt;
    double derivative = (error - prevError)/dt;
    prevError         = error;

    // anti-wind-up

    if (integral > 60.0) {
        integral = 60.0;
    } else if (integral < -60.0) {
        integral = -60.0;
    }
    
    // gain scheduling on *measured* speed
    double v = current;
    if(v < 2.0){           // “get moving”
        eff_kp =  4.0;
        eff_ki =  1.2;
        eff_kd =  0.0;
    }
    else if(v < 6.0){      // “cruise”
        eff_kp =  2.0;
        eff_ki =  0.4;
        eff_kd =  0.05;
    }
    else{                  // “high drag”
        eff_kp =  1.8; // old value 1.2
        eff_ki =  0.15; // old value 0.1
        eff_kd =  0.12; // old value 0.12 
    }

    double fb = eff_kp*error + eff_ki*integral + eff_kd*derivative;

    // ------- 3. Combine and clamp -------------------------
    output = ff + fb;
    if (output < 0.0) {
        output = 0.0; // no negative thrust
    } else if (output > 60.0) {
        output = 60.0; // max thrust
    }
    //upper_bound = 40;                             // for AppCasting table
    //lower_bound = 0;

    return output;
}


//---------------------------------------------------------
// Calculate()

double PIDController::Calculate(double desired, double current, double dt) {
    
  
    double error = desired - current;

    //Passando para a escala de -180 a 180
    // Isso não é o ideal para o controlador de velocidade
    // Porém hoje os valores não chegam a -180 ou 180
    if (error < -180){
      error += 360;
    } else if (error > 180){
      error -= 360;
    }

    integral += error*dt; //Multiplico pelo período dt de tempo

    // 25-set-2023: add Anti-WindUp
    // Limits integral to maximum and minimum output values
    if (integral > upper_bound) {
      integral = upper_bound;
    }
    else if (integral < lower_bound) {
      integral = lower_bound;
    }

    double derivative = (error - prevError)/dt;

    //Gain Scheduling

    
    if (nav_speed < 2) {             
        eff_kp = 0.55;
        eff_ki = 0.0038;
        eff_kd = 0.060;  
        upper_bound = 20; // Modified value for USV
        lower_bound = -20; // Modified value for USV
    } 
    else if (nav_speed < 4) {       
        eff_kp = 0.45;
        eff_ki = 0.0026;
        eff_kd = 0.070;
        upper_bound = 20; // Modified value for USV
        lower_bound = -20; // Modified value for USV
    }
    else if (nav_speed < 6) {        
        eff_kp = 0.35;
        eff_ki = 0.0018;
        eff_kd = 0.080;  
        upper_bound = 20; // Modified value for USV
        lower_bound = -20; // Modified value for USV
    }
    else if (nav_speed < 8) {        
        eff_kp = 0.33;   
        eff_ki = 0.0014;
        eff_kd = 0.120;  
        upper_bound = 20; // Modified value for USV
        lower_bound = -20; // Modified value for USV
    }
    else if (nav_speed < 10) {            
        eff_kp      = 0.24;
        eff_ki      = 0.0010;
        eff_kd      = 0.18;
        upper_bound = 18; // Modified value for USV
        lower_bound = -18; // Modified value for USV
    }
    else if (nav_speed < 12) {       
        eff_kp      = 0.47;
        eff_ki      = 0;
        eff_kd      = 0.22;
        upper_bound = 10; // Modified value for USV
        lower_bound = -10; // Modified value for USV
    }
    else if (nav_speed < 15) {      
        eff_kp      = 0.47;
        eff_ki      = 0;
        eff_kd      = 0.22;
        upper_bound = 10; // Modified value for USV
        lower_bound = -10; // Modified value for USV
    }
    else if (nav_speed < 18.0) {      
        eff_kp      = 0.40;
        eff_ki      = 0;
        eff_kd      = 0.22;
        upper_bound = 10; // Modified value for USV
        lower_bound = -10; // Modified value for USV
    }
    else if (nav_speed < 20) {                             
        eff_kp      = 0.33;
        eff_ki      = 0;
        eff_kd      = 0.22;
        upper_bound = 10;
        lower_bound = -10; // Modified value for USV
    } else if (nav_speed < 22.5) {                             
        eff_kp      = 0.33;
        eff_ki      = 0;
        eff_kd      = 0.22;
        upper_bound = 10;
        lower_bound = -7; // Modified value for USV
    } else if (nav_speed < 25) {
        eff_kp = 0.3;
        eff_ki = 0;
        eff_kd = 0.22;
        upper_bound = 7;
        lower_bound = -5;
    } else {
        eff_kp = 0.25;
        eff_ki = 0;
        eff_kd = 0.2;
        upper_bound = 7;
        lower_bound = -5;
    }

    output = eff_kp * error + eff_ki * integral + eff_kd * derivative;

    // Limits the output value
    if (output > upper_bound)
      output = upper_bound;
    else if (output < lower_bound)
      output = lower_bound;

    prevError = error;

    return output;
} // Calculate()

//---------------------------------------------------------
// setKP()

void PIDController::setKP(double KP) {
  kp = KP;
} // setKP()

//---------------------------------------------------------
// setKI()

void PIDController::setKI(double KI) {
  ki = KI;
  integral = 0;
} // setKI()

//---------------------------------------------------------
// setKD()

void PIDController::setKD(double KD) {
  kd = KD;
} // setKD()

//---------------------------------------------------------
// setUpper()

void PIDController::setUpper(double Upper) {
  upper_bound = Upper;
} // setUpper()

//---------------------------------------------------------
// setLower()

void PIDController::setLower(double Lower) {
  lower_bound = Lower;
} // setKD()

//---------------------------------------------------------
// getKP()

double PIDController::getKP() {
  return kp;
} // getKP()

//---------------------------------------------------------
// getKI()

double PIDController::getKI() {
  return ki;
} // getKI()

//---------------------------------------------------------
// getIterm()

double PIDController::getIterm() {
  return integral;
} // getIterm()

//---------------------------------------------------------
// getKD()

double PIDController::getKD() {
  return kd;
} // getKD()

// Gain Schedulling functions

double PIDController::getEffKP() { return eff_kp; }
double PIDController::getEffKI() { return eff_ki; }
double PIDController::getEffKD() { return eff_kd; }

//---------------------------------------------------------
// resetIntegral()

void PIDController::resetIntegral() {
  integral = 0.0;
} // resetIntegral()

//---------------------------------------------------------
// Constructor()

lanchaPID::lanchaPID()
{
}

//---------------------------------------------------------
// Destructor

lanchaPID::~lanchaPID()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool lanchaPID::OnNewMail(MOOSMSG_LIST &NewMail)
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

     if(key == "NAV_HEADING") {
      double raw = msg.GetDouble();        // incoming 0–360 °

      if(!nav_heading_init) {              // first sample
          nav_heading_filt = raw;
          nav_heading_init = true;
      } else {
          // --- identical EMA structure to NAV_SPEED ---
          // 1) shortest signed difference (handles 0/360 wrap)
          double diff = raw - nav_heading_filt;
          if(diff > 180.0)       diff -= 360.0;
          else if(diff < -180.0) diff += 360.0;

          // 2) integrate a fraction of that difference
          nav_heading_filt += heading_alpha * diff;

          // 3) wrap result back to 0–360 for downstream code
          if(nav_heading_filt < 0.0)        nav_heading_filt += 360.0;
          else if(nav_heading_filt >= 360.) nav_heading_filt -= 360.0;
      }

      nav_heading = nav_heading_filt;       // PID sees filtered value
      }

     else if(key == "DESIRED_HEADING") 
       desired_heading = msg.GetDouble();
     else if(key == "NAV_SPEED"){
       double raw = msg.GetDouble();
       if(!nav_speed_init) {          // Filter to nav_speed
        nav_speed_filt  = raw;
        nav_speed_init  = true;
        } else {
            nav_speed_filt  = speed_alpha * raw +
                              (1.0 - speed_alpha) * nav_speed_filt;
        }

        nav_speed = nav_speed_filt;
        }
     else if(key == "DESIRED_SPEED") 
       desired_speed = msg.GetDouble();  
     else if(key == "SPEED_KP") 
       speedController.setKP(msg.GetDouble());
     else if(key == "SPEED_KI") 
       speedController.setKI(msg.GetDouble());
     else if(key == "SPEED_KD") 
       speedController.setKD(msg.GetDouble());
     else if(key == "HEADING_KP") 
       headingController.setKP(msg.GetDouble());
     else if(key == "HEADING_KI") 
       headingController.setKI(msg.GetDouble());
     else if(key == "HEADING_KD") 
       headingController.setKD(msg.GetDouble());
     else if(key == "DEPLOY") {
       deploy = msg.GetString(); 
       // Reset PID error  
       headingController.resetIntegral();
       speedController.resetIntegral();
       }
     else if(key == "MOOS_MANUAL_OVERIDE") 
       moos_manual_overide = msg.GetString();  
     else if(key == "CONSTANT_HEADING") 
       constant_heading = msg.GetString();  
     else if(key == "SETPOINT_HEADING") 
       setpoint_heading = msg.GetDouble();  
      else if(key == "WAYPOINT")
        waypoint = msg.GetString();
      else if(key == "RETURN")
        return_var = msg.GetString();
      else if(key == "NAV_COG")
        nav_cog = msg.GetDouble();
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool lanchaPID::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool lanchaPID::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  // Loop de controle
  
  // Calcule o controle de velocidade 
  // NÃO ESTÁ SENDO USADO
  if (thrust_map == 1) 
  {
    switch ((int) desired_speed) {
      case 0:
        desired_thrust = 0;
        break;
      case 1:
        desired_thrust = 20;
        break;
      case 4:
        desired_thrust = 34;
        break;
      case 5:
        desired_thrust = 44;
        break;
      case 6:
        desired_thrust = 48;
        break;
      default:
        desired_thrust = 0;
    } // end switch

  } else {
    desired_thrust = speedController.Calculate(desired_speed, nav_speed,dt);
  }

  // Calcule o controle de rumo
  //Caso o rumo constante esteja ativado o setpoint vai ser esse rumo
  if (constant_heading == "true"){
    desired_rudder = headingController.Calculate(setpoint_heading, nav_heading,dt);
  } else {
    desired_rudder = headingController.Calculate(desired_heading, nav_heading,dt);
  }

  //Speed controller - New function
  desired_thrust = speedController.CalculateSpeed(desired_speed, nav_speed,dt);

  // Imprima as saídas dos controladores

  //Publico DESIRED_RUDDER e DESIRED_THRUST
  //Just publish when waypoint behavior is true and manual override is false
  //BHV_Waypoint is active 
  if (waypoint == "true" && moos_manual_overide == "false") {
    Notify("DESIRED_THRUST", desired_thrust);
    Notify("DESIRED_RUDDER", desired_rudder);
  } 

  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool lanchaPID::OnStartUp()
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
    double value = std::stod(line);

    bool handled = false;
    if(param == "apptick") {
      dt = 1/value;
      handled = true;
    }
    else if(param == "heading_kp") {
      headingController.setKP(value);
      handled = true;
    }
    else if(param == "heading_ki") {
      headingController.setKI(value);
      handled = true;
    }
    else if(param == "heading_kd") {
      headingController.setKD(value);
      handled = true;
    }
    else if(param == "heading_upper_bound") {
      headingController.setUpper(value);
      handled = true;
    }
    else if(param == "heading_lower_bound") {
      headingController.setLower(value);
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

void lanchaPID::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_HEADING", 0);
  Register("DESIRED_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("DESIRED_SPEED", 0);
  Register("SPEED_KP", 0);
  Register("SPEED_KI", 0);
  Register("SPEED_KD", 0);
  Register("HEADING_KP", 0);
  Register("HEADING_KI", 0);
  Register("HEADING_KD", 0);
  Register("DEPLOY", 0);
  Register("MOOS_MANUAL_OVERIDE", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
  Register("CONSTANT_HEADING",0);
  Register("SETPOINT_HEADING",0);
  Register("WAYPOINT",0);
  Register("RETURN",0);
  

}

//------------------------------------------------------------
// Procedure: buildReport()

bool lanchaPID::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab1(4);
  actab1 << "DESIRED_THRUST | DESIRED_RUDDER | CONSTANT_HEADING | SETPOINT_HEADING ";
  actab1.addHeaderLines();
  actab1 << desired_thrust << desired_rudder << constant_heading << setpoint_heading;
  m_msgs << actab1.getFormattedString();
  m_msgs << endl;
  m_msgs << "---------------------------------------------------" << endl;

  ACTable actab2(3);
  actab2 << "EFF_KP | EFF_KI | EFF_KD";
  actab2.addHeaderLines();
  actab2 << headingController.getEffKP()
        << headingController.getEffKI()
        << headingController.getEffKD();
  m_msgs << actab2.getFormattedString();
  m_msgs << endl;

  

  return(true);
}