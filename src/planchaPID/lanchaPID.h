/************************************************************/
/*    NAME: Douglas Lima, Eduardo Eiras                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: lanchaPID.h                                     */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef lanchaPID_HEADER
#define lanchaPID_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>

class lanchaPID : public AppCastingMOOSApp
{
 public:
   lanchaPID();
   ~lanchaPID();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
};

class PIDController
{
  public :
    // Constructor
    PIDController(double kp, double ki, double kd);
    //PIDController(const std::string& filename);

    // Member Functions
    double Calculate(double desired, double current, double dt);
    double CalculateSpeed(double desired, double current, double dt);
    void setKP(double KP);
    void setKI(double KI);
    void setKD(double KD);
    void setUpper(double Upper);
    void setLower(double Lower);
    void resetIntegral();
    double getKP();
    double getKI();
    double getIterm();
    double getKD();
    double getEffKP();
    double getEffKI();
    double getEffKD();

  private :
    double kp;
    double ki;
    double kd;
    double integral;
    double prevError;
    double lower_bound;
    double upper_bound;
    double eff_kp;
    double eff_ki;
    double eff_kd;
    double prevMeas;

};

#endif 
