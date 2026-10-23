/************************************************************/
/*    FILE: EngineWatch.h                                   */
/*    pEngineWatch: engine run state and engine alerts      */
/*    (EngineWatchModel) on the boat, so they are raised    */
/*    and logged whether or not a UI is open. Alert only:   */
/*    it never commands the boat.                           */
/************************************************************/

#ifndef EngineWatch_HEADER
#define EngineWatch_HEADER

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "EngineWatchModel.h"

class EngineWatch : public AppCastingMOOSApp
{
 public:
  EngineWatch();
  ~EngineWatch();

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();
  void pollLoop();          // worker thread: Signal K HTTP at ~1 Hz
  void takeSignalK(double now);

 private: // Configuration
  std::string m_source;     // moos | signalk
  std::string m_sk_url;     // http://host:port/signalk/v1/api/vessels/self
  std::string m_sk_token;
  std::string m_engine_id;  // Signal K propulsion id; "" = first one

 private: // Signal K sample, written by the poll thread
  struct Sample {
    bool   ok;
    std::string error, engine_id;
    std::string rpm_ts;     // timestamp of revolutions: a new one = a new reading
    double rpm, temp, alt, batt, fuel;
    bool   has_rpm, has_temp, has_alt, has_batt, has_fuel;
  };
  std::mutex m_mutex;
  Sample m_sample;
  std::string m_last_rpm_ts;
  std::thread m_thread;
  std::atomic<bool> m_stop;

 private: // State
  EngineWatchModel m_model;
  std::string m_helm;       // IVPHELM_STATE
  std::string m_posted_state;
  double m_last_state_post;
  unsigned int m_alerts_raised;
  std::string m_sk_status;
  std::map<std::string, std::string> m_warnings;  // alert key -> run warning text
};

#endif
