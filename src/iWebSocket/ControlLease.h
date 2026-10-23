/*****************************************************************/
/*    FILE: ControlLease.h                                       */
/*                                                               */
/*  Exclusive control of the boat. Plain C++ (no MOOS), unit     */
/*  tested in tests/controllease_test.cpp.                       */
/*                                                               */
/*  One UI session at a time holds the control lease; only it    */
/*  may command the boat. A session is one browser tab (its own  */
/*  id) of one user, so a second tab or device of the same       */
/*  account does not share control.                              */
/*                                                               */
/*  Roles: viewer, planner (never command), operator (may hold   */
/*  the lease), supervisor (operator who may also force a        */
/*  takeover). Every change of holder bumps the generation, so   */
/*  commands sent under an older one can be refused.             */
/*                                                               */
/*  held  -> lost   the holder was not heard for hold_timeout    */
/*  held  -> free   the holder released (boat idle) or logged out */
/*  any   -> free   revokeAll (MOOS link came back)              */
/*  free/lost -> held   an operator takes control               */
/*  held  -> held   handover: the holder accepts a request, a    */
/*                  request is not answered within               */
/*                  request_timeout, or a supervisor forces      */
/*  Nothing here changes what the boat does: a lost lease only   */
/*  stops the link credit, and the existing lost-link failsafe   */
/*  acts on that.                                                */
/*****************************************************************/

#ifndef CONTROL_LEASE_HEADER
#define CONTROL_LEASE_HEADER

#include <string>
#include <vector>

class ControlLease
{
 public:
  struct Session {
    std::string id;     // per browser tab
    std::string user;
    std::string role;   // viewer | planner | operator | supervisor
  };
  struct Result {
    bool ok;
    std::string reason; // why not, for the UI
  };

  ControlLease();

  void setHoldTimeout(double secs)    { m_hold_timeout = secs; }
  void setRequestTimeout(double secs) { m_request_timeout = secs; }
  double holdTimeout() const          { return m_hold_timeout; }

  static bool mayOperate(const std::string& role);   // operator, supervisor
  static bool mayForce(const std::string& role);     // supervisor
  // viewer < planner < operator < supervisor; unknown = viewer
  static int  rank(const std::string& role);

  Result take(const Session& s, double now);
  Result release(const Session& s, bool boat_idle, double now);
  Result request(const Session& s, double now);
  Result answer(const Session& holder, bool accept, double now);
  Result cancelRequest(const Session& s);
  Result force(const Session& s, const std::string& reason, double now);

  // Traffic from a session: keeps the lease alive if it is the holder
  void heard(const Session& s, double now);
  // The session logged out or lost its rights: drops its lease / request
  void logout(const Session& s);
  // The session's last connection closed: drops its request only (the
  // lease runs on until hold_timeout, so a reload can pick it up again)
  void gone(const Session& s);
  // Expires a silent holder and an unanswered request (auto-grant)
  void tick(double now);
  void revokeAll(const std::string& why);

  bool isHolder(const Session& s) const;
  bool held() const                    { return m_state == "held"; }
  const std::string& state() const     { return m_state; }     // held | free | lost
  const Session& holder() const        { return m_holder; }
  unsigned gen() const                 { return m_gen; }
  double lastHeard() const             { return m_last_heard; }
  const std::string& lastReason() const { return m_reason; }   // why the last change
  const std::string& lastUser() const  { return m_last_user; } // holder before it
  bool requestPending() const          { return !m_request.id.empty(); }
  const Session& requester() const     { return m_request; }
  double requestDeadline() const       { return m_request_deadline; }

  // Audit lines since the last call ("take user=alice gen=3", ...)
  std::vector<std::string> takeEvents();

 private:
  void grant(const Session& s, double now, const std::string& why);
  void event(const std::string& e);
  static bool same(const Session& a, const Session& b);

  double m_hold_timeout;
  double m_request_timeout;

  std::string m_state;
  Session     m_holder;
  unsigned    m_gen;
  double      m_last_heard;
  std::string m_reason;
  std::string m_last_user;

  Session     m_request;
  double      m_request_deadline;

  std::vector<std::string> m_events;
};

#endif
