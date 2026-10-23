/*****************************************************************/
/*    FILE: ControlLease.cpp                                     */
/*    See ControlLease.h                                         */
/*****************************************************************/

#include "ControlLease.h"
#include <sstream>

ControlLease::ControlLease() :
  m_hold_timeout(5), m_request_timeout(15), m_state("free"), m_gen(0),
  m_last_heard(0), m_request_deadline(0)
{
}

bool ControlLease::mayOperate(const std::string& role)
{
  return role == "operator" || role == "supervisor";
}

bool ControlLease::mayForce(const std::string& role)
{
  return role == "supervisor";
}

int ControlLease::rank(const std::string& role)
{
  if (role == "supervisor") return 3;
  if (role == "operator")   return 2;
  if (role == "planner")    return 1;
  return 0;
}

bool ControlLease::same(const Session& a, const Session& b)
{
  return !a.id.empty() && a.id == b.id && a.user == b.user;
}

bool ControlLease::isHolder(const Session& s) const
{
  return m_state == "held" && same(s, m_holder);
}

void ControlLease::event(const std::string& e)
{
  if (m_events.size() < 200)
    m_events.push_back(e);
}

std::vector<std::string> ControlLease::takeEvents()
{
  std::vector<std::string> out;
  out.swap(m_events);
  return out;
}

void ControlLease::grant(const Session& s, double now, const std::string& why)
{
  m_last_user = (m_state == "free") ? std::string() : m_holder.user;
  m_holder = s;
  m_state = "held";
  m_gen++;
  m_last_heard = now;
  m_reason = why;
  // a request is answered by any change of holder
  m_request = Session();
  std::ostringstream os;
  os << why << " user=" << s.user << " role=" << s.role << " gen=" << m_gen;
  if (!m_last_user.empty())
    os << " from=" << m_last_user;
  event(os.str());
}

ControlLease::Result ControlLease::take(const Session& s, double now)
{
  if (!mayOperate(s.role))
    return Result{false, "your role (" + (s.role.empty() ? "viewer" : s.role) +
                         ") cannot control the boat"};
  if (s.id.empty())
    return Result{false, "no session"};
  if (isHolder(s))
    return Result{true, ""};
  if (m_state == "held")
    return Result{false, "controlled by " + m_holder.user};
  grant(s, now, "take");
  return Result{true, ""};
}

ControlLease::Result ControlLease::release(const Session& s, bool boat_idle, double now)
{
  (void)now;
  if (!isHolder(s))
    return Result{false, "you do not have control"};
  if (!boat_idle)
    return Result{false, "the boat is running a mission, manual control or "
                         "station keep: hand control to another operator instead"};
  m_last_user = m_holder.user;
  m_holder = Session();
  m_state = "free";
  m_gen++;
  m_reason = "release";
  m_request = Session();
  event("release user=" + m_last_user);
  return Result{true, ""};
}

ControlLease::Result ControlLease::request(const Session& s, double now)
{
  if (!mayOperate(s.role))
    return Result{false, "your role cannot control the boat"};
  if (m_state != "held")
    return Result{false, "control is available: take it"};
  if (isHolder(s))
    return Result{false, "you already have control"};
  if (!m_request.id.empty() && !same(m_request, s))
    return Result{false, m_request.user + " has already asked for control"};
  m_request = s;
  m_request_deadline = now + m_request_timeout;
  event("request user=" + s.user + " holder=" + m_holder.user);
  return Result{true, ""};
}

ControlLease::Result ControlLease::answer(const Session& holder, bool accept, double now)
{
  if (!isHolder(holder))
    return Result{false, "you do not have control"};
  if (m_request.id.empty())
    return Result{false, "no request to answer"};
  if (!accept) {
    event("deny user=" + m_request.user + " holder=" + holder.user);
    m_request = Session();
    return Result{true, ""};
  }
  Session to = m_request;
  grant(to, now, "handover");
  return Result{true, ""};
}

ControlLease::Result ControlLease::cancelRequest(const Session& s)
{
  if (!same(m_request, s))
    return Result{false, "no request of yours"};
  event("cancel user=" + s.user);
  m_request = Session();
  return Result{true, ""};
}

ControlLease::Result ControlLease::force(const Session& s, const std::string& reason, double now)
{
  if (!mayForce(s.role))
    return Result{false, "only a supervisor can force a takeover"};
  if (s.id.empty())
    return Result{false, "no session"};
  if (isHolder(s))
    return Result{true, ""};
  if (reason.empty())
    return Result{false, "give a reason for the takeover"};
  grant(s, now, "force");
  event("force reason=" + reason);
  return Result{true, ""};
}

void ControlLease::heard(const Session& s, double now)
{
  if (isHolder(s))
    m_last_heard = now;
}

void ControlLease::logout(const Session& s)
{
  if (same(m_request, s)) {
    event("cancel user=" + s.user + " (logout)");
    m_request = Session();
  }
  if (isHolder(s)) {
    m_last_user = m_holder.user;
    m_holder = Session();
    m_state = "free";
    m_gen++;
    m_reason = "logout";
    m_request = Session();
    event("logout user=" + m_last_user);
  }
}

void ControlLease::gone(const Session& s)
{
  if (same(m_request, s)) {
    event("cancel user=" + s.user + " (disconnected)");
    m_request = Session();
  }
}

void ControlLease::tick(double now)
{
  if (m_state == "held" && now - m_last_heard > m_hold_timeout) {
    m_last_user = m_holder.user;
    m_holder = Session();
    m_state = "lost";
    m_gen++;
    m_reason = "timeout";
    m_request = Session();
    event("lost user=" + m_last_user + " (not heard)");
    return;
  }
  if (m_state == "held" && !m_request.id.empty() && now >= m_request_deadline) {
    Session to = m_request;
    grant(to, now, "handover-timeout");
  }
}

void ControlLease::revokeAll(const std::string& why)
{
  if (m_state == "free" && m_request.id.empty())
    return;
  m_last_user = m_holder.user;
  m_holder = Session();
  m_state = "free";
  m_gen++;
  m_reason = why;
  m_request = Session();
  event("revoke why=" + why + (m_last_user.empty() ? "" : " user=" + m_last_user));
}
