// Unit tests for iWebSocket's ControlLease. Plain g++, no MOOS.
// Run via tests/run_tests.sh.

#include "ControlLease.h"
#include <iostream>
#include <string>

using namespace std;

static int g_checks = 0, g_failed = 0;
#define CHECK(cond, msg) do { g_checks++; if(!(cond)) { g_failed++; \
  cout << "FAIL " << __LINE__ << ": " << msg << endl; } } while(0)

static ControlLease::Session S(const string& id, const string& user, const string& role)
{
  ControlLease::Session s; s.id = id; s.user = user; s.role = role; return s;
}

int main()
{
  ControlLease::Session op1  = S("tab1", "alice", "operator");
  ControlLease::Session op1b = S("tab2", "alice", "operator");   // same account, other tab
  ControlLease::Session op2  = S("tab3", "bob", "operator");
  ControlLease::Session sup  = S("tab4", "carol", "supervisor");
  ControlLease::Session plan = S("tab5", "dave", "planner");
  ControlLease::Session view = S("tab6", "erin", "viewer");

  // Roles
  {
    ControlLease l;
    CHECK(!l.take(view, 0).ok, "viewer cannot take");
    CHECK(!l.take(plan, 0).ok, "planner cannot take");
    CHECK(l.state() == "free", "still free");
    CHECK(l.take(op1, 0).ok && l.isHolder(op1), "operator takes free control");
    CHECK(l.gen() == 1, "gen 1");
    CHECK(!l.isHolder(op1b), "same account, other tab: not the holder");
    CHECK(!l.take(op1b, 1).ok, "other tab of the same account cannot take");
    CHECK(!l.take(op2, 1).ok, "second operator cannot take held control");
    CHECK(l.take(op1, 1).ok && l.gen() == 1, "holder taking again: no change");
  }

  // Simultaneous takeover: the first one wins, the second is refused
  {
    ControlLease l;
    bool a = l.take(op1, 0).ok, b = l.take(op2, 0).ok;
    CHECK(a && !b && l.isHolder(op1), "only one of two simultaneous takes wins");
  }

  // Heartbeat loss -> lost, then another operator takes; old holder stays out
  {
    ControlLease l; l.setHoldTimeout(5);
    l.take(op1, 0);
    l.heard(op1, 3); l.tick(7.9);
    CHECK(l.isHolder(op1), "heard at 3 s: still held at 7.9 s");
    l.heard(op2, 7.95); l.tick(8.1);
    CHECK(l.state() == "lost" && l.lastUser() == "alice", "silent > 5 s: lost");
    CHECK(l.gen() == 2, "loss bumps the generation");
    l.heard(op1, 8.2);
    CHECK(!l.isHolder(op1), "previous owner reconnecting stays read-only");
    CHECK(l.take(op2, 9).ok && l.isHolder(op2), "another operator takes after confirming");
    CHECK(l.gen() == 3 && l.lastReason() == "take", "take after loss");
    CHECK(!l.take(op1, 9.5).ok, "previous owner cannot take while held");
  }

  // Release only when the boat is idle
  {
    ControlLease l; l.take(op1, 0);
    CHECK(!l.release(op1, false, 1).ok && l.isHolder(op1), "no release during a mission");
    CHECK(!l.release(op2, true, 1).ok, "only the holder releases");
    CHECK(l.release(op1, true, 1).ok && l.state() == "free", "release when idle");
  }

  // Request: accept, deny, timeout auto-grant, cancel
  {
    ControlLease l; l.setRequestTimeout(15); l.take(op1, 0);
    CHECK(l.request(op2, 1).ok && l.requestPending(), "operator asks for control");
    CHECK(!l.request(sup, 1).ok, "one request at a time");
    CHECK(!l.answer(op2, true, 2).ok, "only the holder answers");
    CHECK(l.answer(op1, false, 2).ok && !l.requestPending() && l.isHolder(op1), "deny keeps control");
    l.request(op2, 3);
    CHECK(l.answer(op1, true, 4).ok && l.isHolder(op2) && l.gen() == 2, "accept hands over");
    l.request(op1, 5);
    l.heard(op2, 19); l.tick(19.9);
    CHECK(l.isHolder(op2), "request not yet timed out");
    l.heard(op2, 20); l.tick(20.0);
    CHECK(l.isHolder(op1) && l.lastReason() == "handover-timeout", "unanswered request granted after 15 s");
    l.request(op2, 21);
    CHECK(l.cancelRequest(op2).ok && !l.requestPending(), "requester cancels");
    l.request(op2, 22); l.gone(op2);
    CHECK(!l.requestPending(), "request dropped when the requester disconnects");
    CHECK(!l.request(view, 23).ok, "viewer cannot ask");
  }

  // Supervisor force
  {
    ControlLease l; l.take(op1, 0);
    CHECK(!l.force(op2, "x", 1).ok, "operator cannot force");
    CHECK(!l.force(sup, "", 1).ok, "force needs a reason");
    CHECK(l.force(sup, "operator not answering", 1).ok && l.isHolder(sup), "supervisor forces");
    CHECK(l.lastUser() == "alice" && l.lastReason() == "force", "force recorded");
  }

  // Logout, disconnect, revoke
  {
    ControlLease l; l.take(op1, 0);
    l.gone(op1);
    CHECK(l.isHolder(op1), "a closed connection keeps the lease until the timeout");
    l.heard(op1, 2); l.tick(3);
    CHECK(l.isHolder(op1), "same tab back within the timeout: still the holder");
    l.logout(op1);
    CHECK(l.state() == "free" && l.lastReason() == "logout", "logout frees control");
    l.take(op2, 4);
    unsigned g = l.gen();
    l.revokeAll("moos-reconnect");
    CHECK(l.state() == "free" && l.gen() == g + 1, "revokeAll frees and bumps the generation");
    l.revokeAll("again");
    CHECK(l.gen() == g + 1, "revoking a free lease changes nothing");
  }

  // Audit events
  {
    ControlLease l; l.take(op1, 0); l.request(op2, 1); l.answer(op1, true, 2);
    vector<string> ev = l.takeEvents();
    CHECK(ev.size() == 3, "three events");
    CHECK(ev.size() == 3 && ev[2].find("handover user=bob") == 0, "handover logged");
    CHECK(l.takeEvents().empty(), "events drained");
  }

  cout << g_checks << " checks, " << g_failed << " failed" << endl;
  return g_failed ? 1 : 0;
}
