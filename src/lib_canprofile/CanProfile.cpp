/************************************************************/
/*    FILE: CanProfile.cpp                                  */
/*    DATE: October 2026                                    */
/************************************************************/

#include "CanProfile.h"
#include "AutoTuning.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

using namespace std;

namespace canprofile {

//---------------------------------------------------------
// Small string helpers (kept local so the library has no
// MOOS dependency)

static string trim(const string& s)
{
  size_t a = s.find_first_not_of(" \t\r\n");
  if(a == string::npos)
    return "";
  size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

static string lower(string s)
{
  for(size_t i=0; i<s.size(); i++)
    s[i] = (char)tolower((unsigned char)s[i]);
  return s;
}

static vector<string> split(const string& s, char sep)
{
  vector<string> out;
  string cur;
  for(size_t i=0; i<s.size(); i++) {
    if(s[i] == sep) {
      out.push_back(trim(cur));
      cur.clear();
    }
    else
      cur += s[i];
  }
  out.push_back(trim(cur));
  return out;
}

// "a=1, b=2" -> {a:1, b:2}. Keys are lower-cased.
static map<string, string> keyValues(const string& s)
{
  map<string, string> kv;
  vector<string> items = split(s, ',');
  for(size_t i=0; i<items.size(); i++) {
    size_t eq = items[i].find('=');
    if(eq == string::npos) {
      if(!items[i].empty())
        kv[lower(items[i])] = "";
      continue;
    }
    kv[lower(trim(items[i].substr(0, eq)))] = trim(items[i].substr(eq + 1));
  }
  return kv;
}

static bool parseBool(const string& s, bool& out)
{
  string v = lower(s);
  if(v == "true" || v == "yes" || v == "1")  { out = true;  return true; }
  if(v == "false" || v == "no" || v == "0")  { out = false; return true; }
  return false;
}

static bool parseDouble(const string& s, double& out)
{
  if(s.empty())
    return false;
  char* end = 0;
  out = strtod(s.c_str(), &end);
  return end && *end == '\0';
}

static bool parseInt(const string& s, long long& out)
{
  if(s.empty())
    return false;
  char* end = 0;
  out = strtoll(s.c_str(), &end, 0);   // accepts 0x prefix
  return end && *end == '\0';
}

static bool parseHexInt(const string& s, unsigned long long& out)
{
  string v = s;
  if(v.size() > 2 && v[0] == '0' && (v[1] == 'x' || v[1] == 'X'))
    v = v.substr(2);
  if(v.empty())
    return false;
  for(size_t i=0; i<v.size(); i++)
    if(!isxdigit((unsigned char)v[i]))
      return false;
  out = strtoull(v.c_str(), 0, 16);
  return true;
}

static bool parseHexBytes(const string& s, vector<uint8_t>& out)
{
  string v = s;
  if(v.size() > 2 && v[0] == '0' && (v[1] == 'x' || v[1] == 'X'))
    v = v.substr(2);
  if(v.empty() || v.size() % 2 != 0)
    return false;
  out.clear();
  for(size_t i=0; i<v.size(); i+=2) {
    unsigned long long b;
    if(!parseHexInt(v.substr(i, 2), b))
      return false;
    out.push_back((uint8_t)b);
  }
  return true;
}

// "-100:0|0:511|100:1012"
static bool parsePoints(const string& s, vector<Point>& out)
{
  out.clear();
  vector<string> items = split(s, '|');
  for(size_t i=0; i<items.size(); i++) {
    size_t c = items[i].find(':');
    if(c == string::npos)
      return false;
    Point p;
    if(!parseDouble(trim(items[i].substr(0, c)), p.x) ||
       !parseDouble(trim(items[i].substr(c + 1)), p.y))
      return false;
    out.push_back(p);
  }
  return out.size() >= 2;
}

static double interp(const vector<Point>& pts, double x)
{
  if(x <= pts.front().x)
    return pts.front().y;
  if(x >= pts.back().x)
    return pts.back().y;
  for(size_t i=1; i<pts.size(); i++) {
    if(x <= pts[i].x) {
      double w = (x - pts[i-1].x) / (pts[i].x - pts[i-1].x);
      return pts[i-1].y + w * (pts[i].y - pts[i-1].y);
    }
  }
  return pts.back().y;
}

static void rawRange(int bits, bool is_signed, long long& lo, long long& hi)
{
  if(is_signed) {
    lo = -(1LL << (bits - 1));
    hi =  (1LL << (bits - 1)) - 1;
  }
  else {
    lo = 0;
    hi = (bits >= 63) ? 0x7FFFFFFFFFFFFFFFLL : ((1LL << bits) - 1);
  }
}

static string dirName(const string& path)
{
  size_t slash = path.find_last_of('/');
  if(slash == string::npos)
    return ".";
  if(slash == 0)
    return "/";
  return path.substr(0, slash);
}

static bool readFile(const string& path, string& out)
{
  ifstream in(path.c_str(), ios::in | ios::binary);
  if(!in)
    return false;
  stringstream ss;
  ss << in.rdbuf();
  out = ss.str();
  return true;
}

static string fnv1a(const string& s)
{
  uint64_t h = 1469598103934665603ULL;
  for(size_t i=0; i<s.size(); i++) {
    h ^= (unsigned char)s[i];
    h *= 1099511628211ULL;
  }
  char buf[17];
  snprintf(buf, sizeof(buf), "%016llx", (unsigned long long)h);
  return buf;
}

//---------------------------------------------------------
// Source signals: domains and the states of symbolic ones

static bool srcIsNumeric(const string& src)
{
  return src == "rudder_pct" || src == "thrust_pct" || src == "thrust_abs_pct";
}

static bool srcIsSymbolic(const string& src)
{
  return src == "rudder_side" || src == "gear" || src == "trim";
}

static void srcDomain(const string& src, double& lo, double& hi)
{
  lo = (src == "thrust_abs_pct") ? 0 : -100;
  hi = 100;
}

static vector<string> srcStates(const string& src)
{
  vector<string> s;
  if(src == "gear") {
    s.push_back("fwd"); s.push_back("neutral"); s.push_back("rev");
  }
  else if(src == "rudder_side") {
    s.push_back("port"); s.push_back("center"); s.push_back("stbd");
  }
  else if(src == "trim") {
    // "idle" is optional: some boats stop the motor simply when the
    // up/down frames stop arriving
    s.push_back("up"); s.push_back("down");
  }
  return s;
}

static string capOfSrc(const string& src)
{
  if(src == "rudder_pct" || src == "rudder_side")
    return "steer";
  if(src == "trim")
    return "trim";
  return "thrust";
}

//---------------------------------------------------------

string gearName(Gear gear)
{
  if(gear == GEAR_FWD) return "fwd";
  if(gear == GEAR_REV) return "rev";
  return "neutral";
}

string trimName(Trim trim)
{
  if(trim == TRIM_UP)   return "up";
  if(trim == TRIM_DOWN) return "down";
  return "idle";
}

string Frame::str() const
{
  char buf[64];
  string s;
  snprintf(buf, sizeof(buf), ext ? "%08X#" : "%03X#", id);
  s = buf;
  for(int i=0; i<dlc; i++) {
    snprintf(buf, sizeof(buf), "%02X", data[i]);
    s += buf;
  }
  return s;
}

//---------------------------------------------------------
// Profile

Profile::Profile() : m_loaded(false)
{
  // Defaults for the numeric settings. Every key a profile may set
  // must be listed here; anything else is reported as an error.
  m_settings["rudder_limit_pct"]     = 100;
  m_settings["thrust_limit_pct"]     = 100;
  m_settings["thrust_deadband_pct"]  = 0;
  m_settings["shift_neutral_secs"]   = 0;
  m_settings["shift_settle_secs"]    = 0;
  m_settings["rudder_rate_pct_s"]    = 0;    // 0 = no rate limit
  m_settings["thrust_rate_pct_s"]    = 0;
  m_settings["thrust_rate_up_pct_s"] = 0;    // ramp up only; cuts are immediate
  m_settings["guard_secs"]           = 1.0;
  m_settings["lost_link_rudder_pct"] = 0;
  m_settings["lost_link_thrust_pct"] = 0;
  m_settings["rudder_alarm_pct"]     = 0;    // 0 = no feedback alarm
  m_settings["rudder_alarm_secs"]    = 3.0;
  m_settings["trim_up_secs"]         = 1.0;  // length of one trim-up pulse
  m_settings["trim_down_secs"]       = 1.0;  // length of one trim-down pulse

  m_string_settings["rudder_feedback"] = "";

  // Speed-dependent rudder limit, manual and autopilot alike (iCAN):
  // speed:max-rudder% points in NAV_SPEED units. Empty = no speed limit.
  // Without a NAV_SPEED newer than speed_stale_secs the smallest limit
  // in the table applies.
  m_string_settings["rudder_limit_speed"] = "";
  m_settings["speed_stale_secs"]          = 3.0;

  // Autopilot (planchaPID) tuning, see AutoTuning.h. The defaults are the
  // values that used to be hardcoded in planchaPID / set in real.moos.
  m_settings["auto_heading_limit_pct"]     = 20;
  m_settings["auto_heading_i_authority_pct"] = 0;
  m_string_settings["auto_heading_gains"]  = DEFAULT_AUTO_HEADING_GAINS;
  m_settings["auto_rudder_max_pct"]        = 20;
  m_settings["auto_rudder_min_pct"]        = 8;
  m_settings["auto_rudder_speed_low"]      = 5;
  m_settings["auto_rudder_speed_high"]     = 18;
  m_settings["auto_thrust_max_pct"]        = 50;
  m_settings["auto_speed_i_authority_pct"] = 2;
  m_settings["auto_speed_filter"]          = 0.1;   // NAV_SPEED smoothing
  m_settings["auto_heading_filter"]        = 0.15;  // NAV_HEADING smoothing
  m_string_settings["auto_speed_ff"]       = DEFAULT_AUTO_SPEED_FF;
  m_string_settings["auto_speed_gains"]    = DEFAULT_AUTO_SPEED_GAINS;
}

double Profile::setting(const string& key) const
{
  map<string, double>::const_iterator p = m_settings.find(key);
  return (p == m_settings.end()) ? 0 : p->second;
}

string Profile::stringSetting(const string& key) const
{
  map<string, string>::const_iterator p = m_string_settings.find(key);
  return (p == m_string_settings.end()) ? "" : p->second;
}

bool Profile::hasCapability(const string& cap) const
{
  return find(m_caps.begin(), m_caps.end(), cap) != m_caps.end();
}

string Profile::capabilitiesString() const
{
  string s;
  for(size_t i=0; i<m_caps.size(); i++)
    s += (i ? "," : "") + m_caps[i];
  return s.empty() ? "none" : s;
}

void Profile::addError(int lineno, const string& msg)
{
  if(lineno > 0)
    m_errors.push_back("line " + to_string(lineno) + ": " + msg);
  else
    m_errors.push_back(msg);
}

TxMsg* Profile::findMsg(const string& name)
{
  for(size_t i=0; i<m_tx.size(); i++)
    if(m_tx[i].name == name)
      return &m_tx[i];
  return 0;
}

bool Profile::load(const string& path)
{
  string text;
  if(!readFile(path, text)) {
    m_errors.clear();
    m_loaded = false;
    addError(0, "cannot read profile file " + path);
    return false;
  }
  return loadFromString(text, dirName(path));
}

bool Profile::loadFromString(const string& text, const string& base_dir)
{
  m_base_dir = base_dir;
  m_errors.clear();
  m_tx.clear();
  m_rx.clear();
  m_caps.clear();
  m_hash_input = text;

  istringstream in(text);
  string line;
  int lineno = 0;
  while(getline(in, line)) {
    lineno++;
    size_t c = line.find("//");
    if(c != string::npos)
      line = line.substr(0, c);
    line = trim(line);
    if(line.empty() || line[0] == '#')
      continue;
    size_t eq = line.find('=');
    if(eq == string::npos) {
      addError(lineno, "expected key = value");
      continue;
    }
    parseLine(lower(trim(line.substr(0, eq))), trim(line.substr(eq + 1)), lineno);
  }

  m_loaded = true;
  validate();
  m_hash = fnv1a(m_hash_input);
  return m_errors.empty();
}

void Profile::parseLine(const string& key, const string& val, int lineno)
{
  if(key == "profile")
    m_name = val;
  else if(key == "can_interface")
    m_can_interface = val;
  else if(key == "capabilities") {
    vector<string> caps = split(val, ',');
    for(size_t i=0; i<caps.size(); i++) {
      string cap = lower(caps[i]);
      if(cap.empty() || cap == "none")
        continue;
      if(cap != "steer" && cap != "thrust" && cap != "trim")
        addError(lineno, "unknown capability '" + cap + "' (supported: steer, thrust, trim)");
      else
        m_caps.push_back(cap);
    }
  }
  else if(key == "tx_msg")
    parseTxMsg(val, lineno);
  else if(key == "tx_field")
    parseTxField(val, lineno);
  else if(key == "tx_const")
    parseTxConst(val, lineno);
  else if(key == "rx_field")
    parseRxField(val, lineno);
  else if(m_string_settings.count(key))
    m_string_settings[key] = val;
  else if(m_settings.count(key)) {
    double d;
    if(!parseDouble(val, d))
      addError(lineno, key + " must be a number");
    else
      m_settings[key] = d;
  }
  else
    addError(lineno, "unknown setting '" + key + "'");
}

void Profile::parseTxMsg(const string& val, int lineno)
{
  map<string, string> kv = keyValues(val);
  TxMsg msg;
  msg.name = kv["name"];
  msg.cap  = lower(kv["cap"]);

  if(msg.name.empty())
    addError(lineno, "tx_msg needs name=");
  else if(findMsg(msg.name))
    addError(lineno, "duplicate tx_msg name '" + msg.name + "'");
  if(msg.cap != "steer" && msg.cap != "thrust" && msg.cap != "trim")
    addError(lineno, "tx_msg " + msg.name + ": cap must be steer, thrust or trim");

  unsigned long long id;
  if(!parseHexInt(kv["id"], id) || id > 0x1FFFFFFF)
    addError(lineno, "tx_msg " + msg.name + ": bad id '" + kv["id"] + "' (hex)");
  else
    msg.id = (uint32_t)id;

  msg.ext = (msg.id > 0x7FF);
  if(kv.count("ext") && !parseBool(kv["ext"], msg.ext))
    addError(lineno, "tx_msg " + msg.name + ": ext must be true or false");
  if(!msg.ext && msg.id > 0x7FF)
    addError(lineno, "tx_msg " + msg.name + ": id too large for an 11-bit frame");

  long long n;
  if(!parseInt(kv["dlc"], n) || n < 1 || n > 8)
    addError(lineno, "tx_msg " + msg.name + ": dlc must be 1..8");
  else
    msg.dlc = (int)n;

  if(!parseInt(kv["period_ms"], n) || n < 10 || n > 1000)
    addError(lineno, "tx_msg " + msg.name + ": period_ms must be 10..1000");
  else
    msg.period_ms = (int)n;

  if(kv.count("fill")) {
    unsigned long long f;
    if(!parseHexInt(kv["fill"], f) || f > 0xFF)
      addError(lineno, "tx_msg " + msg.name + ": fill must be one hex byte");
    else
      msg.fill = (uint8_t)f;
  }
  if(kv.count("guard") && !parseBool(kv["guard"], msg.guard))
    addError(lineno, "tx_msg " + msg.name + ": guard must be true or false");

  const char* known[] = {"name","cap","id","ext","dlc","period_ms","fill","guard"};
  for(map<string,string>::iterator p=kv.begin(); p!=kv.end(); p++)
    if(find(known, known + 8, p->first) == known + 8)
      addError(lineno, "tx_msg: unknown parameter '" + p->first + "'");

  m_tx.push_back(msg);
}

void Profile::parseTxField(const string& val, int lineno)
{
  map<string, string> kv = keyValues(val);
  TxMsg* msg = findMsg(kv["msg"]);
  if(!msg) {
    addError(lineno, "tx_field: no tx_msg named '" + kv["msg"] + "' (define tx_msg first)");
    return;
  }
  TxField f;
  f.src = lower(kv["src"]);
  if(!srcIsNumeric(f.src) && !srcIsSymbolic(f.src))
    addError(lineno, "tx_field: unknown src '" + f.src + "'");

  long long n;
  if(!parseInt(kv["byte"], n) || n < 0 || n > 7)
    addError(lineno, "tx_field: byte must be 0..7");
  else
    f.byte = (int)n;
  if(!parseInt(kv["bits"], n) || (n != 8 && n != 16 && n != 24 && n != 32))
    addError(lineno, "tx_field: bits must be 8, 16, 24 or 32");
  else
    f.bits = (int)n;

  string order = lower(kv["order"]);
  if(order == "be")
    f.big_endian = true;
  else if(order != "" && order != "le")
    addError(lineno, "tx_field: order must be le or be");

  if(kv.count("signed") && !parseBool(kv["signed"], f.is_signed))
    addError(lineno, "tx_field: signed must be true or false");

  f.map = lower(kv["map"]);
  if(f.map == "linear") {
    if(kv.count("scale") && !parseDouble(kv["scale"], f.scale))
      addError(lineno, "tx_field: bad scale");
    if(kv.count("offset") && !parseDouble(kv["offset"], f.offset))
      addError(lineno, "tx_field: bad offset");
  }
  else if(f.map == "points") {
    if(!parsePoints(kv["points"], f.points))
      addError(lineno, "tx_field: points must look like -100:0|0:511|100:1012");
  }
  else if(f.map == "table") {
    f.table_file = kv["table"];
    if(f.table_file.empty())
      addError(lineno, "tx_field: map=table needs table=<file>");
    else
      loadTable(f, lineno);
  }
  else if(f.map == "enum") {
    vector<string> items = split(kv["values"], '|');
    for(size_t i=0; i<items.size(); i++) {
      size_t c = items[i].find(':');
      unsigned long long raw;
      if(c == string::npos || !parseHexInt(trim(items[i].substr(c + 1)), raw)) {
        addError(lineno, "tx_field: values must look like fwd:7E|neutral:7D|rev:7C");
        break;
      }
      f.enum_values[lower(trim(items[i].substr(0, c)))] = (long long)raw;
    }
  }
  else
    addError(lineno, "tx_field: map must be linear, points, table or enum");

  const char* known[] = {"msg","src","byte","bits","order","signed","map",
                         "scale","offset","points","table","values"};
  for(map<string,string>::iterator p=kv.begin(); p!=kv.end(); p++)
    if(find(known, known + 12, p->first) == known + 12)
      addError(lineno, "tx_field: unknown parameter '" + p->first + "'");

  msg->fields.push_back(f);
}

bool Profile::loadTable(TxField& f, int lineno)
{
  string path = f.table_file;
  if(path[0] != '/')
    path = m_base_dir + "/" + path;
  string text;
  if(!readFile(path, text)) {
    addError(lineno, "cannot read table " + path);
    return false;
  }
  m_hash_input += "\n" + text;

  istringstream in(text);
  string line;
  int row = 0;
  while(getline(in, line)) {
    row++;
    size_t c = line.find('#');
    if(c != string::npos)
      line = line.substr(0, c);
    line = trim(line);
    if(line.empty())
      continue;
    vector<string> cols = split(line, ',');
    long long key, raw;
    if(cols.size() != 2 || !parseInt(cols[0], key) || !parseInt(cols[1], raw)) {
      addError(lineno, f.table_file + " row " + to_string(row) +
               ": expected 'input,raw' integers");
      return false;
    }
    if(f.table.count((int)key))
      addError(lineno, f.table_file + " row " + to_string(row) + ": duplicate input");
    f.table[(int)key] = raw;
  }
  return true;
}

void Profile::parseTxConst(const string& val, int lineno)
{
  map<string, string> kv = keyValues(val);
  TxMsg* msg = findMsg(kv["msg"]);
  if(!msg) {
    addError(lineno, "tx_const: no tx_msg named '" + kv["msg"] + "'");
    return;
  }
  TxConst c;
  long long n;
  if(!parseInt(kv["byte"], n) || n < 0 || n > 7) {
    addError(lineno, "tx_const: byte must be 0..7");
    return;
  }
  c.byte = (int)n;
  if(!parseHexBytes(kv["value"], c.bytes)) {
    addError(lineno, "tx_const: value must be hex bytes, e.g. C1 or 1AFD");
    return;
  }
  msg->consts.push_back(c);
}

void Profile::parseRxField(const string& val, int lineno)
{
  map<string, string> kv = keyValues(val);
  RxField r;
  r.name = kv["name"];
  if(r.name.empty())
    addError(lineno, "rx_field needs name=");

  unsigned long long id;
  if(!parseHexInt(kv["id"], id) || id > 0x1FFFFFFF)
    addError(lineno, "rx_field " + r.name + ": bad id");
  else
    r.id = (uint32_t)id;
  r.ext = (r.id > 0x7FF);
  if(kv.count("ext") && !parseBool(kv["ext"], r.ext))
    addError(lineno, "rx_field " + r.name + ": ext must be true or false");

  // parts=B3:0-7|B6:2-3  or  parts=B4
  vector<string> parts = split(kv["parts"], '|');
  int total = 0;
  for(size_t i=0; i<parts.size(); i++) {
    string p = parts[i];
    RxPart part;
    part.lo_bit = 0;
    part.hi_bit = 7;
    if(p.size() < 2 || (p[0] != 'B' && p[0] != 'b')) {
      addError(lineno, "rx_field " + r.name + ": parts must look like B3:0-7|B6:2-3");
      return;
    }
    size_t c = p.find(':');
    long long b;
    if(!parseInt(p.substr(1, c == string::npos ? string::npos : c - 1), b) || b < 0 || b > 7) {
      addError(lineno, "rx_field " + r.name + ": byte index must be 0..7");
      return;
    }
    part.byte = (int)b;
    if(c != string::npos) {
      string bits = p.substr(c + 1);
      size_t d = bits.find('-');
      long long lo, hi;
      if(d == string::npos || !parseInt(bits.substr(0, d), lo) ||
         !parseInt(bits.substr(d + 1), hi) || lo < 0 || hi > 7 || lo > hi) {
        addError(lineno, "rx_field " + r.name + ": bit range must look like 2-3");
        return;
      }
      part.lo_bit = (int)lo;
      part.hi_bit = (int)hi;
    }
    total += part.hi_bit - part.lo_bit + 1;
    r.parts.push_back(part);
  }
  if(total > 32)
    addError(lineno, "rx_field " + r.name + ": more than 32 bits");

  if(kv.count("signed") && !parseBool(kv["signed"], r.is_signed))
    addError(lineno, "rx_field " + r.name + ": signed must be true or false");

  r.map = kv.count("map") ? lower(kv["map"]) : "raw";
  if(r.map == "linear") {
    if(kv.count("scale") && !parseDouble(kv["scale"], r.scale))
      addError(lineno, "rx_field " + r.name + ": bad scale");
    if(kv.count("offset") && !parseDouble(kv["offset"], r.offset))
      addError(lineno, "rx_field " + r.name + ": bad offset");
  }
  else if(r.map == "points") {
    if(!parsePoints(kv["points"], r.points))
      addError(lineno, "rx_field " + r.name + ": bad points");
    else
      for(size_t i=1; i<r.points.size(); i++)
        if(r.points[i].x <= r.points[i-1].x)
          addError(lineno, "rx_field " + r.name + ": points must increase");
  }
  else if(r.map == "enum") {
    vector<string> items = split(kv["values"], '|');
    for(size_t i=0; i<items.size(); i++) {
      size_t c = items[i].find(':');
      unsigned long long raw;
      if(c == string::npos || !parseHexInt(trim(items[i].substr(0, c)), raw)) {
        addError(lineno, "rx_field " + r.name + ": values must look like 7E:fwd|7D:neutral");
        break;
      }
      r.enum_values[(long long)raw] = trim(items[i].substr(c + 1));
    }
  }
  else if(r.map != "raw")
    addError(lineno, "rx_field " + r.name + ": map must be raw, linear, points or enum");

  m_rx.push_back(r);
}

//---------------------------------------------------------
// Validation: anything wrong here leaves the boat inhibited

void Profile::validate()
{
  if(m_name.empty())
    addError(0, "profile = <name> is missing");
  if(m_can_interface.empty())
    addError(0, "can_interface is missing");

  double rlim = setting("rudder_limit_pct");
  double tlim = setting("thrust_limit_pct");
  if(rlim < 0 || rlim > 100)
    addError(0, "rudder_limit_pct must be 0..100");
  if(tlim < 0 || tlim > 100)
    addError(0, "thrust_limit_pct must be 0..100");
  if(fabs(setting("lost_link_rudder_pct")) > rlim)
    addError(0, "lost_link_rudder_pct is outside rudder_limit_pct");
  if(fabs(setting("lost_link_thrust_pct")) > tlim)
    addError(0, "lost_link_thrust_pct is outside thrust_limit_pct");
  const char* nonneg[] = {"thrust_deadband_pct","shift_neutral_secs","shift_settle_secs",
                          "rudder_rate_pct_s","thrust_rate_pct_s","guard_secs",
                          "rudder_alarm_pct","rudder_alarm_secs","thrust_rate_up_pct_s"};
  for(int i=0; i<9; i++)
    if(setting(nonneg[i]) < 0)
      addError(0, string(nonneg[i]) + " must not be negative");

  // Trim motors run as long as up/down is sent; a pulse is always short
  const char* trim_keys[] = {"trim_up_secs", "trim_down_secs"};
  for(int i=0; i<2; i++)
    if(setting(trim_keys[i]) <= 0 || setting(trim_keys[i]) > 5.0)
      addError(0, string(trim_keys[i]) + " must be more than 0 and at most 5 seconds");

  for(size_t i=0; i<m_caps.size(); i++) {
    bool found = false;
    for(size_t j=0; j<m_tx.size(); j++)
      if(m_tx[j].cap == m_caps[i])
        found = true;
    if(!found)
      addError(0, "capability " + m_caps[i] + " is enabled but no tx_msg has cap=" + m_caps[i]);
  }

  for(size_t i=0; i<m_tx.size(); i++) {
    const TxMsg& msg = m_tx[i];
    for(size_t j=i+1; j<m_tx.size(); j++)
      if(m_tx[j].id == msg.id && m_tx[j].ext == msg.ext)
        addError(0, "tx_msg " + msg.name + " and " + m_tx[j].name + " use the same id");

    if(msg.fields.empty())
      addError(0, "tx_msg " + msg.name + " has no tx_field");

    // Each byte may be written by one field or constant only
    vector<int> owner(8, 0);
    for(size_t k=0; k<msg.fields.size(); k++) {
      const TxField& f = msg.fields[k];
      validateField(msg, f);
      for(int b=f.byte; b<f.byte + f.bits/8 && b<8; b++)
        if(owner[b]++)
          addError(0, "tx_msg " + msg.name + ": byte " + to_string(b) + " is written twice");
    }
    for(size_t k=0; k<msg.consts.size(); k++) {
      const TxConst& c = msg.consts[k];
      if(c.byte + (int)c.bytes.size() > msg.dlc)
        addError(0, "tx_msg " + msg.name + ": tx_const at byte " + to_string(c.byte) +
                 " runs past dlc");
      for(int b=c.byte; b<c.byte + (int)c.bytes.size() && b<8; b++)
        if(owner[b]++)
          addError(0, "tx_msg " + msg.name + ": byte " + to_string(b) + " is written twice");
    }
  }

  for(size_t i=0; i<m_rx.size(); i++)
    for(size_t j=i+1; j<m_rx.size(); j++)
      if(m_rx[i].name == m_rx[j].name)
        addError(0, "rx_field name " + m_rx[i].name + " is used twice");

  // Speed-dependent rudder limit
  string err;
  vector<SpeedPoint> rls;
  if(!stringSetting("rudder_limit_speed").empty() &&
     !AutoTuning::parseSpeedPct(stringSetting("rudder_limit_speed"), "rudder_limit_speed",
                                rls, err))
    addError(0, err);
  if(setting("speed_stale_secs") <= 0)
    addError(0, "speed_stale_secs must be more than 0");

  // Autopilot tuning
  vector<SpeedPoint> ff;
  vector<GainStep> gains;
  if(!AutoTuning::parseSpeedFF(stringSetting("auto_speed_ff"), ff, err))
    addError(0, err);
  if(!AutoTuning::parseGains(stringSetting("auto_speed_gains"), gains, err))
    addError(0, err);
  if(!AutoTuning::parseHeadingGains(stringSetting("auto_heading_gains"), gains, err))
    addError(0, err);
  const char* auto_pct[] = {"auto_heading_limit_pct", "auto_rudder_max_pct",
                            "auto_rudder_min_pct", "auto_thrust_max_pct",
                            "auto_speed_i_authority_pct", "auto_heading_i_authority_pct"};
  for(int i=0; i<6; i++)
    if(setting(auto_pct[i]) < 0 || setting(auto_pct[i]) > 100)
      addError(0, string(auto_pct[i]) + " must be 0..100");
  const char* filt[] = {"auto_speed_filter", "auto_heading_filter"};
  for(int i=0; i<2; i++)
    if(setting(filt[i]) <= 0 || setting(filt[i]) > 1)
      addError(0, string(filt[i]) + " must be more than 0 and at most 1");
  if(setting("auto_rudder_min_pct") > setting("auto_rudder_max_pct"))
    addError(0, "auto_rudder_min_pct is above auto_rudder_max_pct");
  if(setting("auto_rudder_speed_low") >= setting("auto_rudder_speed_high"))
    addError(0, "auto_rudder_speed_low must be below auto_rudder_speed_high");

  string fb = stringSetting("rudder_feedback");
  if(!fb.empty()) {
    bool found = false;
    for(size_t i=0; i<m_rx.size(); i++)
      if(m_rx[i].name == fb)
        found = true;
    if(!found)
      addError(0, "rudder_feedback names " + fb + " but no rx_field has that name");
  }
}

void Profile::validateField(const TxMsg& msg, const TxField& f)
{
  string where = "tx_msg " + msg.name + " field " + f.src + ": ";

  if(f.byte + f.bits/8 > msg.dlc)
    addError(0, where + "runs past dlc " + to_string(msg.dlc));
  if(!f.src.empty() && capOfSrc(f.src) != msg.cap)
    addError(0, where + "src " + f.src + " does not belong in a cap=" + msg.cap + " message");

  long long lo, hi;
  rawRange(f.bits, f.is_signed, lo, hi);
  vector<long long> raws;   // every raw value this field can produce

  double dlo, dhi;
  srcDomain(f.src, dlo, dhi);

  if(f.map == "enum") {
    if(!srcIsSymbolic(f.src))
      addError(0, where + "map=enum needs src gear, rudder_side or trim");
    vector<string> states = srcStates(f.src);
    for(size_t i=0; i<states.size(); i++) {
      map<string, long long>::const_iterator p = f.enum_values.find(states[i]);
      if(p == f.enum_values.end())
        addError(0, where + "enum has no value for '" + states[i] + "'");
      else
        raws.push_back(p->second);
    }
  }
  else {
    if(!srcIsNumeric(f.src)) {
      addError(0, where + "src " + f.src + " needs map=enum");
      return;
    }
    if(f.map == "linear") {
      raws.push_back(llround(dlo * f.scale + f.offset));
      raws.push_back(llround(dhi * f.scale + f.offset));
    }
    else if(f.map == "points") {
      for(size_t i=1; i<f.points.size(); i++)
        if(f.points[i].x <= f.points[i-1].x)
          addError(0, where + "points must have increasing inputs");
      if(!f.points.empty() && (f.points.front().x > dlo || f.points.back().x < dhi))
        addError(0, where + "points must cover " + to_string((int)dlo) + ".." +
                 to_string((int)dhi));
      for(size_t i=0; i<f.points.size(); i++)
        raws.push_back(llround(f.points[i].y));
    }
    else if(f.map == "table") {
      for(int k=(int)dlo; k<=(int)dhi; k++) {
        map<int, long long>::const_iterator p = f.table.find(k);
        if(p == f.table.end()) {
          addError(0, where + "table " + f.table_file + " has no row for " + to_string(k));
          break;
        }
        raws.push_back(p->second);
      }
    }
  }

  for(size_t i=0; i<raws.size(); i++)
    if(raws[i] < lo || raws[i] > hi) {
      addError(0, where + "value " + to_string(raws[i]) + " does not fit in " +
               to_string(f.bits) + (f.is_signed ? " signed" : " unsigned") + " bits");
      break;
    }
}

//---------------------------------------------------------
// Encoding

long long Profile::fieldRaw(const TxField& f, const DriveCommand& cmd) const
{
  if(f.map == "enum") {
    string state;
    if(f.src == "gear")
      state = gearName(cmd.gear);
    else if(f.src == "trim")
      state = trimName(cmd.trim);
    else {
      long t = (long)cmd.rudder_pct;   // truncate toward zero, as the legacy tables did
      state = (t < 0) ? "port" : ((t > 0) ? "stbd" : "center");
    }
    map<string, long long>::const_iterator p = f.enum_values.find(state);
    return (p == f.enum_values.end()) ? 0 : p->second;
  }

  double v = 0;
  if(f.src == "rudder_pct")          v = cmd.rudder_pct;
  else if(f.src == "thrust_pct")     v = cmd.thrust_pct;
  else if(f.src == "thrust_abs_pct") v = fabs(cmd.thrust_pct);
  if(std::isnan(v))
    v = 0;

  double lo, hi;
  srcDomain(f.src, lo, hi);
  if(v < lo) v = lo;
  if(v > hi) v = hi;

  if(f.map == "linear")
    return llround(v * f.scale + f.offset);
  if(f.map == "points")
    return llround(interp(f.points, v));
  if(f.map == "table") {
    map<int, long long>::const_iterator p = f.table.find((int)v);  // truncates
    return (p == f.table.end()) ? 0 : p->second;
  }
  return 0;
}

Frame Profile::encode(const TxMsg& msg, const DriveCommand& cmd) const
{
  Frame fr;
  fr.id  = msg.id;
  fr.ext = msg.ext;
  fr.dlc = (uint8_t)msg.dlc;
  for(int i=0; i<8; i++)
    fr.data[i] = (i < msg.dlc) ? msg.fill : 0;

  for(size_t i=0; i<msg.consts.size(); i++) {
    const TxConst& c = msg.consts[i];
    for(size_t k=0; k<c.bytes.size() && c.byte + (int)k < msg.dlc; k++)
      fr.data[c.byte + k] = c.bytes[k];
  }

  for(size_t i=0; i<msg.fields.size(); i++) {
    const TxField& f = msg.fields[i];
    long long raw = fieldRaw(f, cmd);
    long long lo, hi;
    rawRange(f.bits, f.is_signed, lo, hi);
    if(raw < lo) raw = lo;
    if(raw > hi) raw = hi;
    uint64_t u = (uint64_t)raw;   // two's complement for negative values
    int n = f.bits / 8;
    for(int k=0; k<n; k++) {
      uint8_t b = (uint8_t)((u >> (8 * k)) & 0xFF);
      int pos = f.big_endian ? (f.byte + n - 1 - k) : (f.byte + k);
      if(pos < msg.dlc)
        fr.data[pos] = b;
    }
  }
  return fr;
}

//---------------------------------------------------------
// Decoding

vector<RxValue> Profile::decode(uint32_t id, bool ext, const uint8_t* data, int dlc) const
{
  vector<RxValue> out;
  for(size_t i=0; i<m_rx.size(); i++) {
    const RxField& r = m_rx[i];
    if(r.id != id || r.ext != ext)
      continue;

    bool fits = true;
    uint64_t raw = 0;
    int shift = 0;
    for(size_t k=0; k<r.parts.size(); k++) {
      const RxPart& p = r.parts[k];
      if(p.byte >= dlc) {
        fits = false;
        break;
      }
      int n = p.hi_bit - p.lo_bit + 1;
      uint64_t bits = (data[p.byte] >> p.lo_bit) & ((1u << n) - 1);
      raw |= bits << shift;
      shift += n;
    }
    if(!fits)
      continue;

    long long sraw = (long long)raw;
    if(r.is_signed && shift > 0 && shift < 64 && (raw & (1ULL << (shift - 1))))
      sraw = (long long)(raw | (~0ULL << shift));

    RxValue v;
    v.name = r.name;
    v.is_string = false;
    v.dval = (double)sraw;
    if(r.map == "linear")
      v.dval = sraw * r.scale + r.offset;
    else if(r.map == "points")
      v.dval = interp(r.points, (double)sraw);
    else if(r.map == "enum") {
      v.is_string = true;
      map<long long, string>::const_iterator p = r.enum_values.find(sraw);
      v.sval = (p == r.enum_values.end()) ? "unknown" : p->second;
    }
    out.push_back(v);
  }
  return out;
}

bool Profile::hasTrimIdle(const TxMsg& msg) const
{
  for(size_t i=0; i<msg.fields.size(); i++)
    if(msg.fields[i].src == "trim" && msg.fields[i].enum_values.count("idle"))
      return true;
  return false;
}

bool Profile::isTxId(uint32_t id, bool ext, string& msg_name) const
{
  for(size_t i=0; i<m_tx.size(); i++)
    if(m_tx[i].id == id && m_tx[i].ext == ext) {
      msg_name = m_tx[i].name;
      return true;
    }
  return false;
}

}  // namespace canprofile
