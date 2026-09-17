/************************************************************/
/*    NAME: DL                                              */
/*    FILE: AisDecode.cpp                                   */
/*    DATE: 2026                                            */
/************************************************************/

#include "AisDecode.h"
#include <cstdio>
#include <cstdlib>

using namespace std;

namespace {

/** Bit reader over a de-armoured AIS payload, MSB first - the exact inverse of
 *  the BitBuffer in src/pTrafficSim/AisEncode.cpp. */
class BitReader
{
 public:
  BitReader(const vector<unsigned char>& bits) : m_bits(bits) {}

  size_t size() const { return m_bits.size(); }

  /** Unsigned field, most significant bit first. Reads past the end return 0,
   *  which is how a truncated sentence degrades rather than crashes. */
  unsigned long getUInt(size_t start, size_t len) const
  {
    unsigned long value = 0;
    for(size_t i = 0; i < len; i++) {
      size_t idx = start + i;
      unsigned long bit = (idx < m_bits.size()) ? m_bits[idx] : 0;
      value = (value << 1) | bit;
    }
    return value;
  }

  /** Two's complement field - latitude, longitude and rate of turn. */
  long getInt(size_t start, size_t len) const
  {
    unsigned long raw = getUInt(start, len);
    // Sign extend from bit len-1.
    unsigned long sign = 1UL << (len - 1);
    if(raw & sign)
      return static_cast<long>(raw) - static_cast<long>(1UL << len);
    return static_cast<long>(raw);
  }

  /** AIS six-bit ASCII. Trailing '@' (the AIS pad character) and trailing
   *  spaces are stripped, which is what turns "TUG ALPHA@@@@@@@@@@@" back into
   *  "TUG ALPHA". */
  string getStr(size_t start, size_t len) const
  {
    string out;
    size_t nchars = len / 6;
    out.reserve(nchars);

    for(size_t i = 0; i < nchars; i++) {
      unsigned char v = static_cast<unsigned char>(getUInt(start + (i * 6), 6));
      // Six-bit values 0..31 map to ASCII 64..95, 32..63 map to themselves.
      char c = static_cast<char>((v < 32) ? (v + 64) : v);
      if(c == '@')
        break;
      out += c;
    }

    while(!out.empty() && (out[out.size() - 1] == ' '))
      out.erase(out.size() - 1);
    return out;
  }

 private:
  const vector<unsigned char>& m_bits;
};

/** Undo the six-bit ASCII armouring. `pad` is the number of fill bits the
 *  sentence declared, which are dropped from the tail. */
vector<unsigned char> dearmour(const string& payload, int pad)
{
  vector<unsigned char> bits;
  bits.reserve(payload.size() * 6);

  for(size_t i = 0; i < payload.size(); i++) {
    int v = static_cast<unsigned char>(payload[i]) - 48;
    if(v > 40)
      v -= 8;
    if((v < 0) || (v > 63))
      return vector<unsigned char>();   // not a valid armoured character

    for(int b = 5; b >= 0; b--)
      bits.push_back(static_cast<unsigned char>((v >> b) & 1));
  }

  if((pad > 0) && (static_cast<size_t>(pad) <= bits.size()))
    bits.resize(bits.size() - pad);

  return bits;
}

/** Split on commas, keeping empty fields - "!AIVDM,1,1,,A,..." has one. */
vector<string> splitFields(const string& str)
{
  vector<string> out;
  string current;
  for(size_t i = 0; i < str.size(); i++) {
    if(str[i] == ',') {
      out.push_back(current);
      current.clear();
    }
    else
      current += str[i];
  }
  out.push_back(current);
  return out;
}

/** NMEA 0183 checksum: XOR of everything between the '!' and the '*'. */
bool checksumOk(const string& sentence)
{
  size_t star = sentence.rfind('*');
  if((star == string::npos) || (star + 2 >= sentence.size()))
    return false;   // no checksum at all - reject rather than guess

  unsigned char cs = 0;
  for(size_t i = 1; i < star; i++)
    cs = static_cast<unsigned char>(cs ^ static_cast<unsigned char>(sentence[i]));

  unsigned int declared = 0;
  if(sscanf(sentence.c_str() + star + 1, "%2x", &declared) != 1)
    return false;

  return cs == static_cast<unsigned char>(declared);
}

// AIS marks "position not available" with 181 degrees of longitude and 91
// degrees of latitude, in units of 1/10000 minute.
const long AIS_LON_NA = 181L * 600000L;
const long AIS_LAT_NA =  91L * 600000L;

} // anonymous namespace

//---------------------------------------------------------
// Procedure: handleSentence()

bool AisDecoder::handleSentence(const string& sentence, AisReport& out,
                                double now)
{
  // Tolerate the trailing CR/LF a serial or UDP source may leave on.
  string line = sentence;
  while(!line.empty() &&
        ((line[line.size() - 1] == '\r') || (line[line.size() - 1] == '\n') ||
         (line[line.size() - 1] == '\0')))
    line.erase(line.size() - 1);

  if(line.size() < 10)
    return false;
  if((line.compare(0, 6, "!AIVDM") != 0) && (line.compare(0, 6, "!AIVDO") != 0))
    return false;

  m_sentences_seen++;

  if(!checksumOk(line)) {
    m_bad_sentences++;
    return false;
  }

  // !AIVDM,<total>,<index>,<seq>,<channel>,<payload>,<pad>*<cs>
  vector<string> f = splitFields(line);
  if(f.size() < 7) {
    m_bad_sentences++;
    return false;
  }

  int total = atoi(f[1].c_str());
  int index = atoi(f[2].c_str());
  const string& seq     = f[3];
  const string& channel = f[4];
  const string& payload = f[5];

  // The pad digit sits just before the '*'.
  string pad_field = f[6];
  size_t star = pad_field.find('*');
  if(star != string::npos)
    pad_field = pad_field.substr(0, star);
  int pad = atoi(pad_field.c_str());

  if((total < 1) || (index < 1) || (index > total) || payload.empty()) {
    m_bad_sentences++;
    return false;
  }

  if(total == 1)
    return decodePayload(payload, pad, out);

  // Multipart. Fragments of different messages interleave on the wire, so the
  // buffer is keyed on the sequence id plus the channel they arrived on.
  string key = seq + "/" + channel;
  Fragments& frag = m_pending[key];

  if(frag.total != total) {          // new message, or a resynchronisation
    frag.total = total;
    frag.parts.assign(total, string());
    frag.first_seen = now;
    frag.pad = 0;
  }

  frag.parts[index - 1] = payload;
  if(index == total)
    frag.pad = pad;                  // only the last fragment ends mid symbol

  for(int i = 0; i < total; i++) {
    if(frag.parts[i].empty())
      return false;                  // still waiting on a fragment
  }

  string joined;
  for(int i = 0; i < total; i++)
    joined += frag.parts[i];
  int joined_pad = frag.pad;
  m_pending.erase(key);

  return decodePayload(joined, joined_pad, out);
}

//---------------------------------------------------------
// Procedure: decodePayload()

bool AisDecoder::decodePayload(const string& payload, int pad, AisReport& out)
{
  vector<unsigned char> bits = dearmour(payload, pad);
  if(bits.size() < 38) {             // not even a type and an MMSI
    m_bad_sentences++;
    return false;
  }

  BitReader r(bits);

  out = AisReport();
  out.msg_type = static_cast<int>(r.getUInt(0, 6));
  out.mmsi     = static_cast<unsigned int>(r.getUInt(8, 30));

  if((out.msg_type == 1) || (out.msg_type == 2) || (out.msg_type == 3)) {
    if(bits.size() < 168) { m_bad_sentences++; return false; }

    out.navstatus = static_cast<int>(r.getUInt(38, 4));

    unsigned long sog = r.getUInt(50, 10);
    out.sog = (sog == 1023) ? -1.0 : (sog / 10.0);

    long lon = r.getInt(61, 28);
    long lat = r.getInt(89, 27);
    if((lon != AIS_LON_NA) && (lat != AIS_LAT_NA)) {
      out.lon = lon / 600000.0;
      out.lat = lat / 600000.0;
      out.has_position = true;
    }

    unsigned long cog = r.getUInt(116, 12);
    out.cog = (cog >= 3600) ? -1.0 : (cog / 10.0);

    out.hdg = static_cast<double>(r.getUInt(128, 9));
  }
  else if(out.msg_type == 18) {      // class B position report
    if(bits.size() < 168) { m_bad_sentences++; return false; }

    unsigned long sog = r.getUInt(46, 10);
    out.sog = (sog == 1023) ? -1.0 : (sog / 10.0);

    long lon = r.getInt(57, 28);
    long lat = r.getInt(85, 27);
    if((lon != AIS_LON_NA) && (lat != AIS_LAT_NA)) {
      out.lon = lon / 600000.0;
      out.lat = lat / 600000.0;
      out.has_position = true;
    }

    unsigned long cog = r.getUInt(112, 12);
    out.cog = (cog >= 3600) ? -1.0 : (cog / 10.0);

    out.hdg = static_cast<double>(r.getUInt(124, 9));
  }
  else if(out.msg_type == 5) {       // class A static and voyage data
    if(bits.size() < 424 - 40) { m_bad_sentences++; return false; }

    out.callsign = r.getStr(70, 42);
    out.shipname = r.getStr(112, 120);
    out.shiptype = static_cast<int>(r.getUInt(232, 8));
    out.dim_a    = static_cast<int>(r.getUInt(240, 9));
    out.dim_b    = static_cast<int>(r.getUInt(249, 9));
    out.dim_c    = static_cast<int>(r.getUInt(258, 6));
    out.dim_d    = static_cast<int>(r.getUInt(264, 6));
    out.has_static = true;
  }
  else if(out.msg_type == 24) {      // class B static, split across two parts
    int part = static_cast<int>(r.getUInt(38, 2));
    if(part == 0) {
      out.shipname = r.getStr(40, 120);
      out.has_static = true;
    }
    else if(part == 1) {
      out.shiptype = static_cast<int>(r.getUInt(40, 8));
      out.callsign = r.getStr(90, 42);
      out.dim_a    = static_cast<int>(r.getUInt(132, 9));
      out.dim_b    = static_cast<int>(r.getUInt(141, 9));
      out.dim_c    = static_cast<int>(r.getUInt(150, 6));
      out.dim_d    = static_cast<int>(r.getUInt(156, 6));
      out.has_static = true;
    }
    else
      return false;
  }
  else
    return false;                    // a type we do not model - not an error

  if(out.mmsi == 0)
    return false;

  m_messages_decoded++;
  return true;
}

//---------------------------------------------------------
// Procedure: expireFragments()

void AisDecoder::expireFragments(double now, double max_age)
{
  map<string, Fragments>::iterator p = m_pending.begin();
  while(p != m_pending.end()) {
    if((now - p->second.first_seen) > max_age)
      m_pending.erase(p++);
    else
      ++p;
  }
}
