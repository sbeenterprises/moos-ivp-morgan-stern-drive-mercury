/************************************************************/
/*    NAME: DL                                              */
/*    FILE: AisEncode.cpp                                   */
/*    DATE: 2026                                            */
/************************************************************/

#include "AisEncode.h"
#include <cmath>
#include <cstdio>

using namespace std;

namespace {

/** Bit accumulator, MSB first, one AIS six-bit symbol per slot. */
class BitBuffer
{
 public:
  BitBuffer(size_t nbits) : m_bits(nbits, 0) {}

  /** Write `len` bits of `value` at `start`, most significant bit first.
   *  Negative values are written two's complement, which is what AIS wants
   *  for latitude, longitude and rate of turn. */
  void putInt(long value, size_t start, size_t len)
  {
    for(size_t i = 0; i < len; i++) {
      // bit (len-1-i) of value lands at start+i
      long bit = (value >> (len - 1 - i)) & 1;
      if((start + i) < m_bits.size())
        m_bits[start + i] = static_cast<unsigned char>(bit);
    }
  }

  /** Write a string as AIS six-bit characters. The field is `len` bits, i.e.
   *  len/6 characters; short strings are padded with '@' (zero), which is how
   *  AIS marks unused character positions. */
  void putStr(const string& text, size_t start, size_t len)
  {
    size_t nchars = len / 6;
    for(size_t i = 0; i < nchars; i++) {
      unsigned char sixbit = 0;
      if(i < text.size()) {
        unsigned char c = static_cast<unsigned char>(text[i]);
        if((c >= 'a') && (c <= 'z'))
          c = static_cast<unsigned char>(c - 'a' + 'A');
        // The AIS six-bit alphabet is ASCII 32..95 truncated to six bits.
        // Anything outside that range has no representation - use a space.
        if((c < 32) || (c > 95))
          c = ' ';
        sixbit = static_cast<unsigned char>(c & 0x3F);
      }
      putInt(sixbit, start + (i * 6), 6);
    }
  }

  /** Six-bit ASCII armouring. `pad` receives the number of fill bits added to
   *  reach a six-bit boundary, which the sentence has to declare. */
  string armour(int& pad) const
  {
    size_t nsym = (m_bits.size() + 5) / 6;
    pad = static_cast<int>((nsym * 6) - m_bits.size());

    string out;
    out.reserve(nsym);
    for(size_t s = 0; s < nsym; s++) {
      unsigned char v = 0;
      for(size_t b = 0; b < 6; b++) {
        size_t idx = (s * 6) + b;
        unsigned char bit = (idx < m_bits.size()) ? m_bits[idx] : 0;
        v = static_cast<unsigned char>((v << 1) | bit);
      }
      out += static_cast<char>((v < 40) ? (v + 48) : (v + 56));
    }
    return out;
  }

 private:
  vector<unsigned char> m_bits;
};

/** NMEA 0183 checksum: XOR of everything between the '!' and the '*'. */
string nmeaChecksum(const string& body)
{
  unsigned char cs = 0;
  for(size_t i = 0; i < body.size(); i++)
    cs = static_cast<unsigned char>(cs ^ static_cast<unsigned char>(body[i]));

  char buf[8];
  snprintf(buf, sizeof(buf), "*%02X", cs);
  return string(buf);
}

/** Wrap one armoured payload in an !AIVDM sentence. */
string buildSentence(int total, int index, const string& seq,
                     const string& payload, int pad)
{
  char head[32];
  snprintf(head, sizeof(head), "AIVDM,%d,%d,%s,A,", total, index, seq.c_str());

  char tail[8];
  snprintf(tail, sizeof(tail), ",%d", pad);

  string body = string(head) + payload + string(tail);
  return "!" + body + nmeaChecksum(body);
}

/** AIS carries lat/lon in 1/10000 of a minute. */
long toAisLatLon(double degrees)
{
  return static_cast<long>(llround(degrees * 600000.0));
}

} // anonymous namespace

//---------------------------------------------------------
// Procedure: encodeAisPosition()

string encodeAisPosition(const AisPosition& p)
{
  BitBuffer bits(168);

  long sog = static_cast<long>(llround(p.sog * 10.0));
  if(sog < 0)    sog = 0;
  if(sog > 1022) sog = 1023;          // 1023 = not available

  double cog_deg = fmod(p.cog, 360.0);
  if(cog_deg < 0) cog_deg += 360.0;
  long cog = static_cast<long>(llround(cog_deg * 10.0));
  if(cog > 3599) cog = 3600;          // 3600 = not available

  long hdg = static_cast<long>(llround(p.hdg));
  if((hdg < 0) || (hdg > 359)) hdg = 511;  // 511 = not available

  bits.putInt(1,               0,   6);   // message type
  bits.putInt(0,               6,   2);   // repeat indicator
  bits.putInt(p.mmsi,          8,  30);
  bits.putInt(p.navstatus,    38,   4);
  bits.putInt(-128,           42,   8);   // rate of turn: not available
  bits.putInt(sog,            50,  10);
  bits.putInt(1,              60,   1);   // position accuracy: DGPS
  bits.putInt(toAisLatLon(p.lon), 61, 28);
  bits.putInt(toAisLatLon(p.lat), 89, 27);
  bits.putInt(cog,           116,  12);
  bits.putInt(hdg,           128,   9);
  bits.putInt(60,            137,   6);   // UTC second: not available
  bits.putInt(0,             143,   2);   // special manoeuvre: not available
  bits.putInt(0,             145,   3);   // spare
  bits.putInt(0,             148,   1);   // RAIM
  bits.putInt(0,             149,  19);   // radio status

  int pad = 0;
  string payload = bits.armour(pad);
  return buildSentence(1, 1, "", payload, pad);
}

//---------------------------------------------------------
// Procedure: encodeAisStatic()

vector<string> encodeAisStatic(const AisStatic& s, int seq_id)
{
  BitBuffer bits(424);

  long draught = static_cast<long>(llround(s.draught * 10.0));
  if(draught < 0)   draught = 0;
  if(draught > 255) draught = 255;

  bits.putInt(5,             0,   6);   // message type
  bits.putInt(0,             6,   2);   // repeat indicator
  bits.putInt(s.mmsi,        8,  30);
  bits.putInt(0,            38,   2);   // AIS version: ITU-R M.1371-1
  bits.putInt(s.imo,        40,  30);
  bits.putStr(s.callsign,   70,  42);   // 7 characters
  bits.putStr(s.shipname,  112, 120);   // 20 characters
  bits.putInt(s.shiptype,  232,   8);
  bits.putInt(s.dim_a,     240,   9);
  bits.putInt(s.dim_b,     249,   9);
  bits.putInt(s.dim_c,     258,   6);
  bits.putInt(s.dim_d,     264,   6);
  bits.putInt(1,           270,   4);   // EPFD type: GPS
  bits.putInt(0,           274,   4);   // ETA month: not available
  bits.putInt(0,           278,   5);   // ETA day
  bits.putInt(24,          283,   5);   // ETA hour: not available
  bits.putInt(60,          288,   6);   // ETA minute: not available
  bits.putInt(draught,     294,   8);
  bits.putStr(s.destination, 302, 120); // 20 characters
  bits.putInt(0,           422,   1);   // DTE
  bits.putInt(0,           423,   1);   // spare

  int pad = 0;
  string payload = bits.armour(pad);

  // 424 bits armour to 71 symbols, well past what one sentence may carry, so
  // the message goes out as two fragments sharing a sequence id. Only the last
  // fragment declares the pad, because only it ends mid-symbol.
  const size_t split = 60;
  char seq[4];
  snprintf(seq, sizeof(seq), "%d", seq_id % 10);

  vector<string> out;
  out.push_back(buildSentence(2, 1, seq, payload.substr(0, split), 0));
  out.push_back(buildSentence(2, 2, seq, payload.substr(split), pad));
  return out;
}
