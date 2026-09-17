/************************************************************/
/*    NAME: DL                                              */
/*    FILE: AisDecode.h                                     */
/*    DATE: 2026                                            */
/*                                                          */
/*  Minimal AIVDM decoder, the mirror image of              */
/*  src/pTrafficSim/AisEncode.{cpp,h}: same bit layouts,    */
/*  read instead of written.                                */
/*                                                          */
/*  Covers what a contact picture needs - message types     */
/*  1/2/3 and 18 (position) and 5 and 24 (name, callsign,   */
/*  dimensions) - including the multipart reassembly that   */
/*  types 5 and 24 require. No external dependency, so the  */
/*  vendored libais copy is left untouched.                 */
/************************************************************/

#ifndef AisDecode_HEADER
#define AisDecode_HEADER

#include <map>
#include <string>
#include <vector>

/** One decoded AIS message. Position and static fields are carried in the same
 *  struct because a single MMSI is described by both over time; the has_*
 *  flags say which half of it this particular message actually filled in. */
struct AisReport {
  unsigned int mmsi;
  int          msg_type;

  bool         has_position;
  double       lat, lon;      // degrees, +N / +E
  double       sog;           // knots     (<0 = not available)
  double       cog;           // degrees   (<0 = not available)
  double       hdg;           // degrees   (511 = not available)
  int          navstatus;

  bool         has_static;
  std::string  shipname;
  std::string  callsign;
  int          shiptype;      // ITU ship-and-cargo code
  int          dim_a, dim_b;  // metres bow / stern of antenna
  int          dim_c, dim_d;  // metres port / starboard of antenna

  AisReport() : mmsi(0), msg_type(0),
                has_position(false), lat(0), lon(0), sog(-1), cog(-1), hdg(511),
                navstatus(0),
                has_static(false), shiptype(0),
                dim_a(0), dim_b(0), dim_c(0), dim_d(0) {}
};

class AisDecoder
{
 public:
  AisDecoder() {}

  /** Feed one raw sentence, e.g. "!AIVDM,1,1,,A,<payload>,0*5C".
   *  Returns true when a complete message was decoded into `out`. A multipart
   *  message only completes on its final fragment; the earlier fragments are
   *  buffered and return false. Anything that is not a well formed !AIVDM /
   *  !AIVDO sentence returns false and is otherwise ignored.
   *  `now` is any monotonic seconds value (the caller passes MOOSTime()); it
   *  only timestamps buffered fragments for expireFragments(). */
  bool handleSentence(const std::string& sentence, AisReport& out, double now);

  /** Drop half assembled multipart messages older than `max_age` seconds, so a
   *  lost fragment can never wedge the buffer. `now` is any monotonic seconds
   *  value - the caller passes MOOSTime(). */
  void expireFragments(double now, double max_age = 10.0);

  unsigned int sentencesSeen() const  { return m_sentences_seen; }
  unsigned int messagesDecoded() const { return m_messages_decoded; }
  unsigned int badSentences() const   { return m_bad_sentences; }

 private:
  /** A multipart message under assembly, keyed on (seq id, channel). */
  struct Fragments {
    int                       total;
    int                       pad;       // declared by the last fragment
    double                    first_seen;
    std::vector<std::string>  parts;     // index 0 == fragment 1

    Fragments() : total(0), pad(0), first_seen(0) {}
  };

  bool decodePayload(const std::string& payload, int pad, AisReport& out);

  std::map<std::string, Fragments> m_pending;

  unsigned int m_sentences_seen   = 0;
  unsigned int m_messages_decoded = 0;
  unsigned int m_bad_sentences    = 0;
};

#endif
