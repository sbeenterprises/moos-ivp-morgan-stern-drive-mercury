/************************************************************/
/*    FILE: CanProfile.h                                    */
/*    DATE: October 2026                                    */
/*                                                          */
/*    Per-boat CAN drive profile: parses a .prf file,       */
/*    validates it, encodes drive commands into CAN frames  */
/*    and decodes feedback frames. No MOOS dependency, so   */
/*    it can be unit tested on its own (see tests/).        */
/************************************************************/

#ifndef CAN_PROFILE_HEADER
#define CAN_PROFILE_HEADER

#include <map>
#include <string>
#include <vector>
#include <cstdint>

namespace canprofile {

enum Gear { GEAR_NEUTRAL = 0, GEAR_FWD = 1, GEAR_REV = -1 };

std::string gearName(Gear gear);   // "fwd" | "neutral" | "rev"

// What the drive layer has decided to send this tick, after limits,
// ownership and the shift interlock have been applied.
struct DriveCommand {
  double rudder_pct;   // -100 port .. +100 starboard
  double thrust_pct;   // -100 astern .. +100 ahead
  Gear   gear;
  DriveCommand() : rudder_pct(0), thrust_pct(0), gear(GEAR_NEUTRAL) {}
};

struct Frame {
  uint32_t id;
  bool     ext;
  uint8_t  dlc;
  uint8_t  data[8];
  Frame() : id(0), ext(true), dlc(0) { for(int i=0; i<8; i++) data[i] = 0; }
  std::string str() const;   // "18FF11D0#FF01FFFFFFFFFFFF", same as candump -L
};

// One calibration point or table row: input -> raw value
struct Point { double x; double y; };

struct TxField {
  std::string src;      // rudder_pct | rudder_side | thrust_pct | thrust_abs_pct | gear
  int         byte;     // first byte
  int         bits;     // 8, 16, 24 or 32
  bool        big_endian;
  bool        is_signed;
  std::string map;      // linear | points | table | enum
  double      scale, offset;                 // linear
  std::vector<Point> points;                 // points
  std::string table_file;
  std::map<int, long long> table;            // table: trunc(input) -> raw
  std::map<std::string, long long> enum_values;  // enum: state -> raw
  TxField() : byte(0), bits(8), big_endian(false), is_signed(false),
              scale(1), offset(0) {}
};

struct TxConst {
  int byte;
  std::vector<uint8_t> bytes;
};

struct TxMsg {
  std::string name;
  std::string cap;      // steer | thrust
  uint32_t    id;
  bool        ext;
  int         dlc;
  int         period_ms;
  uint8_t     fill;
  bool        guard;    // refuse to send while another node sends this ID
  std::vector<TxField> fields;
  std::vector<TxConst> consts;
  TxMsg() : id(0), ext(true), dlc(8), period_ms(100), fill(0xFF), guard(true) {}
};

struct RxPart { int byte; int lo_bit; int hi_bit; };

struct RxField {
  std::string name;     // MOOS variable to post
  uint32_t    id;
  bool        ext;
  std::vector<RxPart> parts;  // concatenated least significant part first
  bool        is_signed;
  std::string map;      // raw | linear | points | enum
  double      scale, offset;
  std::vector<Point> points;
  std::map<long long, std::string> enum_values;
  RxField() : id(0), ext(true), is_signed(false), map("raw"), scale(1), offset(0) {}
};

// Result of decoding one rx_field from a frame
struct RxValue {
  std::string name;
  bool        is_string;
  double      dval;
  std::string sval;
};

class Profile {
 public:
  Profile();

  // Reads and validates the file. Returns false if anything is wrong;
  // errors() then lists every problem found. Never throws.
  bool load(const std::string& path);
  bool loadFromString(const std::string& text, const std::string& base_dir);

  const std::vector<std::string>& errors() const { return m_errors; }
  bool valid() const { return m_loaded && m_errors.empty(); }

  // Boat settings
  const std::string& name() const         { return m_name; }
  const std::string& canInterface() const { return m_can_interface; }
  bool   hasCapability(const std::string& cap) const;
  std::string capabilitiesString() const;
  double setting(const std::string& key) const;   // numeric settings below
  std::string stringSetting(const std::string& key) const;
  std::string hashHex() const { return m_hash; }

  const std::vector<TxMsg>&   txMessages() const { return m_tx; }
  const std::vector<RxField>& rxFields() const   { return m_rx; }

  // Encode one tx message for the given command. Values outside a
  // field's range are clamped; never produces a frame that failed
  // validation.
  Frame encode(const TxMsg& msg, const DriveCommand& cmd) const;

  // Decode all rx_fields that match this frame.
  std::vector<RxValue> decode(uint32_t id, bool ext, const uint8_t* data,
                              int dlc) const;

  // True if a received frame uses the ID of one of our tx messages
  // (meaning another node is transmitting it).
  bool isTxId(uint32_t id, bool ext, std::string& msg_name) const;

 private:
  void   parseLine(const std::string& key, const std::string& val, int lineno);
  void   parseTxMsg(const std::string& val, int lineno);
  void   parseTxField(const std::string& val, int lineno);
  void   parseTxConst(const std::string& val, int lineno);
  void   parseRxField(const std::string& val, int lineno);
  bool   loadTable(TxField& field, int lineno);
  void   validate();
  void   validateField(const TxMsg& msg, const TxField& f);
  TxMsg* findMsg(const std::string& name);
  void   addError(int lineno, const std::string& msg);

  long long fieldRaw(const TxField& f, const DriveCommand& cmd) const;

 private:
  bool        m_loaded;
  std::string m_base_dir;
  std::string m_name;
  std::string m_can_interface;
  std::vector<std::string> m_caps;
  std::map<std::string, double> m_settings;
  std::map<std::string, std::string> m_string_settings;
  std::vector<TxMsg>   m_tx;
  std::vector<RxField> m_rx;
  std::vector<std::string> m_errors;
  std::string m_hash;
  std::string m_hash_input;
};

}  // namespace canprofile

#endif
