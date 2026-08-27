/************************************************************/
/*    NAME: Douglas Lima                                    */
/*    FILE: NmeaOut.h                                       */
/*    DATE: 2026                                            */
/*                                                          */
/*  UDP sink for NMEA0183, pointed at the Signal K server's  */
/*  NMEA0183 provider (127.0.0.1:10112 by default - see      */
/*  ~/.signalk/settings.json). Signal K parses what it       */
/*  understands into deltas for the UI, and its              */
/*  udp-nmea-sender plugin re-emits every sentence to        */
/*  127.0.0.1:10111, where iDivisorNMEA picks it up.         */
/************************************************************/

#ifndef AISSIM_NmeaOut_HEADER
#define AISSIM_NmeaOut_HEADER

#include <netinet/in.h>
#include <string>

class NmeaOut
{
 public:
  NmeaOut();
  ~NmeaOut();

  bool open(const std::string& host, int port);
  bool send(const std::string& sentence);

  unsigned int sent() const { return m_sent; }

 private:
  int                m_sock;
  struct sockaddr_in m_addr;
  unsigned int       m_sent;
};

/** Append "*<checksum>" to a sentence body that already starts with '$' or
 *  '!'. The AIVDM sentences from AisEncode arrive already framed, so this is
 *  only used for the ownship sentences below. */
std::string nmeaFrame(const std::string& body);

/** $GPRMC - position, speed over ground, course over ground.
 *  utc_secs is a Unix timestamp; sog is knots, cog degrees true. */
std::string buildGPRMC(double utc_secs, double lat, double lon,
                       double sog, double cog);

/** $GPHDT - true heading from the gyro. */
std::string buildGPHDT(double heading);

/** $AGRSA - rudder angle. iDivisorNMEA turns this into NAV_YAW, which
 *  planchaPID reads back as its rudder feedback. */
std::string buildAGRSA(double rudder_deg);

/** $SDDPT - depth below transducer. */
std::string buildSDDPT(double depth_m);

#endif
