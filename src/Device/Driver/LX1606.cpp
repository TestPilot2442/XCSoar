// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Device/Driver/LX1606.hpp"
#include "Device/Driver.hpp"
#include "Device/Error.hpp"
#include "Device/Config.hpp"
#include "Device/Port/Port.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "Device/Driver/LX/Parsers.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "Geo/GeoPoint.hpp"
#include "Formatter/NMEAFormatter.hpp"
#include "Operation/Operation.hpp"
#include "Atmosphere/Pressure.hpp"
#include "LogFile.hpp"
#include "util/ScopeExit.hxx"

#include <chrono>
#include <string_view>
#include <cstdio>
#include <cmath>
#include <mutex>

using std::string_view_literals::operator""sv;

/*
 * Driver for the LX Navigation LX1606 vario and FLARM passthrough.
 *
 * The LX1606 PDA port carries bidirectional NMEA:
 *
 *   IN : $GPGGA + $GPRMC (+ optional $GPRMB)
 *   OUT: $LXWP0/$LXWP1/$LXWP2/$LXWP3
 *
 * Read side reuses the LXWPx parsers from src/Device/Driver/LX/Parser.cpp
 * (exposed via Parsers.hpp).
 *
 * Write side mirrors `cai_lnav`: $GPGGA + $GPRMC + $GPRMB are emitted on
 * each calculation tick (~1 Hz) using the shared NMEAFormatter helpers.
 *
 * LX1606 firmware 5.35: tested PFLX2 full-field MacCready writeback.
 * QNH writeback uses the tested PFLX3 altitude-offset command.
 */

class LX1606Device final : public AbstractDevice {
  using Clock = std::chrono::steady_clock;

  Port &port;
  /* Sampled from DeviceConfig at open time; reconnect required to change. */
  const bool send_position;
  std::mutex settings_mutex;
  bool have_wp2 = false;
  double ballast = 1, bugs = 0, polar_a = 0, polar_b = 0, polar_c = 0;
  unsigned volume = 0;
  Clock::time_point last_vario = Clock::time_point::min();
  bool qnh_initial_callback_seen = false;

  std::mutex mode_mutex;
  unsigned normal_baud = 0;
  Clock::time_point next_stream_setup = Clock::time_point::min();

  /* Called with mode_mutex held in normal NMEA mode. */
  void EnsureVarioStream(OperationEnvironment &env)
  {
    constexpr auto interval = std::chrono::seconds(5);
    const auto now = Clock::now();
    if (now < next_stream_setup || port.GetState() != PortState::READY)
      return;

    {
      const std::lock_guard lock{settings_mutex};
      if (last_vario > now - interval)
        return;
    }

    next_stream_setup = now + interval;
    PortWriteNMEA(port, "PFLX0,GPGGA,1,GPRMC,1,LXWP0,1,LXWP1,0,"
                  "LXWP2,5,LXWP3,17,LXWP4,20,LXWP5,50", env);
  }

  bool WaitForVario(OperationEnvironment &env)
  {
    try {
      port.ExpectString("$LXWP0,"sv, env, std::chrono::seconds(5));
      return true;
    } catch (const DeviceTimeout &) {
      return false;
    }
  }

public:
  LX1606Device(Port &_port, bool _send_position) noexcept
    : port(_port), send_position(_send_position) {}

  bool ParseNMEA(const char *line, NMEAInfo &info) override;

  bool PutMacCready(double mc, OperationEnvironment &env) override
  {
    if (!std::isfinite(mc) || mc < 0 || mc > 5)
      return false;

    char body[128];
    {
      const std::lock_guard lock{settings_mutex};
      if (!have_wp2)
        return false; // Do not overwrite unknown polar and ballast settings.
      std::snprintf(body, sizeof(body),
                    "PFLX2,%.1f,%.2f,%.1f,%.2f,%.2f,%.2f,%u",
                    mc, ballast, bugs, polar_a, polar_b, polar_c, volume);
    }
    PortWriteNMEA(port, body, env);
    return true;
  }

  bool PutQNH(const AtmosphericPressure &qnh,
              OperationEnvironment &env) override
  {
    const double hpa = qnh.GetHectoPascal();
    if (!std::isfinite(hpa) || hpa < 850 || hpa > 1300)
      return false;

    // Ignore only the initial standard-atmosphere callback.  A first
    // user-selected QNH (e.g. 990 hPa) must be sent immediately.
    if (!qnh_initial_callback_seen) {
      qnh_initial_callback_seen = true;
      if (std::abs(hpa - AtmosphericPressure::Standard().GetHectoPascal()) < 0.01)
        return true;
    }

    // QNH altitude shift at pressure altitude zero; PFLX3 takes feet.
    const double offset_m = qnh.PressureAltitudeToQNHAltitude(0);
    if (!std::isfinite(offset_m) || std::abs(offset_m) > 5000)
      return false;
    const long offset_ft = std::lround(offset_m / 0.3048);
    char body[64];
    std::snprintf(body, sizeof(body), "PFLX3,%ld,,,,,,,,,,,,", offset_ft);
    PortWriteNMEA(port, body, env);
    return true;
  }

  bool EnablePassThrough(OperationEnvironment &env) override
  {
    const std::lock_guard lock{mode_mutex};
    if (normal_baud != 0)
      return true;

    const unsigned baud = port.GetBaudrate();
    if (baud != 38400) {
      LogFormat("LX1606: FLARM passthrough requires a 38400 baud serial port");
      return false;
    }

    // Arm recovery before writing, in case opening the link is cancelled.
    normal_baud = baud;
    PortWriteNMEA(port, "PFLX0,COLIBRI", env);
    port.Drain();
    env.Sleep(std::chrono::seconds(2));
    return true;
  }

  bool EnableNMEA(OperationEnvironment &operation) override
  {
    const std::lock_guard lock{mode_mutex};
    if (normal_baud == 0) {
      EnsureVarioStream(operation);
      return true;
    }

    port.StopRxThread();
    const unsigned baud = normal_baud;
    AtScopeExit(this, baud) noexcept {
      normal_baud = 0;
      try {
        if (port.GetBaudrate() != baud)
          port.SetBaudrate(baud);
      } catch (...) {
        LogError(std::current_exception(), "LX1606: restoring serial baud");
      }
    };

    // Finish restoring the link even if the caller cancelled its operation.
    QuietOperationEnvironment env;
    const auto send_normal_mode = [&] {
      PortWriteNMEA(port, "PFLX0,LX1600", env);
      port.Drain();
    };

    port.Flush();
    send_normal_mode();
    if (WaitForVario(env))
      return true;

    // Follow the LK8000 recovery sequence verified on the LX1606.
    for (const unsigned speed : {4800u, baud}) {
      port.SetBaudrate(speed);
      env.Sleep(std::chrono::seconds(1));
      send_normal_mode();
      env.Sleep(std::chrono::seconds(1));
      send_normal_mode();
    }

    if (WaitForVario(env))
      return true;

    LogFormat("LX1606: no vario data after FLARM; will retry stream setup");
    return false;
  }

  void OnSysTicker() override
  {
    const std::unique_lock lock{mode_mutex, std::try_to_lock};
    if (lock.owns_lock() && normal_baud == 0) {
      NullOperationEnvironment env;
      EnsureVarioStream(env);
    }
  }

  void OnCalculatedUpdate(const MoreData &basic,
                          const DerivedInfo &calculated) override;
};

bool
LX1606Device::ParseNMEA(const char *string, NMEAInfo &info)
{
  if (!VerifyNMEAChecksum(string))
    return false;

  NMEAInputLine line(string);
  const auto type = line.ReadView();

  if (type == "$LXWP0"sv) {
    if (!LX::LXWP0(line, info))
      return false;

    const std::lock_guard lock{settings_mutex};
    last_vario = Clock::now();
    return true;
  }

  if (type == "$LXWP1"sv) {
    LX::LXWP1(line, info.device);
    return true;
  }

  if (type == "$LXWP2"sv) {
    double mc, b, bug, a, c2, c3;
    unsigned vol;
    if (std::sscanf(string, "$LXWP2,%lf,%lf,%lf,%lf,%lf,%lf,%u",
                    &mc, &b, &bug, &a, &c2, &c3, &vol) == 7 &&
        std::isfinite(b) && std::isfinite(bug) &&
        std::isfinite(a) && std::isfinite(c2) && std::isfinite(c3)) {
      const std::lock_guard lock{settings_mutex};
      ballast = b;
      bugs = bug;
      polar_a = a;
      polar_b = c2;
      polar_c = c3;
      volume = vol;
      have_wp2 = true;
    }
    return LX::LXWP2(line, info);
  }

  if (type == "$LXWP3"sv)
    return LX::LXWP3(line, info, LX::LXWP3QNH::SKIP);

  /* The LX1606 echoes any $GPGGA/$GPRMC it receives back onto its NMEA
     OUT line.  When XCSoar is the GPS source (send_position == true)
     those echoes are our own data bouncing back -- swallow them so
     they don't show up in the merge thread as a second GPS source.
     When send_position == false the user has wired an external GPS
     to the LX1606 and the echoed sentences are the only path to GPS
     in XCSoar -- let the generic parser ingest them. */
  if (send_position &&
      (type == "$GPGGA"sv || type == "$GPRMC"sv))
    return true;

  return false;
}

void
LX1606Device::OnCalculatedUpdate(const MoreData &basic,
                                const DerivedInfo &calculated)
{
  const std::unique_lock lock{mode_mutex, std::try_to_lock};
  if (!lock.owns_lock() || normal_baud != 0)
    return;

  NullOperationEnvironment env;
  char buffer[100];

  if (send_position) {
    FormatGPGGA(buffer, sizeof(buffer), basic);
    PortWriteNMEA(port, buffer, env);

    FormatGPRMC(buffer, sizeof(buffer), basic);
    PortWriteNMEA(port, buffer, env);
  }

  const GeoPoint here = basic.location_available
    ? basic.location
    : GeoPoint::Invalid();

  const ElementStat &current_leg = calculated.task_stats.current_leg;
  const AGeoPoint destination(current_leg.location_remaining,
                              current_leg.solution_planned.min_arrival_altitude);

  if (FormatGPRMB(buffer, sizeof(buffer), here, destination))
    PortWriteNMEA(port, buffer, env);
}

static Device *
LX1606CreateOnPort(const DeviceConfig &config, Port &port)
{
  return new LX1606Device(port, config.send_position);
}

const struct DeviceRegister lx1606_driver = {
  "LX1606",
  "LX1606",
  DeviceRegister::RECEIVE_SETTINGS | DeviceRegister::SEND_SETTINGS |
    DeviceRegister::SEND_POSITION | DeviceRegister::PASS_THROUGH |
    DeviceRegister::TICK_WITHOUT_DATA,
  LX1606CreateOnPort,
};
