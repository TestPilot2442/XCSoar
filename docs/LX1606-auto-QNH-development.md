# LX1606 automatic QNH — implementation checklist (development branch)

Status: **experimental implementation committed, not yet compiled or flight-tested**. Do not use as a calibrated altimeter without ground verification.

## Confirmed on Kobo Glo / LX1606

- LXWP0 QNE pressure altitude, IAS and vario received.
- MacCready bidirectional: LX -> XCSoar fast, XCSoar -> LX about 2 seconds; quick LCD menu entry may show stale MC.
- Manual QNH -> LX via PFLX3 works immediately on subsequent selection.
- Original first QNH callback was suppressed; commit abee05a changes this to suppress only an initial standard QNH of 1013.25 hPa.
- GPS altitude 0 m without valid fix **must not** trigger calibration.

## Required feature (not delivered yet)

1. Add an **Automatic QNH** checkbox to Flight Setup; persist its setting in the XCSoar profile, default choice to be confirmed.
2. Start from 1013.25 hPa without blindly writing that default to the LX1606.
3. In Automatic mode, wait for a **valid GPS fix with valid GPS altitude** and fresh LX1606 QNE pressure altitude; do not use GPS altitude=0 when it is only a placeholder.
4. Require stable ground conditions and multiple consistent samples; reject large altitude discrepancies and implausible QNH.
5. Derive static pressure from LX QNE altitude using `AtmosphericPressure::PressureAltitudeToStaticPressure(qne_altitude)`, then `AtmosphericPressure::FindQNHFromPressure(static_pressure, gps_altitude)`.
6. Apply computed QNH to XCSoar's **central** QNH setting through its existing settings/blackboard pathway (not only the device driver), then send to LX1606 using the tested PFLX3 protocol.
7. Perform automatic calibration once per startup/fix acquisition, not continuously; avoid feedback loops or overwriting a later manual change.
8. When Automatic is unchecked, preserve manually selected QNH and forward changes to LX1606.
9. Preserve LXWP0 as QNE input and existing bidirectional MacCready handling.
10. Tests: no GPS, invalid GPS altitude, stable fix, stale QNE, implausible offset, manual override, reconnect, sync direction, persistence, reboot.

## Architecture warning

`LX160Device::OnCalculatedUpdate` has GPS and QNE samples, but does **not** own XCSoar's central QNH setting. Do not directly emit a PFLX3 from this callback without also updating the central QNH: the display and instrument would diverge. XCSoar already has `src/Computer/AutoQNH.cpp`, but it currently requires static pressure and uses waypoint/terrain elevation; the LX1606-specific GPS-versus-QNE path and user preference must be integrated into the central settings pipeline.

Keep branch `lx1606-qnh-sync-test-7.46.1` at commit `700bbfe` unchanged as the field-tested baseline.

## Experimental implementation

- `src/ProcessTimer.cpp`: 15 stable on-ground timer observations with valid GPS fix, GPS AMSL altitude, QNE altitude and low ground speed, then computes QNH from standard-atmosphere pressure and calls `ActionInterface::SetQNH(qnh, true)`.
- `src/Computer/Settings.hpp`, `src/Computer/Settings.cpp`, `src/Profile/*`: shared profile-persistent `lx1606_auto_qnh` setting (off by default).
- `src/Dialogs/Settings/dlgBasicSettings.cpp`, `src/InfoBoxes/Panel/AltitudeSetup.cpp`: checkbox in Flight Setup and altitude NavBox panel. Manual QNH entry disables automatic mode.
- **Outstanding**: CI build and Kobo ground verification; verify LXWP0 altitude really is QNE and is unaffected by changing LX QNH. The test build must not be used in flight until verified.
