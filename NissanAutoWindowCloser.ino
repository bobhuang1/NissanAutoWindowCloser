/*
 * NissanAutoWindowCloser  —  Arduino Nano + MCP2515 CAN module
 * ---------------------------------------------------------------------------------
 * Automatic window closer / convenience module for a Nissan (built for a Sylphy).
 *
 * Features:
 *   1. Close all windows when the car switches off (BCM ignition state off the bus,
 *      or the ACC divider on A0) and/or when a lock event arrives.
 *   2. Hazards while any door / trunk is open (parked only).
 *   3. Roll all windows down on a triple unlock.
 *   4. Flash the hazards for 10 s on a hard deceleration.
 *   5. Headlights while an automatic close is running.
 *   6. Horn confirmation: one 0.5 s honk when the car locks, two when it unlocks.
 *   7. Automatic re-lock 60 s after an unlock if no door was opened.
 *
 * WHERE THE VALUES COME FROM
 * ---------------------------------------------------------------------------------
 * RX frames come from the Nissan LEAF body-CAN database:
 *   https://github.com/dalathegreat/leaf_can_bus_messages
 *     CAR-can_AZE0.dbc -> x60D_BCM_GeneralStatus7  (doors, lock status, ignition)
 *                      -> x625_USM_GeneralStatus    (headlight status)
 *                      -> x284                      (vehicle speed from the ABS)
 *     that repo's poll table -> BCM query 0x745 / response 0x765
 *
 * TX commands go through the BCM's "InputOutputControlByLocalIdentifier" service
 * ($30) inside the extended diagnostic session ($10 C0), which is the only
 * *documented* command channel a Nissan body bus exposes:
 *   https://github.com/balrog-kun/nissan-qashqai-can-info  (BCM $30 LID map)
 *   https://projectbytes.wordpress.com/2015/07/08/nissan-370z-can-control/
 * That is how the sketch locks, unlocks, honks and drives the beams / hazards.
 *
 * STILL A PLACEHOLDER: the window up/down frames. No public Nissan database has a
 * "roll the windows" body-CAN command - the four motors hang off a LIN bus with the
 * BCM as master - so WINDOW_FRAMES_VERIFIED is 0 and the sketch refuses to put those
 * frames on the bus until you have sniffed your own car. See the README section
 * "Find the real window frame".
 *
 * Library:  MCP_CAN (coryjfowler / Seeed lineage) — install from Library
 *           Manager ("mcp_can"), or PlatformIO dependency. The code targets the
 *           1.5.x API that platformio.ini pins: begin(MCP_STDEXT, ...),
 *           readMsgBuf() and MCP_NORMAL. The 2.x library renamed/removed those
 *           (readMsgBufID, MCP_MODE_*), so do not silently bump the major version.
 *
 * Wiring (MCP2515 v1 module):
 *   module   Nano
 *   VCC   -> 5V
 *   GND   -> GND
 *   CS    -> D10
 *   SCK   -> D13
 *   MOSI  -> D11
 *   MISO  -> D12
 *   INT   -> D2    (optional; receive is polled, no interrupt needed)
 *
 *   ACC sense: ACC wire (12 V) through a divider (e.g. 10 k + 4.7 k) to A0.
 *              Optional once the BCM's own ignition state is read (ACC_SOURCE_CAN).
 * ---------------------------------------------------------------------------------
 */

#include <SPI.h>
#include "mcp_can.h"

/* ============================== CONFIG =====================================
 * Everything you might need to change for another car / module lives here.
 * ==========================================================================*/

/* --- MCP2515 CAN controller ------------------------------------------------ */
#define SPI_CS_PIN        10
#define CAN_SPEED         CAN_500KBPS          // Nissan body CAN is 500 kbit/s
#define CAN_CLOCK         MCP_16MHZ            // integrated PCB uses a 16 MHz crystal; the common plug-in module uses 8 MHz -> MCP_8MHZ
#define CAN_DRAIN_PER_LOOP 8                   // how many RX frames one loop() pass handles

/* --- BCM / IPDM diagnostic request + response IDs ----------------------------
 * From the poll table in the LEAF messages repo: BCM 0x745 -> 0x765,
 * IPDM E/R 0x74D -> 0x76D.  Nissan's response ID is request ID + 0x20. */
#define BCM_CMD_ID        0x745UL
#define BCM_RSP_ID        0x765UL
#define IPDM_CMD_ID       0x74DUL

/* --- ISO 14229 framing of the command frames ---------------------------------
 * Every request below is a single CAN frame carrying an ISO 15765-2 length
 * prefix, then the payload, then zero padding:
 *   session   02 10 C0 00 00 00 00 00         (0x02 = 2 payload bytes)
 *   command   04 30 <LID> <func> <val> 00 00  (0x04 = 4 payload bytes)
 *   reply     05 70 ...                      (0x05 = 5 payload bytes, on 0x765)
 * The BCM only honours $30 while the extended diagnostic session is open. It
 * falls back to the default session on its own after ~5 s, so the sketch opens
 * $10 C0 before a command, reuses it for up to BCM_REUSE_MS (so a double honk or
 * the hazard alternation does not re-open it every frame) and can hand the ECU
 * its default session back with serial 'x'. */
#define BCM_PCI_SESSION   0x02
#define BCM_PCI_IO        0x04
#define BCM_SID_SESSION   0x10
#define BCM_SID_IO        0x30
#define BCM_SESSION_C0    0xC0                 // extended / diagnostic session
#define BCM_SESSION_DEF   0x81                 // back to the default session
#define BCM_GAP_MS        20UL                 // pause around each command frame
#define BCM_REUSE_MS      2000UL               // skip enter/leave if we just commanded this ECU

/* --- BCM $30 local IDs (a.k.a. BCM PIDs) -------------------------------------
 * func 0x00 = one-shot command, func 0x20 = state change that times out after 5 s.
 * Get the func wrong and the BCM just ignores the frame; nothing gets screwed up. */
#define LID_LOCK          0x07                 // power door lock
#define LOCK_FUNC         0x00
#define LOCK_CMD_LOCK     0x01                 // lock all doors
#define LOCK_CMD_UNLOCK   0x02                 // unlock all doors

#define LID_BEEP          0x11                 // key beep, i.e. the horn the fob uses
#define BEEP_FUNC         0x20
#define BEEP_OFF          0x00
#define BEEP_ON           0x01

#define LID_BEAM          0x3B                 // low / high beam
#define BEAM_FUNC         0x20
#define BEAM_OFF          0x00
#define BEAM_ON           0x01                 // 2 = high beam, not used here

/* $30 PID 0x4C drives ONE side at a time, so "hazards" has to alternate 0x01 and
 * 0x02. If you ever want a one-shot horn instead of a 0.5 s tone, the IPDM-E/R
 * $30 PID 0x05 (func 0x00, value 1) is a true "very short honk" - on IPDM_CMD_ID. */
#define LID_TURN          0x4C
#define TURN_FUNC         0x20
#define TURN_OFF          0x00
#define TURN_RIGHT        0x01
#define TURN_LEFT         0x02

/* --- ACC / ignition sensing ------------------------------------------------- */
#define ACC_SENSE_PIN     A0                   // ACC 12 V via divider -> 0..5 V
#define ACC_OFF_MV        1200UL               // below this value ACC is considered OFF
#define ACC_OFF_DEBOUNCE_MS 400UL              // consecutive time below threshold -> OFF
#define ACC_SOURCE_CAN    1                    // 1 = trust the BCM's own ignition state
                                               //     (0x60D) once it arrives and keep
                                               //     the A0 divider as the fallback.
#define TRIGGER_CAN_IGNITION 1                 // actually use it
#define IGN_BYTE          1                    // 0x60D byte 1, mask 0x60 (BCM_VehicleState)
#define IGN_MASK          0x60
#define IGN_SHIFT         5
#define IGN_OFF           0
#define IGN_ACC           1
#define IGN_START         2
#define IGN_ON            3
#define CAN_IGN_DEBOUNCE_MS 250UL

/* Alternative ignition encoding on the Qashqai / Sentra / 370Z generation: the
 * same frame, byte 1, bits 0x0C, 0b00 OFF / 0b01 ACC / 0b10 START / 0b11 ON. If the
 * LEAF encoding above does not track your ignition, switch to:
 *   #define IGN_MASK 0x0C
 *   #define IGN_SHIFT 2                                                      */

#define CLOSE_DELAY_MS    1500UL               // delay AFTER the trigger before closing
#define CLOSE_SPACING_MS  250UL                // pause between window frames
#define SEND_RETRIES      3                    // retries per frame if transmission fails
#define EVENT_COOLDOWN_MS 10000UL              // minimum gap between automatic close/roll-downs
#define AUTO_CLOSE_AT_BOOT 0                   // 1 = close even if ACC already OFF at boot

/* --- Event sources ---------------------------------------------------------- */
#define TRIGGER_ACC_OFF   1                    // close when ACC switches OFF
#define TRIGGER_DOOR_LOCK 1                    // close when the doors lock while ACC OFF
#define TRIGGER_HAZARD_ON_DOOR 1               // hazards while any door/trunk is open (parked)
#define TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK 1   // 3 unlocks in <3 s -> all windows down
#define TRIGGER_BRAKE_WARNING_LIGHTS 1         // car running + fast decel -> 10 s hazards
#define TRIGGER_HORN_ON_LOCK   1               // 1 x 0.5 s honk when the car locks
#define TRIGGER_HORN_ON_UNLOCK 1               // 2 x 0.5 s honks when the car unlocks
#define TRIGGER_AUTO_RELOCK 1                  // re-lock if nobody opened a door
#define LIGHTS_WHILE_CLOSE 1                   // turn on headlights while closing
#define TRIGGER_CAN_MONITOR 0                  // 1 = allow 'm' to print every frame

/* --- Roll windows down after a triple unlock -------------------------------- */
#define TRIPLE_UNLOCK_WINDOW_MS 3000UL         // 3 unlocks inside this window -> roll down
#define ROLL_DOWN_DELAY_MS    1000UL           // delay AFTER the 3rd unlock
#define ROLL_DOWN_SPACING_MS  250UL            // pause between window-down frames

/* --- Horn confirmation ------------------------------------------------------ */
#define HORN_PULSE_MS       500UL              // one honk lasts half a second
#define HORN_GAP_MS         250UL              // silence between the two unlock honks
#define HORN_PULSES_LOCK    1                  // honks on a lock event
#define HORN_PULSES_UNLOCK  2                  // honks on an unlock event

/* --- Automatic re-lock ------------------------------------------------------ */
#define RELOCK_DELAY_MS     60000UL            // unlocked this long with no door opened -> lock again
#define RELOCK_ARM_AT_BOOT  0                  // 1 = also arm if the car is already unlocked at boot

/* --- Door / tailgate open sensing --------------------------------------------
 * x60D_BCM_GeneralStatus7, byte 0 (byte A):
 *   0x80 unused | 0x40|0x20 parking lights | 0x10 passenger door | 0x08 driver door
 *   0x04 rear left | 0x02 rear right | 0x01 trunk
 * The LEAF DBC and the Qashqai measurements agree on 0x08/0x10 (the front doors)
 * but disagree about the neighbouring bits, so narrow DOOR_MASK to 0x18 (front
 * doors only) if 0x1F ever misfires. */
#define DOOR_BYTE        0
#define DOOR_MASK        0x1F                  // FL|FR|RL|RR + trunk
#define DOOR_DEBOUNCE_MS 150UL                 // door-switch debounce
#define HAZARD_ONLY_WHEN_PARKED 1              // 1 = hazard only while ACC is OFF

/* --- Door lock status sensing ------------------------------------------------
 * x60D byte 2 (byte C): 0x20 = other doors locked, 0x10 = driver door locked.
 * Both databases call this "any door locked": 1 = locked, 0 = unlocked. */
#define LOCKSTAT_BYTE    2
#define LOCKSTAT_MASK    0x30

/* --- Headlight status (read-back, for diagnostics only) ---------------------
 * x625_USM_GeneralStatus byte 1: 0x00 off, 0x40 parking, 0x60 headlights,
 * 0x68 headlights + fog. */
#define HEADLIGHT_STAT_ID   0x625UL
#define HEADLIGHT_STAT_BYTE 1

/* --- Brake-hazard: flash the hazards on a hard deceleration -------------------
 * x284 (ABS) VehicleSpeedFromABS: 16-bit little-endian across bytes 4/5,
 * 0.01 km/h per count. Thresholds below are in tenths of a km/h so the decel rate
 * keeps 0.1 km/h resolution. */
#define SPEED_FRAME_ID   0x284UL
#define SPEED_BYTE_LO    4
#define SPEED_BYTE_HI    5
#define SPEED_CMH_PER_COUNT 100UL              // 0.01 km/h per raw count
#define MIN_SPEED_KMH_X10    200UL              // 20.0 km/h
#define DECEL_THRESHOLD_KMPHS_X10 250UL         // 25.0 km/h lost per second
#define DECEL_MIN_DT_MS    100UL                // ignore sub-100 ms glitches
#define DECEL_MAX_DT_MS    2000UL               // ignore long gaps / edge cases
#define BRAKE_HAZARD_MS    10000UL              // how long the hazards flash
#define BRAKE_COOLDOWN_MS  20000UL              // min pause before re-triggering

#define HAZARD_TOGGLE_MS  400UL                 // L/R alternation period
#define HEADLIGHTS_HOLD_MS 1500UL               // keep the beams on after the last window frame

/* --- Master feature switch (integrated PCB only) --------------------------------
 * J5 on the PCB shorts PC1/A1 to GND -> ALL automatic features pause (close,
 * roll-down, door-hazard, brake-hazard, horn, auto-relock). Manual serial
 * commands still work. Set to -1 (not fitted) for breadboard builds. */
#define MASTER_ENABLE_PIN  A1

/* --- STATUS / debug ---------------------------------------------------------- */
#define STATUS_LED        9                    // integrated PCB LED2 -> PB1/D9 (NOT D13, that is SPI SCK!)
#define LED_ACTIVE_HIGH   0                    // PCB: LED anode to +5V -> shines when pin is LOW. UNO built-in LED on D13 would be 1.
#define ledOn()           digitalWrite(STATUS_LED, LED_ACTIVE_HIGH ? HIGH : LOW)
#define ledOff()          digitalWrite(STATUS_LED, LED_ACTIVE_HIGH ? LOW  : HIGH)

/* --- One switch for everything that needs the lock status bits -------------- */
#define NEED_LOCK_STATUS  (TRIGGER_DOOR_LOCK || TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK || \
                           TRIGGER_HORN_ON_LOCK  || TRIGGER_HORN_ON_UNLOCK || \
                           TRIGGER_AUTO_RELOCK)

/* --- RX frame IDs taken from the LEAF database ------------------------------- */
#define BCM_STATUS_ID 0x60DUL                  // x60D_BCM_GeneralStatus7, sender BCM

/* ==================== WINDOW FRAME TABLES ====================================
 * PLACEHOLDERS, and the one place where guessing is genuinely dangerous.
 *
 * No public Nissan database contains a "roll the windows up / down" body-CAN
 * command. On a Sylphy / LEAF the four motors sit on a LIN bus with the BCM as
 * the master, so a request has to reach the BCM and not the motors - and any ID
 * you invent belongs to somebody else: 0x180 is a live VCM torque message on real
 * Nissans. WINDOW_FRAMES_VERIFIED therefore stays 0 and the sketch refuses to put
 * these frames on the bus until you have filled in the real IDs from your own car
 * (README "Find the real window frame").
 *
 * `ext` marks an extended (29-bit) ID: 1 = extended, 0 = standard.
 * ============================================================================*/
#define WINDOW_FRAMES_VERIFIED 0

typedef struct {
  const char *name;                 // window name for logging
  uint32_t    id;                   // CAN frame ID
  uint8_t     ext;                  // 0 standard / 1 extended
  uint8_t     dlc;                  // payload length (<= 8)
  uint8_t     data[8];              // payload bytes
} WinFrame;

/* Windows UP ("close") - used by the auto-close triggers. */
static const WinFrame WINDOWS_UP[] = {
  { "FL", 0x000UL, 0, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} }, // driver
  { "FR", 0x000UL, 0, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} }, // passenger
  { "RL", 0x000UL, 0, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} }, // rear left
  { "RR", 0x000UL, 0, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} }, // rear right
};
#define NUM_WINDOWS_UP  (sizeof(WINDOWS_UP) / sizeof(WINDOWS_UP[0]))

/* Windows DOWN ("open/roll down") - used by the triple-unlock trigger. */
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static const WinFrame WINDOWS_DOWN[] = {
  { "FL", 0x000UL, 0, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} }, // driver
  { "FR", 0x000UL, 0, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} }, // passenger
  { "RL", 0x000UL, 0, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} }, // rear left
  { "RR", 0x000UL, 0, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} }, // rear right
};
#define NUM_WINDOWS_DOWN (sizeof(WINDOWS_DOWN) / sizeof(WINDOWS_DOWN[0]))
#endif

/* ============================== STATE ======================================== */
MCP_CAN CAN0(SPI_CS_PIN);

static unsigned long _nextEventMs = 0;          // millis() gate for EVENT_COOLDOWN_MS
static unsigned long _closeAtMs   = 0;          // pending close timer
static const char  *_closeReason  = nullptr;    // reason for the pending close

static bool _accOn          = false;            // last stable ACC/ignition state
static unsigned long _accLowSinceMs = 0;        // when the ACC reading first went low

#if ACC_SOURCE_CAN
static bool _ignSeen         = false;           // the first 0x60D ignition report arrived
static uint8_t _ignPending    = 0xFF;           // 0xFF = nothing pending
static uint8_t _ignLatched    = 0xFF;
static unsigned long _ignPendingMs = 0;
#endif

#if NEED_LOCK_STATUS
static bool _carLocked          = false;        // level read from 0x60D byte 2
static bool _lockStateValid     = false;        // ignore the very first sample
#endif
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static uint8_t   _unlockCount   = 0;            // unlock burst counter
static unsigned long _unlockWindowStartMs = 0;  // when the first unlock landed
static unsigned long _openAtMs     = 0;          // pending roll-down timer
static const char  *_openReason   = nullptr;    // reason for the pending roll-down
#endif

#if TRIGGER_HAZARD_ON_DOOR
static bool _doorsOpen            = false;      // debounced door/tailgate state
static bool _doorDebouncePending  = false;      // edge seen, debounce running
static unsigned long _doorsChangeAtMs = 0;
#endif

#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
static bool    _hazardsOn        = false;      // is a hazard request active
static bool    _hazardsWanted    = false;      // previous value of _hazardDemands != 0
static uint8_t _hazardDemands    = 0;          // OR of hazard requests
static uint8_t _hazardPhase      = 0;          // 0 off / 1 right / 2 left
static unsigned long _nextHazardToggleMs = 0;
#define HAZARD_REQ_DOOR    0x01
#define HAZARD_REQ_BRAKE   0x02
#define HAZARD_REQ_MANUAL  0x04
#endif

#if TRIGGER_BRAKE_WARNING_LIGHTS
static bool         _brakeHazardsActive = false;
static unsigned long _brakeHazardUntilMs  = 0;
static unsigned long _brakeCooldownUntilMs = 0;
static unsigned long _lastSpeedX10   = 0;       // previous sample, in tenths of km/h
static unsigned long _lastSpeedAtMs = 0;       // when it was sampled
#endif

#if TRIGGER_HORN_ON_LOCK || TRIGGER_HORN_ON_UNLOCK
#define HORN_IDLE 0
#define HORN_ON   1
#define HORN_GAP  2
static uint8_t  _hornPhase      = HORN_IDLE;
static uint8_t  _hornPulsesLeft = 0;
static unsigned long _hornNextMs  = 0;
#endif

#if TRIGGER_AUTO_RELOCK
static bool _relockArmed = false;               // waiting out RELOCK_DELAY_MS
static unsigned long _relockAtMs = 0;
#endif

static bool _headlightsSeen = false;            // 0x625 read-back
static bool _headlightsOn   = false;

static uint8_t  _rxLen;
static uint8_t  _rxData[8];
static uint32_t _rxFrameId;

static uint32_t    _lastCmdId = 0;              // last ECU we sent a $30 command to
static unsigned long _lastCmdMs = 0;
static bool         _sessionOpen = false;       // we left the BCM in a diagnostic session

static bool _enabled = true;                    // cleared by the master-disable switch

static bool     _ledFlashing = false;           // non-blocking status-LED blink
static unsigned long _ledAtMs = 0;

#if TRIGGER_CAN_MONITOR
static bool _monitorOn = false;
#endif

/* ============================ FORWARD DECLARATIONS ========================= */
static void setAccState(bool on);
static void pollAcc(void);
static void readCanPipe(void);
static void handleStatusFrame(void);
static void handleDoorBits(void);
static void handleLockBits(void);
static void handleHeadlightFrame(void);
#if ACC_SOURCE_CAN && TRIGGER_CAN_IGNITION
static void pollCanIgnition(void);
#endif
#if NEED_LOCK_STATUS
static void onCarLocked(void);
static void onCarUnlocked(void);
#endif
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
static void handleUnlockBurst(void);
static void scheduleRollDown(const char *reason);
static void runPendingRollDown(void);
static void openAllWindows(const char *reason);
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
static void handleSpeedFrame(void);
static void startBrakeWarning(unsigned long now);
static void pollBrakeWarning(void);
#endif
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
static void updateHazards(void);
#endif
#if TRIGGER_HORN_ON_LOCK || TRIGGER_HORN_ON_UNLOCK
static void honk(uint8_t pulses, const char *why);
static void pollHorn(void);
static void abortHorn(void);
#endif
#if TRIGGER_AUTO_RELOCK
static void armRelock(void);
static void pollRelock(void);
static void reArmRelock(void);
static void forceRelock(void);
#endif
static void scheduleClose(const char *reason);
static void runPendingClose(void);
static void closeAllWindows(const char *reason);
static bool bcmCommand(uint32_t id, uint8_t lid, uint8_t func, uint8_t val,
                       const char *what);
static void bcmCloseSession(void);
static bool sendFrame(uint32_t id, uint8_t ext, uint8_t dlc,
                      const uint8_t *data, const char *what);
static void flashLed(uint16_t ms);
static void pollLed(void);
static void blink(uint8_t times, uint16_t halfPeriodMs);
static bool frameIdMatches(uint32_t id, uint8_t ext);
static unsigned long readAccMv(void);
static void handleSerial(void);
#if MASTER_ENABLE_PIN >= 0
static void pollMasterEnable(void);
#endif

/* ================================ SETUP ====================================== */
void setup() {
  Serial.begin(115200);
  Serial.println(F("\r\n[NissanAutoWindowCloser] boot"));

  pinMode(STATUS_LED, OUTPUT);
  ledOff();
  pinMode(ACC_SENSE_PIN, INPUT);
#if MASTER_ENABLE_PIN >= 0
  pinMode(MASTER_ENABLE_PIN, INPUT_PULLUP);  // J5 shorts to GND = disabled
#endif

  // CAN controller on the bus. mcp_can 1.5.x wants the ID mode as the first
  // argument, then speed, then the crystal; MCP_STDEXT = accept both 11 and 29 bit.
  if (CAN0.begin(MCP_STDEXT, CAN_SPEED, CAN_CLOCK) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);                   // be explicit rather than rely on library defaults
    Serial.println(F("CAN controller OK (normal mode)"));
  } else {
    Serial.println(F("FATAL: CAN controller init failed"));
    Serial.println(F("Hint: check CAN_CLOCK (8 vs 16 MHz) and wiring."));
    while (1) { blink(SEND_RETRIES, 150); }     // hard stop, blink pattern
  }

  Serial.println(F("RX (dalathegreat/leaf_can_bus_messages):"));
  Serial.print(F("  0x")); Serial.print(BCM_STATUS_ID, HEX);
  Serial.print(F(" door mask 0x"));    Serial.println(DOOR_MASK, HEX);
  Serial.print(F("       lock mask 0x")); Serial.println(LOCKSTAT_MASK, HEX);
  Serial.println(F(" in byte 2, 1 = locked"));
  Serial.print(F("  0x")); Serial.print(SPEED_FRAME_ID, HEX);
  Serial.print(F(" speed bytes ")); Serial.print(SPEED_BYTE_LO);
  Serial.print(F("/")); Serial.println(SPEED_BYTE_HI);
  Serial.print(F("  0x")); Serial.print(HEADLIGHT_STAT_ID, HEX);
  Serial.println(F(" headlight read-back"));
  Serial.println(F("TX (BCM $30 inside diag session C0, on 0x745):"));
  Serial.println(F("  30 07 00 01 lock all   |  30 07 00 02 unlock all"));
  Serial.println(F("  30 11 20 01 horn on   |  30 11 20 00 horn off"));
  Serial.println(F("  30 3B 20 01 beams on  |  30 3B 20 00 beams off"));
  Serial.println(F("  30 4C 20 01/02 hazards, left/right alternated"));
  Serial.print(F("  window frames verified: ")); Serial.println(WINDOW_FRAMES_VERIFIED);
#if TRIGGER_HORN_ON_LOCK || TRIGGER_HORN_ON_UNLOCK
  Serial.print(F("  horn: ")); Serial.print(HORN_PULSES_LOCK);
  Serial.print(F(" x ")); Serial.print(HORN_PULSE_MS);
  Serial.print(F(" ms on lock, ")); Serial.print(HORN_PULSES_UNLOCK);
  Serial.println(F(" on unlock"));
#endif
#if TRIGGER_AUTO_RELOCK
  Serial.print(F("  auto-relock after ")); Serial.print(RELOCK_DELAY_MS / 1000UL);
  Serial.println(F(" s with no door opened"));
#endif
  Serial.print(F("  window frames to send: ")); Serial.print((unsigned)NUM_WINDOWS_UP);
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
  Serial.print(F(" up / ")); Serial.print((unsigned)NUM_WINDOWS_DOWN);
#endif
  Serial.println(F(" down"));
#if ACC_SOURCE_CAN
  Serial.print(F("  ACC divider (fallback) threshold: "));
  Serial.print(ACC_OFF_MV); Serial.println(F(" mV"));
#endif

  // Initial state, so we can detect a falling edge later. The analog reading is
  // used until the BCM's own ignition state shows up on 0x60D.
  _accOn = (readAccMv() >= ACC_OFF_MV);
  if (_accOn) Serial.println(F("ACC: ON"));
  else {
    Serial.println(F("ACC: OFF at boot"));
#if AUTO_CLOSE_AT_BOOT
    scheduleClose(F("ACC off at boot"));
#endif
  }
}

/* ================================= LOOP ====================================== */
void loop() {
#if MASTER_ENABLE_PIN >= 0
  pollMasterEnable();   // watch the physical kill-switch
#endif
  pollLed();            // non-blocking status-LED countdown
  readCanPipe();        // keep ears open (lock / door / speed events)
  pollAcc();            // analog ACC power-off events (fallback ignition source)
  runPendingClose();    // fire the delayed close if it is due
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
  runPendingRollDown(); // fire the delayed roll-down if it is due
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
  pollBrakeWarning();   // count down an active brake-hazard burst
#endif
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
  updateHazards();      // alternate the hazard L/R request while demanded
#endif
#if TRIGGER_HORN_ON_LOCK || TRIGGER_HORN_ON_UNLOCK
  pollHorn();           // count down the current honk
#endif
#if TRIGGER_AUTO_RELOCK
  pollRelock();         // re-lock if the car sat unlocked and untouched
#endif
  handleSerial();       // manual test commands over USB
}

/* ============================= EVENT SOURCES ================================= */

/* Single place where the ACC / ignition edge is reacted to. */
static void setAccState(bool on) {
  if (on == _accOn) return;
  _accOn = on;
  Serial.println(on ? F("ACC: ON") : F("ACC: OFF"));

  if (on) {
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
    // Driving clears the automatic requests but leaves a manual 'd' alone.
    _hazardDemands &= (uint8_t)~(HAZARD_REQ_DOOR | HAZARD_REQ_BRAKE);
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
    _brakeHazardsActive = false;
    _lastSpeedAtMs = 0;                          // re-arm the speed tracker
#endif
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
    _unlockCount = 0;                             // a fresh drive resets the unlock burst
#endif
#if TRIGGER_AUTO_RELOCK
    _relockArmed = false;                         // never lock a car that is running
#endif
  } else {
#if TRIGGER_ACC_OFF
    if (_enabled) scheduleClose(F("ACC off"));
#endif
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
    _hazardDemands &= (uint8_t)~(HAZARD_REQ_DOOR | HAZARD_REQ_BRAKE);
#endif
#if TRIGGER_HAZARD_ON_DOOR
    // Doors may already be open from when ACC was running - light up now.
    if (_enabled && _doorsOpen) _hazardDemands |= HAZARD_REQ_DOOR;
#endif
  }
}

/* ACC on the analog pin, debounced. One edge per transition. */
static void pollAcc(void) {
#if ACC_SOURCE_CAN
  if (_ignSeen) return;                           // the BCM's own ignition state wins
#endif
  bool on = (readAccMv() >= ACC_OFF_MV);
  if (on != _accOn) {
    if (!on) {
      // Low for ACC_OFF_DEBOUNCE_MS in a row and this is a real transition.
      if (_accLowSinceMs == 0) _accLowSinceMs = millis();
      if (millis() - _accLowSinceMs >= ACC_OFF_DEBOUNCE_MS) {
        _accLowSinceMs = 0;
        setAccState(false);
      }
    } else {
      _accLowSinceMs = 0;
      setAccState(true);
    }
  } else if (on) {
    _accLowSinceMs = 0;                          // stay primed for the next drop
  }
}

/* CAN receive pipe: drain a batch of frames and dispatch each one. */
static void readCanPipe(void) {
  for (uint8_t n = 0; n < CAN_DRAIN_PER_LOOP; n++) {
    if (CAN0.checkReceive() != CAN_MSGAVAIL) return;

    // 3-argument readMsgBuf() is the 1.5.x form; it flags extended IDs by ORing
    // CAN_IS_EXTENDED (bit 31) into the returned ID, which frameIdMatches() undoes.
    CAN0.readMsgBuf(&_rxFrameId, &_rxLen, _rxData);

#if TRIGGER_CAN_MONITOR
    if (_monitorOn) {
      Serial.print(F("RX 0x")); Serial.print(_rxFrameId & CAN_EXTENDED_ID, HEX);
      if (_rxFrameId & CAN_IS_EXTENDED) Serial.print(F(" ext"));
      Serial.print(F(" dlc=")); Serial.print(_rxLen);
      Serial.print(F(" "));
      for (uint8_t i = 0; i < _rxLen && i < 8; i++) {
        if (_rxData[i] < 0x10) Serial.print(F("0"));
        Serial.print(_rxData[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }
#endif

    handleStatusFrame();     // 0x60D: doors, lock status, ignition
    handleHeadlightFrame();  // 0x625: headlight read-back
#if TRIGGER_BRAKE_WARNING_LIGHTS
    handleSpeedFrame();      // 0x284: vehicle speed
#endif
  }
}

/* --- 0x60D x60D_BCM_GeneralStatus7 (BCM, 8 bytes) ----------------------------
 * Everything the module needs off a single frame: which doors are open, whether
 * the car is locked, and where the ignition switch is. */
static void handleStatusFrame(void) {
  if (!frameIdMatches(BCM_STATUS_ID, 0)) return;
  if (_rxLen == 0) return;

#if ACC_SOURCE_CAN && TRIGGER_CAN_IGNITION
  if (_rxLen > IGN_BYTE) pollCanIgnition();
#endif
#if TRIGGER_HAZARD_ON_DOOR
  if (_rxLen > DOOR_BYTE) handleDoorBits();
#endif
#if NEED_LOCK_STATUS
  if (_rxLen > LOCKSTAT_BYTE) handleLockBits();
#endif
}

#if ACC_SOURCE_CAN && TRIGGER_CAN_IGNITION
/* Debounced BCM_VehicleState -> ACC on / off. */
static void pollCanIgnition(void) {
  uint8_t v = (uint8_t)((_rxData[IGN_BYTE] & IGN_MASK) >> IGN_SHIFT);
  unsigned long now = millis();

  if (v != _ignPending) { _ignPending = v; _ignPendingMs = now; return; }
  if ((long)(now - _ignPendingMs) < (long)CAN_IGN_DEBOUNCE_MS) return;
  if (v == _ignLatched) return;

  _ignLatched = v;
  _ignSeen    = true;
  setAccState(v == IGN_ACC || v == IGN_START || v == IGN_ON);
}
#endif

#if TRIGGER_HAZARD_ON_DOOR
/* Debounced door/tailgate latch plus the hazard demand that follows it. */
static void handleDoorBits(void) {
  bool anyOpen = (_rxData[DOOR_BYTE] & DOOR_MASK) != 0;

  if (anyOpen == _doorsOpen) {
    _doorDebouncePending = false;               // settled state, nothing to do
    return;
  }
  if (!_doorDebouncePending) {
    _doorsChangeAtMs = millis();
    _doorDebouncePending = true;
    return;
  }
  if ((long)(millis() - _doorsChangeAtMs) < (long)DOOR_DEBOUNCE_MS) return;

  _doorDebouncePending = false;
  _doorsOpen = anyOpen;
  Serial.println(_doorsOpen ? F("Door/tailgate OPEN") : F("Door/tailgate CLOSED"));

  if (!_doorsOpen) {
    _hazardDemands &= (uint8_t)~HAZARD_REQ_DOOR;
    return;
  }

  /* Somebody is getting in, so abandon any pending automatic re-lock. */
#if TRIGGER_AUTO_RELOCK
  if (_relockArmed) {
    _relockArmed = false;
    Serial.println(F("Auto-relock cancelled: a door was opened"));
  }
#endif

  if (!_enabled) return;                        // master switch off: track only
  if (HAZARD_ONLY_WHEN_PARKED && _accOn) {
    Serial.println(F("(parked-only: hazards skipped while driving)"));
  } else {
    _hazardDemands |= HAZARD_REQ_DOOR;
  }
}
#endif

#if NEED_LOCK_STATUS
/* --- 0x60D lock bits: a level, so everything happens on the edge ----------- */
static void handleLockBits(void) {
  bool locked = (_rxData[LOCKSTAT_BYTE] & LOCKSTAT_MASK) != 0;

  if (!_lockStateValid) {
    // First frame after boot: adopt the state without reacting to it, otherwise
    // powering the module up in a locked car would set off a honk.
    _lockStateValid = true;
    _carLocked      = locked;
    Serial.println(locked ? F("Car LOCKED (at boot)") : F("Car UNLOCKED (at boot)"));
#if TRIGGER_AUTO_RELOCK && RELOCK_ARM_AT_BOOT
    if (!locked) armRelock();
#endif
    return;
  }
  if (locked == _carLocked) return;

  _carLocked = locked;
  Serial.println(locked ? F("Car LOCKED") : F("Car UNLOCKED"));

  if (locked) onCarLocked();
  else        onCarUnlocked();
}

static void onCarLocked(void) {
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
  _unlockCount = 0;                            // a fresh lock resets the burst counter
#endif
#if TRIGGER_AUTO_RELOCK
  _relockArmed = false;                        // locked by hand, the timer is moot
#endif
#if TRIGGER_HORN_ON_LOCK
  honk(HORN_PULSES_LOCK, "locked");
#endif
#if TRIGGER_DOOR_LOCK
  // Only react while the car is off: locking while driving should not close windows.
  if (_enabled && !_accOn) {
    Serial.println(F("Lock event while ACC off -> close"));
    scheduleClose(F("door lock"));
  }
#endif
}

static void onCarUnlocked(void) {
#if TRIGGER_HORN_ON_UNLOCK
  honk(HORN_PULSES_UNLOCK, "unlocked");
#endif
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
  handleUnlockBurst();
#endif
#if TRIGGER_AUTO_RELOCK
  armRelock();
#endif
}
#endif

#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
/* 3 unlocks inside TRIPLE_UNLOCK_WINDOW_MS -> roll the windows down. */
static void handleUnlockBurst(void) {
  if (_accOn) return;                            // parked / off only

  unsigned long now = millis();
  if (_unlockCount == 0 || now - _unlockWindowStartMs > TRIPLE_UNLOCK_WINDOW_MS) {
    _unlockWindowStartMs = now;                  // start a fresh window with this event
    _unlockCount = 1;
  } else {
    _unlockCount++;
  }

  Serial.print(F("Unlock burst: ")); Serial.println(_unlockCount);

  if (_unlockCount >= 3) {
    _unlockCount = 0;
    Serial.println(F("Triple unlock detected -> roll windows down"));
    scheduleRollDown(F("triple unlock"));
  }
}
#endif

/* --- 0x625 headlight read-back (diagnostics only) ---------------------------- */
static void handleHeadlightFrame(void) {
  if (!frameIdMatches(HEADLIGHT_STAT_ID, 0)) return;
  if (_rxLen <= HEADLIGHT_STAT_BYTE) return;

  uint8_t st  = _rxData[HEADLIGHT_STAT_BYTE];
  bool on     = (st & 0x40) && (st & 0x20);      // 0x60 = low beam, 0x68 = + fog
  if (!_headlightsSeen || on != _headlightsOn) {
    _headlightsSeen = true;
    _headlightsOn   = on;
    Serial.print(F("Headlight status 0x")); Serial.print(st, HEX);
    Serial.println(on ? F(" (on)") : F(" (off)"));
  }
}

/* --- 0x284 vehicle speed: a hard deceleration flashes the hazards ----------- */
#if TRIGGER_BRAKE_WARNING_LIGHTS
static void handleSpeedFrame(void) {
  if (!_enabled) return;                            // master switch off
  if (!frameIdMatches(SPEED_FRAME_ID, 0)) return;
  if (_rxLen <= SPEED_BYTE_HI) return;
  if (!_accOn) return;                             // car not running -> no brake warning

  unsigned long now = millis();
  unsigned long raw = (unsigned long)_rxData[SPEED_BYTE_LO]
                    | ((unsigned long)_rxData[SPEED_BYTE_HI] << 8);
  unsigned long spdX10 = (raw * 10UL) / SPEED_CMH_PER_COUNT;   // tenths of km/h

  if (_lastSpeedAtMs != 0) {
    unsigned long dt = now - _lastSpeedAtMs;     // unsigned diff is wrap-safe
    if (dt >= DECEL_MIN_DT_MS && dt <= DECEL_MAX_DT_MS && spdX10 < _lastSpeedX10) {
      unsigned long dropPerSecX10 = (_lastSpeedX10 - spdX10) * 1000UL / dt;

      if (dropPerSecX10 >= DECEL_THRESHOLD_KMPHS_X10 && spdX10 >= MIN_SPEED_KMH_X10
          && !_brakeHazardsActive && now >= _brakeCooldownUntilMs) {
        Serial.print(F("Hard deceleration: lost "));
        Serial.print((_lastSpeedX10 - spdX10) / 10UL);
        Serial.print(F("."));
        Serial.print((_lastSpeedX10 - spdX10) % 10UL);
        Serial.println(F(" km/h across the sample window"));
        startBrakeWarning(now);
      }
    }
  }

  _lastSpeedX10  = spdX10;
  _lastSpeedAtMs = now;
}

static void startBrakeWarning(unsigned long now) {
  _brakeHazardsActive   = true;
  _brakeHazardUntilMs   = now + BRAKE_HAZARD_MS;
  _brakeCooldownUntilMs = now + BRAKE_HAZARD_MS + BRAKE_COOLDOWN_MS;
  _hazardDemands |= HAZARD_REQ_BRAKE;
  Serial.print(F("Brake hazard ON for "));
  Serial.print(BRAKE_HAZARD_MS);
  Serial.println(F(" ms"));
}

/* Count down the brake burst; clearing the demand lets updateHazards turn off. */
static void pollBrakeWarning(void) {
  if (!_brakeHazardsActive) return;
  unsigned long now = millis();
  if ((long)(now - _brakeHazardUntilMs) >= 0) {   // wrap-safe timer
    _brakeHazardsActive = false;
    _hazardDemands &= (uint8_t)~HAZARD_REQ_BRAKE;
    Serial.println(F("Brake hazard OFF"));
  }
}
#endif

/* ================================= ACTIONS =================================== */

/* --- Horn: N x 0.5 s, driven from loop() so nothing blocks ----------------- */
#if TRIGGER_HORN_ON_LOCK || TRIGGER_HORN_ON_UNLOCK
static void honk(uint8_t pulses, const char *why) {
  if (!_enabled || pulses == 0) return;

  _hornPulsesLeft = pulses;
  _hornPhase      = HORN_ON;
  _hornNextMs     = millis() + HORN_PULSE_MS;
  bcmCommand(BCM_CMD_ID, LID_BEEP, BEEP_FUNC, BEEP_ON, "horn ON");

  Serial.print(F("Honking ")); Serial.print(pulses);
  Serial.print(F(" x ")); Serial.print(HORN_PULSE_MS);
  Serial.print(F(" ms  (")); Serial.print(why); Serial.println(F(")"));
}

static void pollHorn(void) {
  if (_hornPhase == HORN_IDLE) return;
  if ((long)(millis() - _hornNextMs) < 0) return;

  if (_hornPhase == HORN_ON) {
    bcmCommand(BCM_CMD_ID, LID_BEEP, BEEP_FUNC, BEEP_OFF, "horn OFF");
    if (_hornPulsesLeft > 1) {
      _hornPulsesLeft--;
      _hornPhase  = HORN_GAP;
      _hornNextMs = millis() + HORN_GAP_MS;
    } else {
      _hornPhase = HORN_IDLE;
    }
  } else {                                        // HORN_GAP -> next pulse
    _hornPhase  = HORN_ON;
    _hornNextMs = millis() + HORN_PULSE_MS;
    bcmCommand(BCM_CMD_ID, LID_BEEP, BEEP_FUNC, BEEP_ON, "horn ON");
  }
}

/* Safety: never leave the horn sounding after the master switch goes off. */
static void abortHorn(void) {
  if (_hornPhase == HORN_IDLE) return;
  bcmCommand(BCM_CMD_ID, LID_BEEP, BEEP_FUNC, BEEP_OFF, "horn OFF (aborted)");
  _hornPhase      = HORN_IDLE;
  _hornPulsesLeft = 0;
}
#endif

/* --- Automatic re-lock ------------------------------------------------------ */
#if TRIGGER_AUTO_RELOCK
static void armRelock(void) {
  if (_accOn) return;                            // driving -> no timer
  _relockArmed = true;
  _relockAtMs  = millis() + RELOCK_DELAY_MS;
  Serial.print(F("Auto-relock armed, "));
  Serial.print(RELOCK_DELAY_MS / 1000UL);
  Serial.println(F(" s to open a door"));
}

static void pollRelock(void) {
  if (!_relockArmed || !_enabled) return;
  if ((long)(millis() - _relockAtMs) < 0) return;

  _relockArmed = false;
  if (_accOn)     return;                        // started driving -> hands off
#if NEED_LOCK_STATUS
  if (_carLocked) return;                        // somebody locked it by hand
#endif
  Serial.println(F("Still unlocked and no door opened -> locking"));
  forceRelock();
}

/* Fire the lock command, re-arming is left to the lock-status edge. */
static void forceRelock(void) {
  bcmCommand(BCM_CMD_ID, LID_LOCK, LOCK_FUNC, LOCK_CMD_LOCK, "lock all (auto-relock)");
  // The lock bits coming back from the BCM raise the usual "Car LOCKED" edge,
  // which is also what plays the confirmation honk.
}

/* Re-arm the countdown when the master switch comes back on. */
static void reArmRelock(void) {
  if (_relockArmed) _relockAtMs = millis() + RELOCK_DELAY_MS;
}
#endif

/* Schedule one close sequence EVENT_COOLDOWN_MS after the previous one. */
static void scheduleClose(const char *reason) {
  unsigned long now = millis();
  if (now < _nextEventMs) {
    Serial.println(F("(cooldown active, skipping)"));
    return;
  }
  _nextEventMs = now + EVENT_COOLDOWN_MS;
  _closeAtMs   = now + CLOSE_DELAY_MS;
  _closeReason = reason;
  Serial.print(F("Will close windows in "));
  Serial.print(CLOSE_DELAY_MS);
  Serial.print(F(" ms  ("));
  Serial.print(reason);
  Serial.println(F(")"));
}

#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
/* Schedule one roll-down, also gated by the shared EVENT_COOLDOWN_MS. */
static void scheduleRollDown(const char *reason) {
  unsigned long now = millis();
  if (now < _nextEventMs) {
    Serial.println(F("(cooldown active, skipping)"));
    return;
  }
  _nextEventMs = now + EVENT_COOLDOWN_MS;
  _openAtMs    = now + ROLL_DOWN_DELAY_MS;
  _openReason  = reason;
  Serial.print(F("Will roll windows down in "));
  Serial.print(ROLL_DOWN_DELAY_MS);
  Serial.print(F(" ms  ("));
  Serial.print(reason);
  Serial.println(F(")"));
}

static void runPendingRollDown(void) {
  if (_openAtMs == 0) return;
  if (!_enabled) { _openAtMs = 0; return; }        // disabled mid-timer: cancel
  if ((long)(millis() - _openAtMs) < 0) return;   // not yet (millis wrap-safe)
  _openAtMs = 0;
  openAllWindows(_openReason);
}
#endif

static void runPendingClose(void) {
  if (_closeAtMs == 0) return;
  if (!_enabled) { _closeAtMs = 0; return; }       // disabled mid-timer: cancel
  if ((long)(millis() - _closeAtMs) < 0) return;   // not yet (millis wrap-safe)
  _closeAtMs = 0;
  closeAllWindows(_closeReason);
}

/* Send every "up" frame in WINDOWS_UP[]. The headlights cover the sequence. */
static void closeAllWindows(const char *reason) {
  Serial.print(F("Closing all windows ("));
  Serial.print(reason ? reason : "manual");
  Serial.println(F(")"));

#if LIGHTS_WHILE_CLOSE
  bcmCommand(BCM_CMD_ID, LID_BEAM, BEAM_FUNC, BEAM_ON, "headlights ON");
#endif

#if WINDOW_FRAMES_VERIFIED
  for (uint8_t w = 0; w < NUM_WINDOWS_UP; w++) {
    const WinFrame &wf = WINDOWS_UP[w];
    sendFrame(wf.id, wf.ext, wf.dlc, wf.data, wf.name);
    delay(CLOSE_SPACING_MS);
  }
#else
  Serial.println(F("!! window TX skipped: WINDOW_FRAMES_VERIFIED = 0"));
  Serial.println(F("!! no public Nissan window command frame exists - sniff yours"));
#endif

#if LIGHTS_WHILE_CLOSE
  delay(HEADLIGHTS_HOLD_MS);                     // let the last frame finish the roll
  bcmCommand(BCM_CMD_ID, LID_BEAM, BEAM_FUNC, BEAM_OFF, "headlights OFF");
#endif

  Serial.println(F("Close sequence done"));
}

#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
/* Send every "down" frame in WINDOWS_DOWN[]. */
static void openAllWindows(const char *reason) {
  Serial.print(F("Rolling all windows down ("));
  Serial.print(reason ? reason : "manual");
  Serial.println(F(")"));

#if WINDOW_FRAMES_VERIFIED
  for (uint8_t w = 0; w < NUM_WINDOWS_DOWN; w++) {
    const WinFrame &wf = WINDOWS_DOWN[w];
    sendFrame(wf.id, wf.ext, wf.dlc, wf.data, wf.name);
    delay(ROLL_DOWN_SPACING_MS);
  }
#else
  Serial.println(F("!! window TX skipped: WINDOW_FRAMES_VERIFIED = 0"));
#endif
  Serial.println(F("Roll-down sequence done"));
}
#endif

/* --- One BCM / IPDM $30 command, with the diagnostic session around it ------
 * The session is entered when needed and then reused for up to BCM_REUSE_MS, so a
 * burst (the two-honk unlock, the hazard alternation) does not re-send $10 C0
 * between every frame. The BCM drops back to the default session by itself after
 * ~5 s; serial 'x' calls bcmCloseSession() to hand it back immediately. */
static bool bcmCommand(uint32_t id, uint8_t lid, uint8_t func, uint8_t val,
                       const char *what) {
  static const uint8_t sessOn[8]  = { BCM_PCI_SESSION, BCM_SID_SESSION, BCM_SESSION_C0,
                                      0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t data[8] = { BCM_PCI_IO, BCM_SID_IO, lid, func, val, 0x00, 0x00, 0x00 };

  // A burst of commands (a double honk, the hazard alternation) reuses the open
  // session: the BCM keeps $30 alive for about 5 s and BCM_REUSE_MS stays under that.
  bool reuse = _sessionOpen && (id == _lastCmdId) &&
               ((long)(millis() - _lastCmdMs) < (long)BCM_REUSE_MS);

  if (!reuse) {
    sendFrame(id, 0, 8, sessOn, "diag session C0");
    delay(BCM_GAP_MS);
  }

  bool ok = sendFrame(id, 0, 8, data, what);
  delay(BCM_GAP_MS);

  _sessionOpen = true;
  _lastCmdId   = id;
  _lastCmdMs   = millis();
  return ok;
}

/* Hand the ECU back its default diagnostic session. */
static void bcmCloseSession(void) {
  static const uint8_t sessOff[8] = { BCM_PCI_SESSION, BCM_SID_SESSION, BCM_SESSION_DEF,
                                      0x00, 0x00, 0x00, 0x00, 0x00 };
  if (!_sessionOpen) return;
  sendFrame(_lastCmdId, 0, 8, sessOff, "diag session default");
  _sessionOpen = false;
}

/* Generic CAN frame sender with retries + status LED confirmation. */
static bool sendFrame(uint32_t id, uint8_t ext, uint8_t dlc,
                      const uint8_t *data, const char *what) {
  uint32_t frameId = ext ? (id | CAN_IS_EXTENDED) : id;

  for (uint8_t attempt = 1; attempt <= SEND_RETRIES; attempt++) {
    byte result = CAN0.sendMsgBuf(frameId, ext, dlc, (INT8U *)data);

    if (result == CAN_OK) {
      Serial.print(F("  sent ")); Serial.print(what);
      Serial.print(F("  ID=0x")); Serial.print(id, HEX);
      Serial.print(F(" data="));
      for (uint8_t i = 0; i < dlc; i++) {
        if (data[i] < 0x10) Serial.print(F("0"));
        Serial.print(data[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
      flashLed(80);                              // non-blocking status LED tick
      return true;
    }
    Serial.print(F("  tx failed ")); Serial.print(what);
    Serial.print(F(" (attempt ")); Serial.print(attempt); Serial.println(F(")"));
    delay(50);
  }
  Serial.print(F("  giving up on ")); Serial.println(what);
  return false;
}

/* Reconcile all hazard requests (door / brake / manual) into BCM $30 PID 0x4C.
 * That PID drives one side at a time, so "hazards" is a left/right alternation. */
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
static void updateHazards(void) {
  bool want = (_hazardDemands != 0);
  unsigned long now = millis();

  if (!want) {
    if (_hazardPhase != 0) {
      _hazardPhase = 0;
      bcmCommand(BCM_CMD_ID, LID_TURN, TURN_FUNC, TURN_OFF, "hazards OFF");
    }
    _hazardsOn     = false;
    _hazardsWanted = false;
    return;
  }

  if (!_hazardsWanted) _nextHazardToggleMs = 0;   // react on the first demand
  _hazardsWanted = true;
  _hazardsOn     = true;

  if ((long)(now - _nextHazardToggleMs) < 0) return;
  _nextHazardToggleMs = now + HAZARD_TOGGLE_MS;

  _hazardPhase = (_hazardPhase == 1) ? 2 : 1;
  bcmCommand(BCM_CMD_ID, LID_TURN, TURN_FUNC,
             (_hazardPhase == 1) ? TURN_RIGHT : TURN_LEFT,
             (_hazardPhase == 1) ? "hazards RIGHT" : "hazards LEFT");
}
#endif

/* ================================ HELPERS ==================================== */

/* Does the received frame (in _rxFrameId) match an ID/ext pair? */
static bool frameIdMatches(uint32_t id, uint8_t ext) {
  uint32_t normId = _rxFrameId & CAN_EXTENDED_ID;         // strip the ext flag bit31
  bool normExt   = (_rxFrameId & CAN_IS_EXTENDED) != 0;
  return (normId == id) && (normExt == (ext != 0));
}

/* ACC voltage in millivolts (5 V reference, 10-bit ADC). */
static unsigned long readAccMv(void) {
  return (unsigned long)analogRead(ACC_SENSE_PIN) * 5000UL / 1023UL;
}

/* Non-blocking status-LED flash, used as a "frame went out" tick. */
static void flashLed(uint16_t ms) {
  ledOn();
  _ledFlashing = true;
  _ledAtMs     = millis() + ms;
}

static void pollLed(void) {
  if (!_ledFlashing) return;
  if ((long)(millis() - _ledAtMs) >= 0) {
    _ledFlashing = false;
    ledOff();
  }
}

/* Physical kill-switch (PCB J5): low -> every automatic feature pauses. */
#if MASTER_ENABLE_PIN >= 0
static void pollMasterEnable(void) {
  bool en = (digitalRead(MASTER_ENABLE_PIN) == HIGH);
  if (en == _enabled) return;

  _enabled = en;
  Serial.println(en ? F("Master enable ON")
                    : F("Master enable OFF (all auto features paused)"));
#if TRIGGER_HORN_ON_LOCK || TRIGGER_HORN_ON_UNLOCK
  if (!en) abortHorn();
#endif
#if TRIGGER_AUTO_RELOCK
  if (en) reArmRelock();
#endif
}
#endif

/* Blocking blink, only for the fatal CAN-init path. */
static void blink(uint8_t times, uint16_t halfPeriodMs) {
  for (uint8_t i = 0; i < times; i++) {
    ledOn();  delay(halfPeriodMs);
    ledOff(); delay(halfPeriodMs);
  }
}

/* USB serial test harness (handy before wiring the car). */
static void handleSerial(void) {
  if (!Serial.available()) return;

  char c = (char)Serial.read();
  switch (c) {
    case 'c':
    case 'C':
      closeAllWindows("serial-cmd");
      break;
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
    case 'o':
    case 'O':
      openAllWindows("serial-cmd");
      break;
#endif
    case 'l':
    case 'L':
      bcmCommand(BCM_CMD_ID, LID_LOCK, LOCK_FUNC, LOCK_CMD_LOCK, "lock all (serial)");
      break;
    case 'u':
    case 'U':
      bcmCommand(BCM_CMD_ID, LID_LOCK, LOCK_FUNC, LOCK_CMD_UNLOCK, "unlock all (serial)");
      break;
#if TRIGGER_HORN_ON_LOCK || TRIGGER_HORN_ON_UNLOCK
    case 'p':
    case 'P':
      honk(1, "serial-cmd");
      break;
    case 'n':
    case 'N':
      honk(2, "serial-cmd");
      break;
#endif
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
    case 'd':
    case 'D':
      _hazardDemands |= HAZARD_REQ_MANUAL;       // reconciled by loop()'s updateHazards()
      break;
    case 'f':
    case 'F':
      _hazardDemands &= (uint8_t)~HAZARD_REQ_MANUAL;
      break;
#endif
#if TRIGGER_AUTO_RELOCK
    case 'r':
    case 'R':
      _relockArmed = false;
      forceRelock();
      break;
#endif
    case 'x':
    case 'X':
      bcmCloseSession();
      break;
    case 'a':
    case 'A':
      Serial.print(F("ACC mV: ")); Serial.println(readAccMv());
      break;
    case 'm':
    case 'M':
#if TRIGGER_CAN_MONITOR
      _monitorOn = !_monitorOn;
      Serial.println(_monitorOn ? F("CAN monitor on") : F("CAN monitor off"));
#else
      Serial.println(F("CAN monitor compiled out (TRIGGER_CAN_MONITOR=0)"));
#endif
      break;
    case 's':
    case 'S':
      Serial.print(F("ACC: ")); Serial.println(_accOn ? F("ON") : F("OFF"));
#if ACC_SOURCE_CAN
      Serial.print(F("  from CAN: ")); Serial.println(_ignSeen ? F("yes") : F("no (analog)"));
#endif
#if NEED_LOCK_STATUS
      Serial.print(F("Locked: ")); Serial.println(_carLocked ? F("yes") : F("no"));
#endif
#if TRIGGER_ROLL_DOWN_ON_TRIPLE_UNLOCK
      Serial.print(F("Unlock burst: ")); Serial.println(_unlockCount);
#endif
#if TRIGGER_HAZARD_ON_DOOR
      Serial.print(F("Door/tailgate open: ")); Serial.println(_doorsOpen ? F("yes") : F("no"));
#endif
#if TRIGGER_HAZARD_ON_DOOR || TRIGGER_BRAKE_WARNING_LIGHTS
      Serial.print(F("Hazard demands: 0x")); Serial.println(_hazardDemands, HEX);
      Serial.print(F("Hazards active: "));   Serial.println(_hazardsOn ? F("yes") : F("no"));
      Serial.print(F("Hazard side: "));
      Serial.println(_hazardPhase == 1 ? F("right")
                          : (_hazardPhase == 2 ? F("left") : F("off")));
#endif
#if TRIGGER_BRAKE_WARNING_LIGHTS
      Serial.print(F("Last speed km/h: "));
      Serial.print(_lastSpeedX10 / 10UL); Serial.print(F("."));
      Serial.print(_lastSpeedX10 % 10UL);
      Serial.println();
      Serial.print(F("Brake hazard active: ")); Serial.println(_brakeHazardsActive ? F("yes") : F("no"));
#endif
#if TRIGGER_HORN_ON_LOCK || TRIGGER_HORN_ON_UNLOCK
      Serial.print(F("Horn: "));
      Serial.println(_hornPhase == HORN_IDLE ? F("idle")
                          : (_hornPhase == HORN_ON ? F("sounding") : F("gap")));
#endif
#if TRIGGER_AUTO_RELOCK
      Serial.print(F("Auto-relock armed: ")); Serial.println(_relockArmed ? F("yes") : F("no"));
      if (_relockArmed) {
        long left = (long)(_relockAtMs - millis());
        Serial.print(F("  fires in "));
        Serial.println(left > 0 ? (unsigned long)(left / 1000L) : 0UL);
      }
#endif
      if (_headlightsSeen) {
        Serial.print(F("Headlights: ")); Serial.println(_headlightsOn ? F("on") : F("off"));
      }
      Serial.print(F("Diag session open: ")); Serial.println(_sessionOpen ? F("yes") : F("no"));
      Serial.print(F("Enabled: ")); Serial.println(_enabled ? F("yes") : F("no"));
      break;
    case 'h':
    case 'H':
    case '?':
      Serial.println(F("commands: c=close, o=roll down, l=lock, u=unlock, p=1 honk,"));
      Serial.println(F("           n=2 honks, r=relock now, d=hazards on, f=hazards off,"));
      Serial.println(F("           a=ACC mV, m=monitor, x=close diag session, s=status, ?=help"));
      break;
    default:
      break;
  }
}
