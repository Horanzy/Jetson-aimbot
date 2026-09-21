# P5 General (PS5 Authentication Dongle) USB Protocol Specification

This document is reverse-engineered from the GP2040-CE firmware source and describes the complete USB-protocol-level behavior of the P5 General dongle: the enumerated device identity, the authentication handshake (challenge–response), the runtime data flow, and all timings. Its goal is to enable a reader to implement, from scratch on any platform (e.g. Linux + raw_gadget or a standalone USB host stack), a driver that interoperates with the P5 General.

Every claim carries a source citation `file:line` (paths relative to the GP2040-CE repository root). Items marked **[code]** are direct code evidence; items marked **[inferred]** are conclusions drawn from the code structure and are explicitly labeled as such. Everything that cannot be determined from the code is collected in Section 10; this document makes no guesses about them.

---

## Table of Contents

1. [System Topology](#1-system-topology)
2. [Enumeration: the USB Device Presented to the PS5](#2-enumeration-the-usb-device-presented-to-the-ps5)
3. [HID Report Overview and Report Descriptor Decode](#3-hid-report-overview-and-report-descriptor-decode)
4. [Input Report 0x01, Byte by Byte](#4-input-report-0x01-byte-by-byte)
5. [Authentication Handshake (Control Transfers)](#5-authentication-handshake-control-transfers)
6. [Runtime Data Flow: the Input-Report Signing Pipeline](#6-runtime-data-flow-the-input-report-signing-pipeline)
7. [Dongle-Side Interfacing Contract (Host Implementer's Guide)](#7-dongle-side-interfacing-contract-host-implementers-guide)
8. [DualSense Host Input Path](#8-dualsense-host-input-path)
9. [Timing, Retry and Failure-Path Summary](#9-timing-retry-and-failure-path-summary)
10. [Items That Could Not Be Determined from Code](#10-items-that-could-not-be-determined-from-code)
11. [Configuration Semantics and User-Visible Behavior](#11-configuration-semantics-and-user-visible-behavior)
12. [Implementation Checklist](#12-implementation-checklist)
13. [Source Evidence Inventory](#13-source-evidence-inventory)

**One-sentence summary**: In P5GENERAL input mode the GP2040-CE firmware presents itself to the PS5 as an HID gamepad with the shape of a P5 General dongle (VID 0x2B81 / PID 0x0101), while simultaneously driving a real P5 General dongle from its PIO USB host port: it relays the PS5's authentication challenges to the dongle verbatim, relays the dongle's answers back verbatim, and every 64-byte input report sent to the PS5 must first be delivered to the dongle and re-received from it (the trailing 8-byte `hash` field being filled in by the dongle) before it may be transmitted.

---

## 1. System Topology

```
                 USB bus A (RP2040 hardware USB, device role, full speed)
  PS5 console ◄────────────────────────────────────────────────►  GP2040 board
  (USB host)         enumerates an "P5General" HID gamepad        (USB device,
             VID 0x2B81 / PID 0x0101                               core0 runs the
             IN EP 0x82 (interrupt, 64B, 1 ms)                      TinyUSB device stack)
             OUT EP 0x01 (interrupt, 64B, 6 ms)
             control endpoint: feature reports 0x03/0xF0/0xF1/0xF2
                                                                │
                                                                │ button/stick state (on-board GPIO/ADC)
                                                                ▼
                                                     P5GeneralDriver::process()
                                                     assembles the 64B P5GenerorReport
                                                                │
                 USB bus B (PIO USB, host role, full speed)      │
  P5 General dongle ◄────────────────────────────────────────────┘
  (USB device,        enumeration match: VID 0x2B81 / PID 0x0101
   plugged into        IN EP: dongle → GP2040 (continuously polled, 64B interrupt reports)
   GP2040's PIO        OUT EP: GP2040 → dongle (64B input-report state data)
   host port)          control endpoint: SET_REPORT(F0) / GET_REPORT(F1) / GET_REPORT(F2)
```

Topological conclusions (all **[code]** evidence):

- The GP2040 board plays two USB roles at once: a **USB device** toward the PS5 (`src/gp2040.cpp:255` starts the TinyUSB device stack with `tud_init(TUD_OPT_RHPORT)`; the device descriptor comes from `P5GeneralDriver::get_descriptor_device_cb`, `src/drivers/p5general/P5GeneralDriver.cpp:319-322`) and a **USB host** toward the dongle (`src/gp2040.cpp:258` → `USBHostManager::start()`, which configures the PIO USB port with `tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, ...)` + `tuh_init(BOARD_TUH_RHPORT)` in `src/usbhostmanager.cpp:14-25`; `BOARD_TUH_RHPORT=1` at `headers/tusb_config.h:46`).
- The dongle is recognized on the host port by exactly one condition: `VID == 0x2B81 && PID == 0x0101` (`src/drivers/p5general/P5GeneralAuthUSBListener.cpp:91-99`). The first HID interface instance enumerating with that VID/PID is taken as the dongle (`dongle_ready = true`).
- The device the PS5 sees carries the **same VID/PID** as the dongle (device descriptor bytes `0x81, 0x2B` / `0x01, 0x01`, little-endian 0x2B81/0x0101, `headers/drivers/p5general/P5GeneralDescriptors.h:133-134`). **[inferred]** The GP2040 device side is therefore a functional clone of the dongle's host-facing interface: the PS5 treats the GP2040 as a directly attached P5 General device, while all real cryptographic work happens inside the dongle on the host port. The source macro `#define P5GENERAL_VENDOR_ID 0x28B1` (`P5GeneralDescriptors.h:14`) disagrees with the descriptor bytes (which encode 0x2B81, and the host-side match also uses 0x2B81); the macro is referenced by no matching logic and is a source-level typo — the on-wire truth is 0x2B81.
- A real DualSense (Sony 054c/0CE6) is not part of the P5 authentication chain. `DualsensePS5Host` is a branch of the generic "plug a pad into the host port" addon (`src/addons/gamepad_usb_host_listener.cpp:49-50`), coexisting with the auth listener; each claims its own VID/PID (the auth listener accepts only 0x2B81/0x0101, `DualsensePS5Host` only 0x054c/0x0CE6, `src/hosts/DualsensePS5Host.cpp:3-13`). The PIO host port hosts one device at a time; coexistence of both would require an external hub, which the code does not address. See Section 8.
- Core split: core0 runs the main loop — read buttons, assemble the report, `inputDriver->process(gamepad)`, `tud_task()`, `USBHostManager::process()` (i.e. `tuh_task()`) (`src/gp2040.cpp:249-313`, specifically lines 277, 304, 308); core1 runs the auxiliary loop — `inputDriver->processAux()`, i.e. the authentication state-machine poll (`src/gp2040aux.cpp:53-64`). The two sides exchange data through the shared `P5GeneralAuthData` structure, without locks (implementation note in Section 9).

---

## 2. Enumeration: the USB Device Presented to the PS5

These descriptors are returned by TinyUSB callbacks during standard `GET_DESCRIPTOR` requests (routing: `src/usbdriver.cpp:74-100` `tud_descriptor_*_cb` → `DriverManager::getDriver()` virtual functions → this driver's static arrays).

### 2.1 Device Descriptor (18 bytes)

Source: `headers/drivers/p5general/P5GeneralDescriptors.h:124-140`.

| Offset | Field | Value | Notes |
|---|---|---|---|
| 0 | bLength | 0x12 | |
| 1 | bDescriptorType | 0x01 | Device |
| 2-3 | bcdUSB | 0x0200 | USB 2.00 |
| 4 | bDeviceClass | 0x00 | class defined at interface level |
| 5 | bDeviceSubClass | 0x00 | |
| 6 | bDeviceProtocol | 0x00 | |
| 7 | bMaxPacketSize0 | 0x40 | 64-byte control endpoint |
| 8-9 | idVendor | **0x2B81** | byte order `81 2B` |
| 10-11 | idProduct | **0x0101** | byte order `01 01` |
| 12-13 | bcdDevice | 0x0001 | |
| 14 | iManufacturer | 0x01 | string "Activtor" (sic) |
| 15 | iProduct | 0x02 | string "P5General" |
| 16 | iSerialNumber | 0x00 | none |
| 17 | bNumConfigurations | 0x01 | |

String table (`P5GeneralDescriptors.h:111-122`): index 0 = language 0x0409 (US English); 1 = `Activtor`; 2 = `P5General`; 3 = `0.1`. String descriptors are assembled at runtime into USB UTF-16LE form by `getStringDescriptor()` (`headers/drivers/shared/driverhelper.h:4-25`; routing entry at `P5GeneralDriver.cpp:313-317`).

### 2.2 Configuration Descriptor (41 bytes, one config, one interface, two endpoints)

Source: `headers/drivers/p5general/P5GeneralDescriptors.h:236-278`.

| Segment | Field | Value |
|---|---|---|
| Configuration | wTotalLength | 41 (0x29) |
| | bNumInterfaces | 1 |
| | bConfigurationValue | 1 |
| | bmAttributes | 0x80 (bus-powered; remote-wakeup bit D5 **not** set) |
| | bMaxPower | 0xFA = 500 mA |
| Interface #0 | bAlternateSetting | 0 |
| | bNumEndpoints | 2 |
| | bInterfaceClass / SubClass / Protocol | 0x03 / 0x00 / 0x00 (HID, no subclass/protocol) |
| | iInterface | 0 |
| HID | bcdHID | 0x0111 (1.11) |
| | bCountryCode | 0 |
| | bNumDescriptors / type / length | 1 / 0x22 (Report) / **165 (0xA5)** |
| Endpoint | bEndpointAddress | **0x82** (IN, endpoint 2) |
| | bmAttributes | 0x03 (interrupt) |
| | wMaxPacketSize | 64 |
| | bInterval | **1** (1 ms at full speed) |
| Endpoint | bEndpointAddress | **0x01** (OUT, endpoint 1) |
| | bmAttributes | 0x03 (interrupt) |
| | wMaxPacketSize | 64 |
| | bInterval | **6** (6 ms at full speed) |

The report-descriptor length of 165 bytes was independently verified to match the array's actual byte count **[code]** (`P5GeneralDescriptors.h:142-223`; exactly 165 bytes once comments are stripped).

One implementation-level fact: the configuration descriptor does not declare remote-wakeup support, yet the driver calls `tud_remote_wakeup()` while the bus is suspended (`src/drivers/p5general/P5GeneralDriver.cpp:110-112`). **[inferred]** The practical effect of that call on a host that was not told D5 is set is not validated anywhere in the code.

### 2.3 Report Rate

- Input reports toward the PS5: bounded by the IN endpoint's `bInterval=1` (1 ms), i.e. up to 1000 Hz in theory; the actual cadence is set by the "state change → dongle signing → send" pipeline (Section 6). When idle (no input change) **no input reports are sent at all** **[code]** (`P5GeneralDriver.cpp:229-242`: identical report with the 4-repeat quota exhausted returns false, producing no transfer).
- Toward the dongle: the host side uses TinyUSB's interrupt-IN polling model — it queues a receive on mount and re-queues after every delivered report (`src/usbhostmanager.cpp:108-114`, `141-148`); the cadence is set by the dongle's own endpoint `bInterval` (its descriptor is never read by this code, see Section 10).

---

## 3. HID Report Overview and Report Descriptor Decode

The report descriptor (`headers/drivers/p5general/P5GeneralDescriptors.h:142-223`) declares two top-level Application Collections:

**Collection 1: Generic Desktop / Game Pad**

| Report ID | Type | Length (on wire, incl. ID byte) | Declared Usage | Implemented by driver? |
|---|---|---|---|---|
| 0x01 | Input | 64 | see 3.1 | Yes (Sections 4, 6) |
| 0x02 | Output | 48 | Vendor Page 0xFF00 Usage 0x23, 47 bytes | **No** — OUT endpoint data is discarded by the driver (`P5GeneralDriver.cpp:297-299` returns immediately for non-Feature types) |
| 0x03 | Feature | 48 | Vendor Page 0xFF00 Usage 0x2821, 47 bytes | Yes — returns the static 47-byte `output_0x03` (Section 5.2) |
| 0xE0 | Feature | 3 | Vendor Page 0xFF80 Usage 0x57, 2 bytes | **No** — the driver has no case for it; `GET_REPORT` STALLs (`P5GeneralDriver.cpp:269-289` has no matching branch and returns -1; TinyUSB's zero-length assertion then stalls, `lib/tinyusb/src/class/hid/hid_device.c:305-306`) |

**Collection 2: Vendor Page 0xFFF0 / Usage 0x40** (authentication only)

| Report ID | Type | Length (on wire, incl. ID byte) | Usage | Purpose |
|---|---|---|---|---|
| 0xF0 | Feature | 64 | 0x47, 63 bytes | auth payload load (SET) |
| 0xF1 | Feature | 64 | 0x48, 63 bytes | read signature/nonce (GET) |
| 0xF2 | Feature | 16 | 0x49, 15 bytes | read signing state (GET) |

### 3.1 Bit-level layout of input report 0x01 (report-descriptor view)

`P5GeneralDescriptors.h:144-192` declares, in order (after report ID 0x01):

1. six 8-bit absolute axes (Usages X, Y, Z, Rz, Rx, Ry, logical 0–255) → bytes 1–6;
2. Vendor Page 0xFF00 Usage 0x20, 8 bits ×1 → byte 7 (the struct's `reportCounter`);
3. a 4-bit hat switch (logical 0–7, physical 0–315°, Null State);
4. Button page Usages 1–14, 1 bit ×14;
5. Vendor Page 0xFF00 Usage 0x21, 1 bit ×14 (pure padding bits covering the top 6 bits of byte 10 plus all 8 bits of byte 11);
6. Vendor Page 0xFF00 Usage 0x22, 8 bits ×52 → bytes 12–63.

Bit packing **[code]** (descriptor order + `P5GenerorReport` bitfields, `P5GeneralDescriptors.h:62-109`): byte 8 = hat(bits 0–3) ‖ west/south/east/north(bits 4–7); byte 9 = l1/r1/l2/r2/select/start/l3/r3; byte 10 bits 0–1 = home/touchpad, bits 2–7 are Usage 0x21 padding; byte 11 = Usage 0x21 padding (struct field `data_11`).

---

## 4. Input Report 0x01, Byte by Byte

Struct definition: `headers/drivers/p5general/P5GeneralDescriptors.h:62-109` (`packed, aligned(1)`, 64 bytes total). Fill logic: `src/drivers/p5general/P5GeneralDriver.cpp:127-227`.

| Offset | Size | Field | Semantics | Source |
|---|---|---|---|---|
| 0 | 1 | report_id | constant 0x01 | `P5GeneralDriver.cpp:52` |
| 1 | 1 | left_stick_x | 8-bit, midpoint 0x80 (`P5GENERAL_JOYSTICK_MID`); truncated from the 16-bit internal state by `>> 8` | `P5GeneralDriver.cpp:156` |
| 2 | 1 | left_stick_y | same | :157 |
| 3 | 1 | right_stick_x | same | :158 |
| 4 | 1 | right_stick_y | same | :159 |
| 5 | 1 | left_trigger | analog trigger 0–255; 0xFF when digital L2 is pressed on hardware without analog triggers | `P5GeneralDriver.cpp:160-166` |
| 6 | 1 | right_trigger | same | same |
| 7 | 1 | reportCounter | **never incremented by the GP2040** (initialized to 0, never written again); the DualSense host path uses it for dedup (Section 8) | `P5GeneralDriver.cpp:51-66` (no write site after init) |
| 8 | 4b+4b | dpad ‖ west,south,east,north | hat 0–7 / 0x0F neutral (`P5GENERAL_HAT_*`, `P5GeneralDescriptors.h:18-26`) | `P5GeneralDriver.cpp:129-145` |
| 9 | 8 bit | l1,r1,l2,r2,select,start,l3,r3 | bit order as listed | `P5GeneralDriver.cpp:147-153` |
| 10 | 2 bit + 6 bit | home(bit0), touchpad(bit1) ‖ padding(bits 2–7) | | `P5GeneralDriver.cpp:154-155` |
| 11 | 1 | data_11 | always 0 | struct zero value |
| 12–15 | 4 | auth_seq_number | always 0, never written | same |
| 16–21 | 6 | gyroscope | 3 × int16; written with a byte swap of the internal aux state (`(lo<<8)\|(hi>>8)`, `P5GeneralDriver.cpp:169-173`), i.e. on-wire byte order is reversed relative to the aux state | :169-173 |
| 22–27 | 6 | accelerometer | same | :176-180 |
| 28–29 | 2 | data_28_29 | always 0 | struct zero value |
| 30–31 | 2 | data_30_31_0x001a | **fixed 0x001A** (little-endian on-wire bytes `1A 00`), written once at initialization | `P5GeneralDriver.cpp:64` |
| 32–35 | 4 | touchpad_data.p1 | see below | `PS4Descriptors.h:107-129` (TouchpadXY/TouchpadData, pulled in via `P5GeneralDescriptors.h`'s include); `P5GeneralDriver.cpp:183-207` |
| 36–39 | 4 | touchpad_data.p2 | see below | same |
| 40–55 | 16 | data_40_55 | always 0 | struct zero value |
| 56–63 | 8 | hash | **never written by the GP2040**. The field name matches the shared structure's `hash_pending_buffer` / `hash_finish_buffer`; **[inferred]** it is filled in by the dongle in the returned report and is the per-report authenticator the PS5 validates (Sections 6, 10) | struct zero value + data flow (`P5GeneralAuthUSBListener.cpp:115-120`) |

TouchpadXY (4 bytes per point, `PS4Descriptors.h:107-124`, pulled in via `P5GeneralDescriptors.h`'s include):

| Byte | Bits | Meaning |
|---|---|---|
| +0 | bit0–6 | counter: touch counter, +1 per new touch, wraps at 128 (`P5GENERAL_TP_MAX_COUNT`, `P5GeneralDriver.cpp:209-226`) |
| +0 | bit7 | unpressed: 1 = not touched |
| +1 | – | X low 8 bits |
| +2 | low nibble / high nibble | X high 4 bits / Y low 4 bits |
| +3 | – | Y high 8 bits |

X/Y are 12-bit with 1920 × 943 resolution (`P5GENERAL_TP_X_MAX / Y_MAX`, `P5GeneralDescriptors.h:50-53`). Touchpad-button emulation when no real touch sensor exists: A2 = touchpad pressed (center), A3 = left, A4 = right (`P5GeneralDriver.cpp:182-193`); the `switchTpShareForDs4` option swaps Select/touchpad semantics (`P5GeneralDriver.cpp:150-155`).

---

## 5. Authentication Handshake (Control Transfers)

### 5.1 Control-Transfer Encoding

All authentication exchanges are HID class requests with interface recipient, wrapped by TinyUSB into standard control transfers **[code]**:

| Direction | bmRequestType | bRequest | wValue | wIndex | wLength | Scenario |
|---|---|---|---|---|---|---|
| PS5 → GP2040 (data OUT) | 0x21 | 0x09 (SET_REPORT) | 0x03F0 (Feature << 8 \| 0xF0) | 0 (interface) | 64 | challenge load |
| PS5 → GP2040 (data IN) | 0xA1 | 0x01 (GET_REPORT) | 0x0303 | 0 | 48 | read definition |
| PS5 → GP2040 (data IN) | 0xA1 | 0x01 | 0x03F1 | 0 | 64 | read signature/nonce |
| PS5 → GP2040 (data IN) | 0xA1 | 0x01 | 0x03F2 | 0 | 16 | read signing state |
| GP2040 → dongle (data OUT) | 0x21 | 0x09 | 0x03F0 | dongle interface | 64 | challenge relay |
| GP2040 → dongle (data IN) | 0xA1 | 0x01 | 0x03F1 | same | 64 | fetch signature/nonce |
| GP2040 → dongle (data IN) | 0xA1 | 0x01 | 0x03F2 | same | 16 | fetch signing state |

Encoding basis: device side `lib/tinyusb/src/class/hid/hid_device.c:289-331` (GET_REPORT prepends the 1-byte report ID before invoking the callback; SET_REPORT strips a matching report-ID byte at the ACK stage, :299-303, :324-327); host side `lib/tinyusb/src/class/hid/hid_host.c:240-311` (`tuh_hid_get_report` / `tuh_hid_set_report` build bmRequestType IN/0xA1 and OUT/0x21 respectively, with `wValue = tu_u16(report_type, report_id)` and `wIndex` = the peer's interface number); the Feature type value is 3 (`lib/tinyusb/src/class/hid/hid.h:86-89`). The device-side control buffer is 64 bytes (`CFG_TUD_HID_EP_BUFSIZE` default, `lib/tinyusb/src/class/hid/hid_device.h:46-48`), so a SET_REPORT's wLength must not exceed 64.

**Data-stage byte layout (all feature reports)**: on the wire, byte 0 = report ID, followed by the payload. The device-side callback sees the buffer after TinyUSB has stripped the ID byte (SET) / writes after the ID byte (GET).

### 5.2 Feature 0x03: device definition (static answer)

PS5 `GET_REPORT(Feature, 0x03)` → 48 bytes on the wire = `[0x03][47-byte output_0x03]`. `output_0x03` (`src/drivers/p5general/P5GeneralDriver.cpp:19-26`, 47 bytes):

```
21 28 03 C3 00 2C 56
01 00 D0 07 00 80 04 00
00 80 0D 0D 84 00 00 00
00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00
```

**[code]**: bytes 0–1 = `21 28`, matching the little-endian form of this report's Usage 0x2821 from the report descriptor; the response length is gated by `reqlen >= 47` and the normal path (wLength=48) yields exactly 47 bytes (`P5GeneralDriver.cpp:270-276`). **[inferred]**: the remaining bytes' semantics (a device-capability/configuration bitmap) share provenance with the PS4 driver's same-numbered feature report — the PS4 `output_0x03`/`controllerConfig` (`src/drivers/ps4/PS4Driver.cpp:29-46`, `:283-297`) likewise begins with a usage echo (`21 27` ↔ Usage 0x2721), with `0D 0D` and `0D 84` at corresponding offsets; the P5 variant should be the same kind of controller-capability statement. Bit-level meaning is not recoverable from the code.

### 5.3 Feature 0xF0: challenge load (PS5 → device → dongle)

1. PS5 `SET_REPORT(Feature, 0xF0)`, data 64 bytes: `[0xF0][63-byte challenge payload]`.
2. The device-side callback requires `bufsize == 63` (TinyUSB has already stripped the ID byte; equivalent to wLength = 64 on the wire), otherwise the request is silently ignored (`P5GeneralDriver.cpp:301-304`); additionally the state machine must be in `p5g_auth_idle`, otherwise it is likewise ignored (`:305`).
3. On acceptance: `auth_buffer = [0xF0][63-byte payload]` (64 bytes), state → `p5g_auth_send_f0` (`P5GeneralDriver.cpp:306-309`).
4. core1 polls into `send_f0`: sends the 64-byte `auth_buffer` verbatim to the dongle as `SET_REPORT(Feature, 0xF0)` (`P5GeneralAuthUSBListener.cpp:44-49`, `:79-82`) — a **byte-for-byte relay with no validation and no modification** (contrast: the PS4 driver first validates the payload's trailing CRC32, `src/drivers/ps4/PS4Driver.cpp:893-899`; the P5 path includes `CRC32.h` but never uses it, `P5GeneralAuthUSBListener.cpp:5`). State → `send_f0_wait`.
5. The dongle ACKs (host-side `set_report_complete`, dispatched only when len = 64 ≠ 0 on success, `src/usbhostmanager.cpp:151-154`) → the state machine dispatches on `auth_buffer[1]` (payload byte 1, the **type byte**) and `auth_buffer[3]` (payload byte 3) (`P5GeneralAuthUSBListener.cpp:129-160`):

| auth_buffer[1] | f1_num (F1 fetch quota) | Auto F2 (after 500 ms)? | Next state |
|---|---|---|---|
| 0x01 (and auth_buffer[3] == 3) | 4 | yes | `recv_f2_delay_500mS` |
| 0x01 (and auth_buffer[3] != 3) | 4 | no | `idle` (wait for the PS5 to read F1) |
| 0x02 | 0 | yes | `recv_f2_delay_500mS` |
| 0x03 | 1 | yes | `recv_f2_delay_500mS` |
| other | 0 | no | `idle` |

**[inferred]** (from the isomorphic PS4 protocol): the type byte corresponds to an authentication phase number (nonce_id), with 0x01 = nonce load and 0x02/0x03 = subsequent signature exchanges; payload byte 2 (auth_buffer[2]) is the nonce page number in the PS4 protocol. The P5 code consumes only the two branch conditions above and assigns them no meaning.

### 5.4 Feature 0xF1: signature/nonce read (fetch-then-serve pipeline)

- PS5 `GET_REPORT(Feature, 0xF1)` → 64 bytes on the wire = `[0xF1][63 bytes]`, taken from the shared `auth_buffer` (skipping its byte 0, `P5GeneralDriver.cpp:277-282`). **The answer is synchronous**: the control-transfer SETUP stage returns the current `auth_buffer` contents and simultaneously (if the state is idle) sets the state to `recv_f1`, asking core1 to fetch the **next** F1 from the dongle (`P5GeneralDriver.cpp:279-281`).
- core1 sees `recv_f1` with `f1_num > 0`: issues `GET_REPORT(Feature, 0xF1, wLength=64)` to the dongle, state → `recv_f1_wait`, `f1_num--`; if `f1_num == 0`: returns directly to `idle` without fetching (`P5GeneralAuthUSBListener.cpp:50-59`).
- The dongle returns 64 bytes (`[0xF1][63 bytes]`) → `get_report_complete` writes all 64 bytes back into `auth_buffer`, state → `idle` (`P5GeneralAuthUSBListener.cpp:171-178`).
- The net effect is a **one-request lookahead**: the PS5's Nth GET receives the data fetched for request N−1; the 1st GET receives whatever `auth_buffer` held before (all zeros after boot, `src/drivers/p5general/P5GeneralAuth.cpp:12`).
- Once f1_num is exhausted, further GET(F1)s merely repeat the last fetched data (each still briefly passes through `recv_f1` and back to `idle`).
- **[inferred]** (PS4 isomorphism): the F1 payload layout is `[nonce_id][chunk#][0][56-byte signature chunk][CRC32]` (PS4 side: 19 chunks × 56 bytes, `src/drivers/ps4/PS4Driver.cpp:823-842`). P5's f1_num of only 4 or 1 suggests fewer signature blocks or a different fetching strategy; payload semantics belong to the dongle and are not recoverable from the code.

### 5.5 Feature 0xF2: signing state (500 ms auto-fetch)

- After a qualifying challenge is ACKed by the dongle, `auth_recv_f2_us = now + 500 000 µs` is recorded and the state → `recv_f2_delay_500mS` (`P5GeneralAuthUSBListener.cpp:147-149`).
- When core1's poll passes the deadline: `GET_REPORT(Feature, 0xF2, wLength=16)` → state `recv_f2_wait` → on completion `auth_buffer` is overwritten with the 16 bytes (`[0xF2][15 bytes]`) and the state returns to `idle` (`P5GeneralAuthUSBListener.cpp:60-70`, `:179-186`).
- PS5 `GET_REPORT(Feature, 0xF2)` → 16 bytes on the wire = `[0xF2][15 bytes]`, taken from `auth_buffer+1` (`P5GeneralDriver.cpp:283-288`). Note this branch also sets the state to `recv_f1` (the same code as the F1 branch, `P5GeneralDriver.cpp:285-287`); if `f1_num > 0` at that moment it additionally triggers one F1 fetch from the dongle — this is the code's existing behavior.
- **[inferred]** (PS4 isomorphism): the F2 payload is `[nonce_id][status][9 zero bytes][CRC32]`, status 0 = signing ready, 16 = still computing (`src/drivers/ps4/PS4Driver.cpp:844-851`).

### 5.6 Complete State-Machine Table

State definitions: `headers/drivers/p5general/P5GeneralAuth.h:6-15`. Transition sources are cited in the sections above.

```
idle ──(SET_REPORT F0 accepted)──► send_f0 ──(core1 issues host SET F0)──► send_f0_wait
  ^                                                                        |
  |                                                        (dongle ACK, dispatch by type)
  |                                                                        |
  |       ┌── auto-F2 condition not met ◄─────────────────────────────────┤
  |       ▼                                                               ▼
  ├──(PS5 GET F1/F2 while idle)──► recv_f1 ──f1_num>0──► recv_f1_wait   recv_f2_delay_500mS
  |                                  │                     │              │(deadline)
  |                              f1_num==0                 │(host GET F1 done)  ▼
  |                                  │                     ▼            recv_f2
  └──────────────────────────────────┘                     idle   ──(issues host GET F2)──► recv_f2_wait
                                                                            │(host GET F2 done)
                                                                            ▼
                                                                          idle
```

Entry/exit conditions:

| State | Entry condition | Exit condition |
|---|---|---|
| idle | initial / all completion points / recv_f1 with f1_num=0 | F0 received (SET_REPORT) or PS5 reads F1/F2 (armed inside the get_report callback) |
| send_f0 | device side accepted F0 | transitions within one core1 poll cycle |
| send_f0_wait | host SET_REPORT(F0) submitted | dongle ACK (success) |
| recv_f1 | PS5 GET(F1) or GET(F2) while idle | dispatched within one core1 poll cycle |
| recv_f1_wait | host GET_REPORT(F1) submitted | dongle returned data (success) |
| recv_f2_delay_500mS | F0 ACK with auto-F2 condition met | `getMicro() >= auth_recv_f2_us` (500 ms) |
| recv_f2 | delay expired | transitions within one core1 poll cycle |
| recv_f2_wait | host GET_REPORT(F2) submitted | dongle returned data (success) |

---

## 6. Runtime Data Flow: the Input-Report Signing Pipeline

Data path (all **[code]**):

```
core0: button state → P5GeneralReport (64B)
   │  when the state changed (or the repeat quota is not exhausted)
   ▼  hash_pending_buffer = copy of the report; hash_pending = true   (P5GeneralDriver.cpp:229-242)
core1: process() sees hash_pending and the OUT endpoint idle
   │  tuh_hid_send_report(dev, inst, report_id=0, buf, 64)           (P5GeneralAuthUSBListener.cpp:38-41)
   ▼  → dongle interrupt OUT endpoint: 64 raw bytes (no extra ID prefix; first byte is 0x01)
dongle internal processing (black box)
   ▼
host continuously polls the dongle interrupt IN endpoint → 64-byte report
   │  report_received: if !hash_ready, hash_finish_buffer = report    (P5GeneralAuthUSBListener.cpp:115-120)
   ▼  hash_ready = true; if the previous copy has not been sent out yet, this one is dropped
core0: process() sees hash_ready and tud_hid_ready()
   │  tud_hid_report(0, hash_finish_buffer, 64)                       (P5GeneralDriver.cpp:114-121)
   ▼  → PS5 IN endpoint 0x82: the 64 bytes forwarded verbatim (report_id=0 adds no prefix; the leading 0x01 is the on-wire ID)
```

Key rules:

- **No output before the dongle is ready**: with `dongle_ready == false`, `process()` returns immediately (`P5GeneralDriver.cpp:106-108`) and the PS5 receives no input reports at all.
- **Change-driven with 4 repeats**: when the report differs from the previous one, `diff_report_repeat = 4` is set (`P5GeneralDriver.cpp:233`); while the content is unchanged but the quota is > 0 the same report keeps being sent (`:235-239`); once the quota is exhausted and nothing changed, nothing is sent (`:240-242`). The wire is silent at idle.
- **Send throttling**: while `hash_pending == true` (a report awaits signing), core0 produces no new report (`P5GeneralDriver.cpp:123-125`); likewise while `hash_ready == true` and the previous report has not been successfully delivered (endpoint busy) (`:114-121`).
- **Drop rule**: a dongle IN report arriving while the previous `hash_finish_buffer` has not yet been delivered to the PS5 is dropped outright (`P5GeneralAuthUSBListener.cpp:116-119`) — pipeline back-pressure works by dropping, not by queueing.
- **Suspend wakeup**: while the bus is suspended, `tud_remote_wakeup()` is attempted every cycle (`P5GeneralDriver.cpp:110-112`).
- **Dataflow integrity**: the input report sent to the PS5 is **always the bytes returned by the dongle**; the GP2040 rewrites nothing (`P5GeneralDriver.cpp:115` sends `hash_finish_buffer` directly). **[inferred]**: the dongle fills in (at least) the trailing 8-byte `hash` field before returning, and may also rewrite `reportCounter` / `auth_seq_number` and similar fields — the GP2040 never maintains those fields itself, so if the PS5 requires them to advance, only the dongle can do it (see Section 10).
- **PS5 → device output reports (rumble/LED etc.) are discarded**: OUT endpoint data reaches `set_report` via TinyUSB as `HID_REPORT_TYPE_OUTPUT`, and the driver returns immediately for non-Feature types (`lib/tinyusb/src/class/hid/hid_device.c:389-398`; `P5GeneralDriver.cpp:297-299`). **[inferred]** This driver supports neither forwarding rumble/LED to the dongle nor acting on it locally.

---

## 7. Dongle-Side Interfacing Contract (Host Implementer's Guide)

The complete contract for playing "the GP2040 role" toward a real P5 General dongle on any platform **[code]** (implementation in `src/drivers/p5general/P5GeneralAuthUSBListener.cpp`):

1. **Enumerate and identify**: match `VID == 0x2B81 && PID == 0x0101` and take its HID interface (class 0x03). The code does not parse the report descriptor and does not check usage pages (contrast the PS4 listener, which checks 0xFFF0/0xF3, `src/drivers/ps4/PS4AuthUSBListener.cpp:90-105` — the P5 listener has no such check). First match wins (`P5GeneralAuthUSBListener.cpp:84-100`).
2. **Interrupt IN**: continuously submit 64-byte receives; every delivered report is an "input report to forward to the console" (re-queue on mount and after every report, `src/usbhostmanager.cpp:108-114`, `141-148`).
3. **Interrupt OUT**: write the raw 64-byte input report (no separate report-ID prefix; the buffer's first byte 0x01 is the report ID). Confirm the endpoint is idle before sending (`tuh_hid_send_ready` → `!usbh_edpt_busy(ep_out)`, `lib/tinyusb/src/class/hid/hid_host.c:373-377`, `379-404`).
4. **Control transfers (all Feature, recipient = interface)**:
   - `SET_REPORT` F0: `0x21, 0x09, wValue=0x03F0, wIndex=interface, wLength=64`, data = the upstream challenge verbatim, 64 bytes;
   - `GET_REPORT` F1: `0xA1, 0x01, wValue=0x03F1, wIndex=interface, wLength=64`, expect `[0xF1][63B]`;
   - `GET_REPORT` F2: `0xA1, 0x01, wValue=0x03F2, wIndex=interface, wLength=16`, expect `[0xF2][15B]`.
5. **State machine and timing**: drive per the table in 5.6; the only explicit timing parameter is the **500 ms** wait after a successful F0 before fetching F2; the F1 fetch quota 4/1/0 is selected by the challenge type byte.
6. **Unplug handling**: device removal clears `dongle_ready` and the address (`P5GeneralAuthUSBListener.cpp:102-113`); note the **state machine is not reset** (`resetHostData()` is empty, `:30-32`) — see the failure paths in Section 9.

To implement the **device side** (toward the PS5) with raw_gadget on Linux, besides the exact descriptor bytes of Section 2 you need: the response sources and STALL semantics of GET_REPORT(0x03/0xF1/0xF2) (unimplemented report IDs always STALL), the acceptance conditions of SET_REPORT(0xF0) (wLength=64, idle state), 1 ms servicing of the IN endpoint, silent discard of OUT endpoint data, and the remote-wakeup attempt on suspend.

---

## 8. DualSense Host Input Path

`DualsensePS5Host` (`src/hosts/DualsensePS5Host.cpp`, `headers/hosts/DualsensePS5Host.h`) is the PS5 branch of the generic host-port pad addon and is not part of the authentication chain:

- Match: VID 0x054C, PID 0x0CE6 (wired DualSense, `DualsensePS5Host.cpp:3-13`).
- Report parsing: when `report[0] == 0x01`, the 64 bytes are interpreted wholesale through the `P5GenerorReport` struct (`:32-35`) — **[code]** a real DualSense's input report 0x01 coincides with the `P5GenerorReport` layout (the same struct is memcpy-compatible); **[inferred]** the P5 General protocol derives from the DualSense HID dialect.
- Dedup: state updates only when `reportCounter` changes (`:37-70`).
- Mapping: sticks via `map(v, 0..255, 0..65535)` linearly scaled into the 16-bit internal state (`:38-41`, `:91-93`); triggers 8-bit passthrough; buttons and hat mapped bit-wise onto GP2040 internal masks (`:45-69`).
- Output capability declaration: `hasAnalogTriggers/hasLeftAnalogStick/hasRightAnalogStick = true` (`:76-80`).
- This host path never writes output reports to the DualSense (`set_report_complete`/`get_report_complete` are empty, `DualsensePS5Host.h:24-25`).

---

## 9. Timing, Retry and Failure-Path Summary

| Item | Value / behavior | Source |
|---|---|---|
| Auto F2 fetch delay | 500 ms (`getMicro() + 500*1000` µs) | `P5GeneralAuthUSBListener.cpp:148` |
| F1 fetch quota | type 0x01 → 4; 0x03 → 1; 0x02/other → 0 | `:133-142` |
| Device→PS5 IN endpoint bInterval | 1 ms | `P5GeneralDescriptors.h:270` |
| Device→PS5 OUT endpoint bInterval | 6 ms | `:277` |
| Repeat-send quota | 4 identical reports per state change | `P5GeneralDriver.cpp:233-239` |
| Touch counter wrap | 128 | `P5GeneralDescriptors.h:55`; `P5GeneralDriver.cpp:209-226` |
| `P5GENERAL_KEEPALIVE_US` | 5000 µs, **defined but unused** (no reference site) | `P5GeneralDriver.cpp:8` |
| Host-side control retries | **none**. Each GET/SET is submitted once | `P5GeneralAuthUSBListener.cpp:74-82` |
| Consequence of a failed transfer | the host-side completion callback reports len=0 and is filtered out by `usbhostmanager` (`src/usbhostmanager.cpp:151-154`, `158-161`); the state machine stays in `*_wait` with **no timeout recovery**; subsequent F0s are also rejected (requires idle) | same + `P5GeneralDriver.cpp:305` |
| Dongle unplug | `dongle_ready=false`, address cleared to 0xFF; the **state machine is not reset** (`resetHostData()` empty), so a state wedged in `*_wait` stays wedged after replug | `P5GeneralAuthUSBListener.cpp:102-113`, `:30-32` |
| SET_REPORT(F0) with wrong length | wLength ≠ 64 (callback bufsize ≠ 63) silently ignored | `P5GeneralDriver.cpp:302-304` |
| SET_REPORT(F0) while not idle | silently ignored | `:305` |
| GET of an unimplemented feature report | device STALLs (driver returns -1 → TinyUSB assertion fails) | `P5GeneralDriver.cpp:290`; `hid_device.c:305-306` |
| OUT endpoint (PS5 output reports) | received and discarded | `P5GeneralDriver.cpp:297-299` |
| GET(F1)/GET(F2) reading stale data | see 5.4 "one-request lookahead" | `P5GeneralDriver.cpp:277-288` |
| Concurrency | `auth_buffer`/`hash_*` are shared lock-free between core0 (USB device-stack callbacks) and core1 (listener poll); the report pipeline is single-slot (one pending, one finish) | `P5GeneralAuth.h:17-29` |

---

## 10. Items That Could Not Be Determined from Code

The following are not recoverable from the GP2040-CE sources; this document makes no guesses:

1. **The dongle's signing algorithm and keys**: all cryptography happens inside the dongle; the GP2040 only moves bytes. Key material and algorithm (symmetric MAC / asymmetric signature / other) are entirely invisible.
2. **The cryptographic meaning of the `hash[8]` field**: only that the dongle fills it (the GP2040 never writes the field, and every report must round-trip through the dongle). Whether it is a CRC, truncated MAC or sequence-bound signature cannot be determined; the field name is the only clue.
3. **The internal layout of the F0 challenge payload (63 bytes)**: the code consumes only payload byte 1 (the type) and byte 3 (the ==3 test). The semantics of the other 61 bytes, whether a CRC32 is present (the isomorphic PS4 protocol has one at the tail, `src/drivers/ps4/PS4Driver.cpp:893-899`, but the P5 path never validates one), and the nonce length/chunking are unknown.
4. **The internal layout of the F1/F2 answer payloads**: transported as 64/16-byte black boxes. The PS4 layouts `[id][chunk][0][56B][CRC32]` / `[id][state][pad][CRC32]` are analogy only, unconfirmed on the P5 side.
5. **The PS5-side polling counts and decision logic**: how many times the console reads F1, when it deems authentication to succeed or fail, and the protocol meaning of the type byte (which phase 0x01/0x02/0x03 maps to) — console-firmware behavior.
6. **The dongle's own USB descriptors**: the code identifies the dongle solely by VID/PID and never reads its device/configuration/report descriptors (unlike the PS4 listener, which parses usage pages). Endpoint numbers, bInterval values, interface count and strings must be obtained by live enumeration.
7. **The dongle's IN-report cadence and content relationship**: its spontaneous report rate, whether each OUT report produces exactly one IN report, and which fields besides `hash` it rewrites (e.g. `reportCounter`, `auth_seq_number`) are unknown.
8. **The protocol semantics of report 0xE0 (2-byte feature) and report 0x02 (47-byte output)**: the descriptor declares them but the driver does not implement them (0xE0 reads STALL; 0x02 receipts are discarded). Whether the PS5 uses them, and their formats, are unknown.
9. **Minimum requirements the dongle places on report content fields**: the GP2040 never increments `reportCounter`/`auth_seq_number`; if the protocol requires monotonicity, only the dongle could enforce it on the returned copy — whether it does is unknown.
10. **Byte-level identity between the cloned identity and the real dongle's descriptors**: the VID/PID match has code evidence (0x2B81/0x0101 appears in both the device descriptor and the host-side match); whether bcdDevice, strings and the endpoint table are identical to a real dongle is not verified inside this repository.
11. **The `CRC32.h` include**: `P5GeneralAuthUSBListener.cpp:5` includes the CRC32 header but the file contains no call — whether the P5 protocol uses CRC32 anywhere is undeterminable.
12. **Remote-wakeup effectiveness**: the descriptor's bmAttributes=0x80 does not declare remote wakeup while the code calls `tud_remote_wakeup()`; whether the PS5 honors it is unverified.

---

## 11. Configuration Semantics and User-Visible Behavior

- **Mode enum**: `INPUT_MODE_P5GENERAL = 16` (`proto/enums.proto:159`); `DriverManager::setup` instantiates `P5GeneralDriver` in that mode (`src/drivermanager.cpp:62-64`).
- **Enablement**: the authentication subsystem requires peripheral block USB0 to be enabled (`P5GeneralAuth::available()` → `PeripheralManager::isUSBEnabled(0)`, `src/drivers/p5general/P5GeneralAuth.cpp:18-20`); the PIO host stack starts only when "USB0 enabled and at least one listener exists" (`src/usbhostmanager.cpp:16-24`). The web config marks the mode as "optional USB, authentication USB", with the note "Requires a USB host connection and P5General to properly authenticate in PS5 general mode" (`www/src/Data/InputBootModes.ts:193-200`; `www/src/Locales/en/SettingsPage.jsx:85-86`).
- **Listener registration chain**: `GP2040Aux::setup()` calls `initializeAux()` (which creates the auth driver and listener) and pushes the listener returned by `get_usb_auth_listener()` into `USBHostManager` (`src/gp2040aux.cpp:29-37`); the core1 loop keeps calling `processAux()` to drive the state machine (`:53-64`).
- **UI/LED**: the status bar shows "P5G"; the auth-sent flag `getAuthSent()` is constantly false (":AS" never displays, `headers/drivers/p5general/P5GeneralDriver.h:37`; `src/display/ui/screens/ButtonLayoutScreen.cpp:288-293`); the player-LED animation reuses the PS4 style (`src/addons/neopicoleds.cpp:305-307`); the main-menu name is "P5 General" (`headers/display/ui/screens/MainMenuScreen.h:25`).
- **Config-save gating**: `Storage::save`'s "no save while PS4/PS5 + USB auth" rule has no P5GENERAL branch (`src/storagemanager.cpp:44-54`); `getDongleAuthRequired()` is constantly true (`P5GeneralDriver.cpp:100-103`) but no caller currently reaches it in P5 mode.
- **Build**: the three P5 sources are part of the firmware build (`CMakeLists.txt:219-221`).

---

## 12. Implementation Checklist

Verify each item when implementing a driver that interoperates with the P5 General (the GP2040 role):

- [ ] Device side (toward the PS5): reproduce the device/configuration/HID/report descriptors and string table of Section 2 byte-exactly.
- [ ] Device side: GET_REPORT(0x03) returns the 47-byte static data of 5.2; unimplemented feature IDs always STALL.
- [ ] Device side: SET_REPORT(0xF0) is accepted only at wLength=64 and only in the idle state.
- [ ] Device side: GET(F1)/GET(F2) return the buffer's current contents while arming the next fetch (one-request lookahead semantics).
- [ ] Device side: the IN endpoint only ever transmits bytes returned by the dongle; OUT endpoint data is discarded; remote wakeup is attempted on suspend.
- [ ] Host side (toward the dongle): identify by VID/PID 0x2B81/0x0101; poll interrupt IN continuously; write raw 64 bytes to interrupt OUT.
- [ ] Host side: the three feature control transfers follow the encodings of 5.1; F0 is relayed with no validation or modification.
- [ ] State machine: follow the transition table of 5.6; 500 ms F2 delay; f1_num = 4/1/0.
- [ ] Report pipeline: change detection + 4 repeats; single pending/finish slot; dongle reports arriving while hash_ready is uncleared are dropped.
- [ ] Failure paths: decide explicitly whether to add a timeout to `*_wait` (the original implementation has none, see Section 9).

---

## 13. Source Evidence Inventory

P5-specific (read line by line):

- `src/drivers/p5general/P5GeneralDriver.cpp` / `headers/drivers/p5general/P5GeneralDriver.h`
- `src/drivers/p5general/P5GeneralAuth.cpp` / `headers/drivers/p5general/P5GeneralAuth.h`
- `src/drivers/p5general/P5GeneralAuthUSBListener.cpp` / `headers/drivers/p5general/P5GeneralAuthUSBListener.h`
- `headers/drivers/p5general/P5GeneralDescriptors.h` (all 278 descriptor lines decoded byte by byte)
- `src/hosts/DualsensePS5Host.cpp` / `headers/hosts/DualsensePS5Host.h`
- `src/drivermanager.cpp`

Supporting framework:

- `headers/gpdriver.h`, `headers/usblistener.h`, `headers/drivers/shared/gpauthdriver.h`, `headers/gphost.h`
- `src/usbdriver.cpp` (tud_* callback → driver routing), `src/usbhostmanager.cpp` (tuh_* callback → listener dispatch)
- `src/gp2040.cpp` (core0 main loop), `src/gp2040aux.cpp` (core1 auxiliary loop and listener registration)
- `src/addons/gamepad_usb_host_listener.cpp` (DualSense host-path attachment), `headers/addons/gamepad_usb_host_listener.h` (3 ms poll interval)
- `headers/drivers/ps4/PS4Descriptors.h` (PSSensor/TouchpadData structs), `src/drivers/ps4/PS4Driver.cpp`, `src/drivers/ps4/PS4AuthUSBListener.cpp` (protocol analogy)
- `lib/tinyusb/src/class/hid/hid_device.c`, `hid_device.h`, `hid_host.c`, `hid_host.h`, `hid.h` (control-transfer encodings and callback semantics)
- `headers/tusb_config.h`, `headers/drivers/shared/driverhelper.h`, `src/gamepad.cpp` (getMicro/getMillis)
- `src/peripheralmanager.cpp`, `lib/PicoPeripherals/peripheral_usb.h` (PIO USB enablement and default config)
- `src/storagemanager.cpp`, `proto/enums.proto`, `CMakeLists.txt`, `www/src/Data/InputBootModes.ts`, `www/src/Locales/en/SettingsPage.jsx`, `www/src/Locales/zh-CN/SettingsPage.jsx`, `src/addons/neopicoleds.cpp`, `src/display/ui/screens/ButtonLayoutScreen.cpp`, `headers/display/ui/screens/MainMenuScreen.h`

---

## 14. Hardware Measurement Addendum

The following facts come from measurements in this repository on a Jetson Orin with a real P5 General dongle and a PS5; they answer part of section 10 directly. Reproduction path: `sudo python3 scripts/test/p5g_dongle_probe.py` (dongle side) plus launching the backend with `-M p5g` while watching the console (PS5 side).

| Item | Measured result | Section 10 entry |
|---|---|---|
| The dongle's own descriptors | **Byte-for-byte identical to the clone described in sections 2 and 3**: the report descriptor is exactly 165 bytes (every byte the same), the configuration is 41 bytes with wTotalLength=0x29, bMaxPower=0xFA, bcdHID=1.11, and endpoints 0x82 IN 64 B bInterval=1 with 0x01 OUT 64 B | §10.6 |
| Kernel binding | Binds under `hid-generic` → a `hidraw` node plus an evdev joystick node (a host implementation must exclude that evdev node from the human-input channel by VID/PID) | §10.6 |
| Feature 0x03 reply | 48 bytes; the first 47 bytes are **byte-for-byte identical** to the static table in section 5.2, the 48th is zero | §10.8 |
| Features 0xF1 / 0xF2 (pre-auth) | The transfers work; payloads are all zero (no auth in progress, matching "auth_buffer all zero after boot") | §10.4 |
| OUT → IN relation | **Strictly 1:1**: every 64-byte write to the interrupt OUT endpoint draws exactly one 64-byte IN report; the dongle **never streams spontaneously** (zero reports over a 3 s idle window), so wire silence while idle is guaranteed from both sides of the protocol | §10.7 |
| Round-trip latency | mean 2.8 ms, fastest 1.5 ms (8/8 answered) ⇒ ≈356 Hz report-rate ceiling under continuous input | §10.7 |
| Fields the dongle fills in | The trailing 8-byte `hash` is filled; **`auth_seq_number` (bytes 12–15) is rewritten by the dongle as well** (different on every round trip) — with byte 7 `reportCounter` held at zero as in section 4, monotonicity is the dongle's job on the return path, which is the direction §10.9 inferred | §10.2, §10.9 |
| PS5-side flow | Enumeration succeeds → it reads `GET_REPORT(0x03)` → **the auth handshake starts only on the first physical-pad input (a PS press)**: four type-0x01 challenges (f1 quota 4), one type-0x02, an F2 signing-state fetch → the pad is assigned and its input reaches the console | §10.5 |
| One-line conclusion | The dongle is a purely reactive signing black box: content is submitted in the report shape of section 4, and the report it returns is what must go on the wire | §10.1–§10.3 |
