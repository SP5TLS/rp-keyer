//! CDC-ACM class with SERIAL_STATE notification support.
//!
//! Embassy-usb 0.4.0's built-in `CdcAcmClass` creates the interrupt IN endpoint
//! required by the CDC spec but then discards it (`_comm_ep`), making it impossible
//! to send SERIAL_STATE notifications (device-to-host DCD/DSR signalling).
//!
//! This class is a minimal replacement that keeps the interrupt endpoint and exposes
//! `send_serial_state(dcd, dsr)` for signalling paddle state via modem control lines.

use core::mem::MaybeUninit;
use core::sync::atomic::AtomicU8;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_usb::control::{self, InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

const USB_CLASS_CDC: u8 = 0x02;
const USB_CLASS_CDC_DATA: u8 = 0x0a;
const CDC_SUBCLASS_ACM: u8 = 0x02;
const CDC_PROTOCOL_NONE: u8 = 0x00;

const CS_INTERFACE: u8 = 0x24;
const CDC_TYPE_HEADER: u8 = 0x00;
const CDC_TYPE_ACM: u8 = 0x02;
const CDC_TYPE_UNION: u8 = 0x06;

const REQ_SEND_ENCAPSULATED_COMMAND: u8 = 0x00;
const REQ_SET_LINE_CODING: u8 = 0x20;
const REQ_GET_LINE_CODING: u8 = 0x21;
const REQ_SET_CONTROL_LINE_STATE: u8 = 0x22;

/// CDC SERIAL_STATE notification code (CDC spec table 69).
const NOTIF_SERIAL_STATE: u8 = 0x20;

/// Bit 0 of SERIAL_STATE bitmask: bRxCarrier (DCD).
const SERIAL_STATE_DCD: u16 = 1 << 0;
/// Bit 1 of SERIAL_STATE bitmask: bTxCarrier (DSR).
const SERIAL_STATE_DSR: u16 = 1 << 1;

// ── Internal shared state (same as embassy-usb's CdcAcmClass) ────────────────

pub struct State<'a> {
    control: MaybeUninit<Control<'a>>,
    shared: ControlShared,
}

impl<'a> State<'a> {
    pub fn new() -> Self {
        Self {
            control: MaybeUninit::uninit(),
            shared: ControlShared::default(),
        }
    }
}

impl Default for State<'_> {
    fn default() -> Self {
        Self::new()
    }
}

struct ControlShared {
    dtr: AtomicBool,
    rts: AtomicBool,
    /// Interface number embedded in the wIndex field of every SERIAL_STATE
    /// notification. Written once in `CdcWithSerialState::new()` with `Release`
    /// and loaded with `Acquire` in `send_serial_state` so the read can never
    /// observe the uninitialized 0 even if the constructor's happens-before is
    /// ever provided by something other than task-spawn synchronization.
    comm_if: AtomicU8,
}

impl Default for ControlShared {
    fn default() -> Self {
        Self {
            dtr: AtomicBool::new(false),
            rts: AtomicBool::new(false),
            comm_if: AtomicU8::new(0),
        }
    }
}

struct Control<'a> {
    comm_if: InterfaceNumber,
    shared: &'a ControlShared,
}

impl<'a> Control<'a> {
    fn shared(&mut self) -> &'a ControlShared {
        self.shared
    }
}

impl Handler for Control<'_> {
    fn reset(&mut self) {
        let shared = self.shared();
        shared.dtr.store(false, Ordering::Relaxed);
        shared.rts.store(false, Ordering::Relaxed);
    }

    fn control_out(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient, req.index)
            != (
                RequestType::Class,
                Recipient::Interface,
                self.comm_if.0 as u16,
            )
        {
            return None;
        }
        match req.request {
            REQ_SEND_ENCAPSULATED_COMMAND => Some(OutResponse::Accepted),
            REQ_SET_LINE_CODING if data.len() >= 7 => Some(OutResponse::Accepted),
            REQ_SET_CONTROL_LINE_STATE => {
                let dtr = (req.value & 0x0001) != 0;
                let rts = (req.value & 0x0002) != 0;
                let shared = self.shared();
                shared.dtr.store(dtr, Ordering::Relaxed);
                shared.rts.store(rts, Ordering::Relaxed);
                Some(OutResponse::Accepted)
            }
            _ => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if (req.request_type, req.recipient, req.index)
            != (
                RequestType::Class,
                Recipient::Interface,
                self.comm_if.0 as u16,
            )
        {
            return None;
        }
        match req.request {
            REQ_GET_LINE_CODING if req.length == 7 => {
                // Return a default 8N1 / 9600 baud line coding.
                assert!(buf.len() >= 7);
                buf[0..4].copy_from_slice(&9600u32.to_le_bytes()); // data rate
                buf[4] = 0; // 1 stop bit
                buf[5] = 0; // no parity
                buf[6] = 8; // 8 data bits
                Some(InResponse::Accepted(&buf[0..7]))
            }
            _ => Some(InResponse::Rejected),
        }
    }
}

// ── Public class ─────────────────────────────────────────────────────────────

/// CDC-ACM class with SERIAL_STATE notification support.
///
/// Drop-in replacement for `embassy_usb::class::cdc_acm::CdcAcmClass` that
/// keeps the interrupt IN endpoint and can drive DCD/DSR modem status lines
/// on the host side via [`send_serial_state`][Self::send_serial_state].
pub struct CdcWithSerialState<'d, D: Driver<'d>> {
    comm_ep: D::EndpointIn,
    read_ep: D::EndpointOut,
    /// Bulk IN endpoint — declared in the USB descriptor per CDC-ACM spec but
    /// not used by this firmware (we only signal state, never send data).
    _write_ep: D::EndpointIn,
    control: &'d ControlShared,
}

impl<'d, D: Driver<'d>> CdcWithSerialState<'d, D> {
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State<'d>,
        max_packet_size: u16,
    ) -> Self {
        assert!(builder.control_buf_len() >= 7);

        let mut func = builder.function(USB_CLASS_CDC, CDC_SUBCLASS_ACM, CDC_PROTOCOL_NONE);

        // Control interface
        let mut iface = func.interface();
        let comm_if = iface.interface_number();
        let data_if = u8::from(comm_if) + 1;
        let mut alt = iface.alt_setting(USB_CLASS_CDC, CDC_SUBCLASS_ACM, CDC_PROTOCOL_NONE, None);

        alt.descriptor(CS_INTERFACE, &[CDC_TYPE_HEADER, 0x10, 0x01]); // bcdCDC 1.10
        alt.descriptor(
            CS_INTERFACE,
            &[
                CDC_TYPE_ACM,
                0x02, // bmCapabilities: supports Set/Get_Line_Coding,
                      // Set_Control_Line_State, Serial_State notification
            ],
        );
        alt.descriptor(CS_INTERFACE, &[CDC_TYPE_UNION, comm_if.into(), data_if]);

        // 1 ms poll interval (not the CDC-ACM standard 10 ms) — we use
        // SERIAL_STATE to carry per-element CW key state, so a 10 ms
        // host poll adds up to 10 ms of wall-clock delay per transition,
        // which is ~25% of a dit at 28 WPM.  1 ms matches the MIDI
        // interrupt-IN cadence and is the minimum at full-speed USB.
        let comm_ep = alt.endpoint_interrupt_in(None, 10, 1);

        // Data interface
        let mut iface = func.interface();
        let mut alt = iface.alt_setting(USB_CLASS_CDC_DATA, 0x00, CDC_PROTOCOL_NONE, None);
        let read_ep = alt.endpoint_bulk_out(None, max_packet_size);
        let write_ep = alt.endpoint_bulk_in(None, max_packet_size);

        drop(func);

        state
            .shared
            .comm_if
            .store(comm_if.into(), core::sync::atomic::Ordering::Release);

        let control = state.control.write(Control {
            shared: &state.shared,
            comm_if,
        });
        builder.handler(control);

        Self {
            comm_ep,
            read_ep,
            _write_ep: write_ep,
            control: &state.shared,
        }
    }

    /// Returns the DTR (data terminal ready) signal set by the host.
    pub fn dtr(&self) -> bool {
        self.control.dtr.load(Ordering::Relaxed)
    }

    /// Returns the RTS (request to send) signal set by the host.
    pub fn rts(&self) -> bool {
        self.control.rts.load(Ordering::Relaxed)
    }

    /// Waits for the USB host to enable this interface.
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }

    /// Send a CDC SERIAL_STATE notification to the host.
    ///
    /// - `dcd` — Data Carrier Detect (dit paddle)
    /// - `dsr` — Data Set Ready (dah paddle)
    ///
    /// The host exposes these as modem status bits on the virtual COM port.
    /// On Linux: readable via `TIOCMGET ioctl` as `TIOCM_CAR` (DCD) and `TIOCM_DSR`.
    /// On Windows: readable via `GetCommModemStatus` as `MS_RLSD_ON` and `MS_DSR_ON`.
    pub async fn send_serial_state(&mut self, dcd: bool, dsr: bool) -> Result<(), EndpointError> {
        let mut state: u16 = 0;
        if dcd {
            state |= SERIAL_STATE_DCD;
        }
        if dsr {
            state |= SERIAL_STATE_DSR;
        }

        // CDC spec §6.3.5 — SERIAL_STATE notification, 10-byte packet:
        //   [0]   bmRequestType = 0xA1 (IN | CLASS | INTERFACE)
        //   [1]   bNotificationCode = 0x20
        //   [2-3] wValue = 0
        //   [4-5] wIndex = interface number
        //   [6-7] wLength = 2
        //   [8-9] data = 16-bit state bitmask (LE)
        let iface = self
            .control
            .comm_if
            .load(core::sync::atomic::Ordering::Acquire);
        let packet: [u8; 10] = [
            0xA1,
            NOTIF_SERIAL_STATE,
            0x00,
            0x00, // wValue
            iface,
            0x00, // wIndex (LE)
            0x02,
            0x00, // wLength (LE)
            (state & 0xFF) as u8,
            (state >> 8) as u8,
        ];

        self.comm_ep.write(&packet).await
    }
}
