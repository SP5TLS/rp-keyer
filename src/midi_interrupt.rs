//! MIDI class using **interrupt** endpoints instead of bulk.
//!
//! USB MIDI 1.0 specifies bulk endpoints, but interrupt endpoints guarantee a
//! maximum host polling interval (`poll_ms`).  Many low-latency MIDI devices
//! use interrupt endpoints in practice; the 1 ms poll keeps keying latency
//! bounded.

use embassy_usb::Builder;
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};

/// Mirrors `embassy_usb::class::midi` constants.
const USB_AUDIO_CLASS: u8 = 0x01;
const USB_AUDIOCONTROL_SUBCLASS: u8 = 0x01;
const USB_MIDISTREAMING_SUBCLASS: u8 = 0x03;
const MIDI_IN_JACK_SUBTYPE: u8 = 0x02;
const MIDI_OUT_JACK_SUBTYPE: u8 = 0x03;
const EMBEDDED: u8 = 0x01;
const EXTERNAL: u8 = 0x02;
const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const HEADER_SUBTYPE: u8 = 0x01;
const MS_HEADER_SUBTYPE: u8 = 0x01;
const MS_GENERAL: u8 = 0x01;
const PROTOCOL_NONE: u8 = 0x00;
const MIDI_IN_SIZE: u8 = 0x06;
const MIDI_OUT_SIZE: u8 = 0x09;

/// Packet-level USB MIDI device using interrupt endpoints.
///
/// Drop-in replacement for `embassy_usb::class::midi::MidiClass` with the
/// single difference that endpoints are interrupt (with configurable `poll_ms`)
/// rather than bulk.  Both IN and OUT endpoints are only allocated when their
/// corresponding jack count is > 0; a device with `n_in_jacks == 0` cannot
/// `write_packet` (the call returns `EndpointError::Disabled`).
pub struct MidiInterruptClass<'d, D: Driver<'d>> {
    read_ep: Option<D::EndpointOut>,
    write_ep: Option<D::EndpointIn>,
}

impl<'d, D: Driver<'d>> MidiInterruptClass<'d, D> {
    /// Creates a new MIDI class with interrupt endpoints.
    ///
    /// `poll_ms` sets the host polling interval for both IN and OUT endpoints.
    /// Use `1` for the lowest possible latency on full-speed USB.
    ///
    /// The OUT endpoint (host → device) is only allocated when `n_out_jacks > 0`,
    /// avoiding a wasted hardware endpoint slot for send-only devices.
    pub fn new(
        builder: &mut Builder<'d, D>,
        n_in_jacks: u8,
        n_out_jacks: u8,
        max_packet_size: u16,
        poll_ms: u8,
    ) -> Self {
        let mut func = builder.function(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE);

        // Audio control interface
        let mut iface = func.interface();
        let audio_if = iface.interface_number();
        let midi_if = u8::from(audio_if) + 1;
        let n_collections: u8 = 1;
        let ac_total_len: u16 = 8 + n_collections as u16;
        let mut alt = iface.alt_setting(
            USB_AUDIO_CLASS,
            USB_AUDIOCONTROL_SUBCLASS,
            PROTOCOL_NONE,
            None,
        );
        alt.descriptor(
            CS_INTERFACE,
            &[
                HEADER_SUBTYPE,
                0x00,
                0x01,
                (ac_total_len & 0xFF) as u8,
                ((ac_total_len >> 8) & 0xFF) as u8,
                n_collections,
                midi_if,
            ],
        );

        // MIDIStreaming interface
        let mut iface = func.interface();
        let _midi_if = iface.interface_number();
        let mut alt = iface.alt_setting(
            USB_AUDIO_CLASS,
            USB_MIDISTREAMING_SUBCLASS,
            PROTOCOL_NONE,
            None,
        );

        // wTotalLength covers the MS_HEADER plus all jack and endpoint descriptors
        // (including standard endpoint descriptors, per USB MIDI 1.0 §6.2.2).
        // Each logical jack (in or out) generates one IN_JACK + one OUT_JACK descriptor.
        let has_out_ep = n_out_jacks > 0;
        let has_in_ep = n_in_jacks > 0;
        let midi_streaming_total_length = 7 // MS_HEADER
            + (n_in_jacks as usize + n_out_jacks as usize)
                * (MIDI_IN_SIZE as usize + MIDI_OUT_SIZE as usize)
            + if has_out_ep { 7 + (4 + n_out_jacks as usize) } else { 0 }
            + if has_in_ep { 7 + (4 + n_in_jacks as usize) } else { 0 };

        alt.descriptor(
            CS_INTERFACE,
            &[
                MS_HEADER_SUBTYPE,
                0x00,
                0x01,
                (midi_streaming_total_length & 0xFF) as u8,
                ((midi_streaming_total_length >> 8) & 0xFF) as u8,
            ],
        );

        let in_jack_id_ext = |index| 2 * index + 1;
        let out_jack_id_emb = |index| 2 * index + 2;
        let out_jack_id_ext = |index| 2 * n_in_jacks + 2 * index + 1;
        let in_jack_id_emb = |index| 2 * n_in_jacks + 2 * index + 2;

        for i in 0..n_in_jacks {
            alt.descriptor(
                CS_INTERFACE,
                &[MIDI_IN_JACK_SUBTYPE, EXTERNAL, in_jack_id_ext(i), 0x00],
            );
        }

        for i in 0..n_out_jacks {
            alt.descriptor(
                CS_INTERFACE,
                &[MIDI_IN_JACK_SUBTYPE, EMBEDDED, in_jack_id_emb(i), 0x00],
            );
        }

        for i in 0..n_out_jacks {
            alt.descriptor(
                CS_INTERFACE,
                &[
                    MIDI_OUT_JACK_SUBTYPE,
                    EXTERNAL,
                    out_jack_id_ext(i),
                    0x01,
                    in_jack_id_emb(i),
                    0x01,
                    0x00,
                ],
            );
        }

        for i in 0..n_in_jacks {
            alt.descriptor(
                CS_INTERFACE,
                &[
                    MIDI_OUT_JACK_SUBTYPE,
                    EMBEDDED,
                    out_jack_id_emb(i),
                    0x01,
                    in_jack_id_ext(i),
                    0x01,
                    0x00,
                ],
            );
        }

        let mut endpoint_data = [
            MS_GENERAL, 0, // Number of jacks
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Jack mappings
        ];

        // OUT endpoint (host → device) — only when there are OUT jacks
        let read_ep = if has_out_ep {
            endpoint_data[1] = n_out_jacks;
            for i in 0..n_out_jacks {
                endpoint_data[2 + i as usize] = in_jack_id_emb(i);
            }
            let ep = alt.endpoint_interrupt_out(None, max_packet_size, poll_ms);
            alt.descriptor(CS_ENDPOINT, &endpoint_data[0..2 + n_out_jacks as usize]);
            Some(ep)
        } else {
            None
        };

        // IN endpoint (device → host) — interrupt instead of bulk.
        // Skipped entirely when `n_in_jacks == 0`; the previous behaviour
        // of allocating a "minimal endpoint" was wasting a hardware
        // endpoint slot on send-less configurations.
        let write_ep = if has_in_ep {
            endpoint_data[1] = n_in_jacks;
            for i in 0..n_in_jacks {
                endpoint_data[2 + i as usize] = out_jack_id_emb(i);
            }
            let ep = alt.endpoint_interrupt_in(None, max_packet_size, poll_ms);
            alt.descriptor(CS_ENDPOINT, &endpoint_data[0..2 + n_in_jacks as usize]);
            Some(ep)
        } else {
            None
        };

        MidiInterruptClass { read_ep, write_ep }
    }

    /// Writes a single packet into the IN endpoint.
    ///
    /// Returns `EndpointError::Disabled` if no IN endpoint was allocated
    /// (i.e. `n_in_jacks` was 0).
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        match &mut self.write_ep {
            Some(ep) => ep.write(data).await,
            None => Err(EndpointError::Disabled),
        }
    }

    /// Reads a single packet from the OUT endpoint.
    ///
    /// Returns `EndpointError::Disabled` if no OUT endpoint was allocated
    /// (i.e. `n_out_jacks` was 0).
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        match &mut self.read_ep {
            Some(ep) => ep.read(data).await,
            None => Err(EndpointError::Disabled),
        }
    }

    /// Waits for the USB host to enable this interface.
    ///
    /// Returns immediately if neither endpoint was allocated.
    pub async fn wait_connection(&mut self) {
        if let Some(ep) = &mut self.write_ep {
            ep.wait_enabled().await;
        } else if let Some(ep) = &mut self.read_ep {
            ep.wait_enabled().await;
        }
    }
}
