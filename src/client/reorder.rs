// Copyright (C) The Retina Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Reordering of UDP RTP packets.
//!
//! Without it, a UDP packet that arrives after a later one is dropped (see
//! [`super::rtp::InorderParser`]), which breaks the frame it belongs to.
//! [`Reorder`] holds packets that arrive ahead of a gap for up to a configured
//! window, and releases them in sequence order once the gap is filled. If the
//! window passes first, the missing packets are given up on: the held ones are
//! released anyway, and the parser reports the gap as loss.

use std::time::{Duration, Instant};

/// Packets held at most at once, however long the window. A gap early in a
/// large keyframe can hold several hundred packets; past this many, the gap is
/// given up on early.
const MAX_HELD: usize = 1024;

/// A packet that arrived ahead of a gap.
#[derive(Debug)]
pub(super) struct Held {
    pub seq: u16,
    pub data: Box<[u8]>,
    pub received: Instant,
    pub received_wall: crate::WallTime,
}

/// What to do with a packet that just arrived.
#[derive(Debug, PartialEq, Eq)]
pub(super) enum Arrival {
    /// It's the next one in sequence (or reordering is off): pass it on now.
    Deliver,
    /// It's ahead of a gap: hold it with [`Reorder::hold`].
    Hold,
    /// It's older than what was already passed on, or a duplicate of a held
    /// one: drop it.
    Drop,
}

#[derive(Debug)]
pub(super) struct Reorder {
    window: Duration,
    /// The sequence number expected next, once one has been passed on.
    next: Option<u16>,
    /// Packets ahead of `next`, in arrival order.
    held: Vec<Held>,
}

impl Reorder {
    /// A zero `window` turns reordering off: every packet is delivered as it
    /// arrives.
    pub fn new(window: Duration) -> Self {
        Self {
            window,
            next: None,
            held: Vec::new(),
        }
    }

    /// Decides about a packet that just arrived with sequence number `seq`.
    pub fn arrive(&mut self, seq: u16) -> Arrival {
        if self.window.is_zero() {
            return Arrival::Deliver;
        }
        let Some(next) = self.next else {
            self.next = Some(seq.wrapping_add(1));
            return Arrival::Deliver;
        };
        match seq.wrapping_sub(next) {
            0 => {
                self.next = Some(seq.wrapping_add(1));
                Arrival::Deliver
            }
            ahead if ahead < 0x8000 => {
                if self.held.iter().any(|h| h.seq == seq) {
                    Arrival::Drop
                } else {
                    Arrival::Hold
                }
            }
            // Behind `next`: it was passed on already, or given up on.
            _ => Arrival::Drop,
        }
    }

    /// Holds a packet [`arrive`](Self::arrive) said to.
    pub fn hold(&mut self, packet: Held) {
        self.held.push(packet);
    }

    /// The next held packet to pass on: the one `next` is waiting for, or, if
    /// the oldest held packet has waited out the window (or too many are
    /// held), the earliest one after the gap.
    pub fn pop_ready(&mut self, now: Instant) -> Option<Held> {
        let next = self.next?;
        let i = match self.held.iter().position(|h| h.seq == next) {
            Some(i) => i,
            None => {
                let expired = self
                    .deadline()
                    .is_some_and(|d| now >= d || self.held.len() > MAX_HELD);
                if !expired {
                    return None;
                }
                // Give up on the gap: resume from the earliest held packet.
                self.held
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, h)| h.seq.wrapping_sub(next))
                    .map(|(i, _)| i)?
            }
        };
        let h = self.held.swap_remove(i);
        self.next = Some(h.seq.wrapping_add(1));
        Some(h)
    }

    /// When the oldest held packet's window ends.
    pub fn deadline(&self) -> Option<Instant> {
        self.held
            .iter()
            .map(|h| h.received)
            .min()
            .map(|r| r + self.window)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WINDOW: Duration = Duration::from_millis(20);

    fn held(seq: u16, received: Instant) -> Held {
        Held {
            seq,
            data: Box::new([]),
            received,
            received_wall: crate::WallTime::now(),
        }
    }

    /// Feeds `seqs` at `t0`, holding and releasing as the session would, and
    /// returns the order they come out in (ready packets are drained after
    /// each arrival).
    fn run(r: &mut Reorder, t0: Instant, seqs: &[u16]) -> Vec<u16> {
        let mut out = Vec::new();
        for &seq in seqs {
            match r.arrive(seq) {
                Arrival::Deliver => out.push(seq),
                Arrival::Hold => r.hold(held(seq, t0)),
                Arrival::Drop => {}
            }
            while let Some(h) = r.pop_ready(t0) {
                out.push(h.seq);
            }
        }
        out
    }

    #[test]
    fn in_order_passes_straight_through() {
        let mut r = Reorder::new(WINDOW);
        let t0 = Instant::now();
        assert_eq!(run(&mut r, t0, &[10, 11, 12, 13]), vec![10, 11, 12, 13]);
        assert_eq!(r.deadline(), None);
    }

    #[test]
    fn swapped_packets_come_out_in_order() {
        let mut r = Reorder::new(WINDOW);
        let t0 = Instant::now();
        assert_eq!(
            run(&mut r, t0, &[1, 3, 2, 4, 7, 6, 5, 8]),
            vec![1, 2, 3, 4, 5, 6, 7, 8]
        );
        assert_eq!(r.deadline(), None);
    }

    #[test]
    fn a_lost_packet_is_given_up_on_after_the_window() {
        let mut r = Reorder::new(WINDOW);
        let t0 = Instant::now();
        // 3 never arrives.
        assert_eq!(run(&mut r, t0, &[1, 2, 4, 5]), vec![1, 2]);
        assert_eq!(r.deadline(), Some(t0 + WINDOW));
        assert!(
            r.pop_ready(t0 + WINDOW - Duration::from_millis(1))
                .is_none()
        );
        let after: Vec<u16> = std::iter::from_fn(|| r.pop_ready(t0 + WINDOW))
            .map(|h| h.seq)
            .collect();
        assert_eq!(after, vec![4, 5]);
        // 3 turning up now is too late.
        assert_eq!(r.arrive(3), Arrival::Drop);
        assert_eq!(r.arrive(6), Arrival::Deliver);
    }

    #[test]
    fn late_and_duplicate_packets_are_dropped() {
        let mut r = Reorder::new(WINDOW);
        let t0 = Instant::now();
        assert_eq!(run(&mut r, t0, &[1, 2, 2, 1, 4, 4, 3]), vec![1, 2, 3, 4]);
    }

    #[test]
    fn sequence_numbers_wrap() {
        let mut r = Reorder::new(WINDOW);
        let t0 = Instant::now();
        assert_eq!(
            run(&mut r, t0, &[65534, 0, 65535, 1]),
            vec![65534, 65535, 0, 1]
        );
    }

    #[test]
    fn too_many_held_gives_up_early() {
        let mut r = Reorder::new(Duration::from_secs(10));
        let t0 = Instant::now();
        let seqs: Vec<u16> = std::iter::once(0)
            .chain(2..=(MAX_HELD as u16 + 2))
            .collect();
        let out = run(&mut r, t0, &seqs);
        // Everything after the gap comes out, in order, without waiting 10 s.
        assert_eq!(out, seqs);
    }

    #[test]
    fn zero_window_delivers_everything() {
        let mut r = Reorder::new(Duration::ZERO);
        let t0 = Instant::now();
        assert_eq!(run(&mut r, t0, &[2, 1, 1, 3]), vec![2, 1, 1, 3]);
    }
}
