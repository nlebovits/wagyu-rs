//! Debug logging infrastructure for wagyu algorithm tracing.
//!
//! This module provides efficient debug logging that can be enabled via
//! the `WAGYU_DEBUG` environment variable. When enabled, it outputs
//! structured log messages that match the C++ oracle's debug output,
//! allowing divergence points to be identified by diffing logs.
//!
//! # Log Format
//!
//! All log messages follow this format for easy parsing and diffing:
//!
//! ```text
//! [SCANBEAM] y=100
//! [AEL_ADD] idx=5 bot=(0,100) top=(10,0) type=Subject
//! [AEL_REMOVE] idx=3
//! [INTERSECT] b1=5 b2=7 pt=(5,50)
//! [RING_NEW] id=1 pt=(5,50)
//! [RING_POINT] id=1 pt=(10,60) front=true
//! [RING_MERGE] from=2 to=3
//! [WINDING] idx=5 wc=1 wc2=0 delta=1
//! [CONTRIBUTING] idx=5 result=true
//! [HORIZONTAL] idx=5 left=0 right=100
//! ```
//!
//! # Usage
//!
//! ```rust,ignore
//! use crate::debug::{debug_enabled, debug_log};
//!
//! if debug_enabled() {
//!     debug_log!("[SCANBEAM] y={}", scanline_y);
//! }
//! ```

use std::sync::OnceLock;

/// Global flag indicating whether debug logging is enabled.
/// Initialized once from `WAGYU_DEBUG` environment variable.
static DEBUG_ENABLED: OnceLock<bool> = OnceLock::new();

/// Check if debug logging is enabled.
///
/// This function is efficient - it only checks the environment variable once.
#[inline]
pub fn debug_enabled() -> bool {
    *DEBUG_ENABLED.get_or_init(|| std::env::var("WAGYU_DEBUG").is_ok())
}

/// Log a debug message to stderr.
///
/// This macro only evaluates its arguments if debug logging is enabled.
#[macro_export]
macro_rules! debug_log {
    ($($arg:tt)*) => {
        if $crate::debug::debug_enabled() {
            eprintln!($($arg)*);
        }
    };
}

/// Log a scanbeam event.
#[inline]
pub fn log_scanbeam(y: f64) {
    if debug_enabled() {
        eprintln!("[SCANBEAM] y={:.0}", y);
    }
}

/// Log an active edge list addition.
#[inline]
pub fn log_ael_add(idx: usize, bot: (f64, f64), top: (f64, f64), poly_type: &str) {
    if debug_enabled() {
        eprintln!(
            "[AEL_ADD] idx={} bot=({:.0},{:.0}) top=({:.0},{:.0}) type={}",
            idx, bot.0, bot.1, top.0, top.1, poly_type
        );
    }
}

/// Log an active edge list removal.
#[inline]
pub fn log_ael_remove(idx: usize) {
    if debug_enabled() {
        eprintln!("[AEL_REMOVE] idx={}", idx);
    }
}

/// Log an intersection detection.
#[inline]
pub fn log_intersect(b1: usize, b2: usize, pt: (f64, f64)) {
    if debug_enabled() {
        eprintln!(
            "[INTERSECT] b1={} b2={} pt=({:.0},{:.0})",
            b1, b2, pt.0, pt.1
        );
    }
}

/// Log a new ring creation.
#[inline]
pub fn log_ring_new(id: usize, pt: (f64, f64)) {
    if debug_enabled() {
        eprintln!("[RING_NEW] id={} pt=({:.0},{:.0})", id, pt.0, pt.1);
    }
}

/// Log a point added to a ring.
#[inline]
pub fn log_ring_point(id: usize, pt: (f64, f64), to_front: bool) {
    if debug_enabled() {
        eprintln!(
            "[RING_POINT] id={} pt=({:.0},{:.0}) front={}",
            id, pt.0, pt.1, to_front
        );
    }
}

/// Log a ring merge operation.
#[inline]
pub fn log_ring_merge(from_id: usize, to_id: usize) {
    if debug_enabled() {
        eprintln!("[RING_MERGE] from={} to={}", from_id, to_id);
    }
}

/// Log a ring close operation.
#[inline]
pub fn log_ring_close(id: usize, point_count: usize) {
    if debug_enabled() {
        eprintln!("[RING_CLOSE] id={} points={}", id, point_count);
    }
}

/// Log winding count calculation.
#[inline]
pub fn log_winding(idx: usize, wc: i32, wc2: i32, delta: i32) {
    if debug_enabled() {
        eprintln!(
            "[WINDING] idx={} wc={} wc2={} delta={}",
            idx, wc, wc2, delta
        );
    }
}

/// Log contributing edge decision.
#[inline]
pub fn log_contributing(idx: usize, result: bool, reason: &str) {
    if debug_enabled() {
        eprintln!(
            "[CONTRIBUTING] idx={} result={} reason={}",
            idx, result, reason
        );
    }
}

/// Log horizontal edge processing.
#[inline]
pub fn log_horizontal(idx: usize, left: f64, right: f64) {
    if debug_enabled() {
        eprintln!(
            "[HORIZONTAL] idx={} left={:.0} right={:.0}",
            idx, left, right
        );
    }
}

/// Log local minimum insertion.
#[inline]
pub fn log_local_min(y: f64, left_idx: usize, right_idx: usize, poly_type: &str) {
    if debug_enabled() {
        eprintln!(
            "[LOCAL_MIN] y={:.0} left={} right={} type={}",
            y, left_idx, right_idx, poly_type
        );
    }
}

/// Log intersection handling result.
#[inline]
pub fn log_intersect_result(b1: usize, b2: usize, result: &str) {
    if debug_enabled() {
        eprintln!("[INTERSECT_RESULT] b1={} b2={} result={}", b1, b2, result);
    }
}

/// Log the start of vatti algorithm.
#[inline]
pub fn log_vatti_start(num_minima: usize, scanbeam_count: usize) {
    if debug_enabled() {
        eprintln!(
            "[VATTI_START] minima={} scanbeam={}",
            num_minima, scanbeam_count
        );
    }
}

/// Log the end of vatti algorithm.
#[inline]
pub fn log_vatti_end(ring_count: usize) {
    if debug_enabled() {
        eprintln!("[VATTI_END] rings={}", ring_count);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_debug_disabled_by_default() {
        // Note: This test assumes WAGYU_DEBUG is not set in the test environment
        // The OnceLock means we can't easily test both states
    }

    #[test]
    fn test_log_format_scanbeam() {
        // Just verify the format string compiles correctly
        let _ = format!("[SCANBEAM] y={:.0}", 100.0);
    }

    #[test]
    fn test_log_format_intersect() {
        let _ = format!("[INTERSECT] b1={} b2={} pt=({:.0},{:.0})", 1, 2, 5.0, 50.0);
    }
}
