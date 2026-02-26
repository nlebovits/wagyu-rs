//! Interrupt handling for cancellation of long-running operations.
//!
//! PORT FROM: wagyu/include/mapbox/geometry/wagyu/interrupt.hpp
//!
//! This module provides a mechanism to interrupt wagyu operations. This is useful
//! for applications that need to cancel long-running polygon operations, such as
//! web servers with request timeouts.
//!
//! # Usage
//!
//! By default, interrupt checking is a no-op with zero overhead. To enable
//! interrupt support, use the provided functions:
//!
//! ```
//! use wagyu_rs::interrupt::{interrupt_request, interrupt_check, interrupt_reset};
//!
//! // In a separate thread or signal handler:
//! // interrupt_request();
//!
//! // In the wagyu operation (called periodically):
//! if let Err(_) = interrupt_check() {
//!     // Handle interruption
//! }
//! ```
//!
//! # Thread Safety
//!
//! The interrupt flag is thread-local, so each thread has its own independent
//! interrupt state. This means:
//! - `interrupt_request()` only affects the current thread
//! - Multi-threaded applications need to call `interrupt_request()` on each
//!   thread that should be interrupted
//!
//! # C++ Compatibility
//!
//! The C++ version throws `std::runtime_error` when interrupted. In Rust, we
//! return a `Result` instead. The behavior is:
//! - `interrupt_check()` returns `Ok(())` if not interrupted
//! - `interrupt_check()` returns `Err(InterruptError)` if interrupted
//! - Calling `interrupt_check()` also resets the flag (matches C++ behavior)

use std::cell::Cell;

// Thread-local interrupt flag
thread_local! {
    static INTERRUPT_REQUESTED: Cell<bool> = const { Cell::new(false) };
}

/// Error type returned when an operation is interrupted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterruptError;

impl std::fmt::Display for InterruptError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Wagyu interrupted")
    }
}

impl std::error::Error for InterruptError {}

/// Requests an interruption of the current thread's wagyu operations.
///
/// After calling this, the next `interrupt_check()` call on this thread will
/// return `Err(InterruptError)`.
///
/// # Examples
///
/// ```
/// use wagyu_rs::interrupt::{interrupt_request, interrupt_check, interrupt_reset};
///
/// // Request interruption
/// interrupt_request();
///
/// // Check returns error
/// assert!(interrupt_check().is_err());
///
/// // Flag is auto-reset after check, so next check succeeds
/// assert!(interrupt_check().is_ok());
/// ```
#[inline]
pub fn interrupt_request() {
    INTERRUPT_REQUESTED.with(|flag| flag.set(true));
}

/// Resets the interrupt flag without checking it.
///
/// This is useful if you want to clear a pending interrupt request without
/// triggering error handling.
#[inline]
pub fn interrupt_reset() {
    INTERRUPT_REQUESTED.with(|flag| flag.set(false));
}

/// Checks if an interrupt has been requested, and resets the flag.
///
/// This should be called periodically during long-running operations.
/// If an interrupt was requested, this returns `Err(InterruptError)` and
/// resets the flag (so subsequent checks will succeed unless interrupted again).
///
/// # Returns
///
/// - `Ok(())` if no interrupt was requested
/// - `Err(InterruptError)` if an interrupt was requested
///
/// # Examples
///
/// ```
/// use wagyu_rs::interrupt::{interrupt_check, interrupt_request};
///
/// // No interrupt requested
/// assert!(interrupt_check().is_ok());
///
/// // Request and check
/// interrupt_request();
/// assert!(interrupt_check().is_err());
/// ```
#[inline]
pub fn interrupt_check() -> Result<(), InterruptError> {
    INTERRUPT_REQUESTED.with(|flag| {
        if flag.get() {
            flag.set(false); // Reset after checking (matches C++ behavior)
            Err(InterruptError)
        } else {
            Ok(())
        }
    })
}

/// Returns whether an interrupt is currently requested without resetting the flag.
///
/// This is useful for checking the interrupt state without clearing it.
#[inline]
pub fn is_interrupt_requested() -> bool {
    INTERRUPT_REQUESTED.with(|flag| flag.get())
}

/// A guard that automatically resets the interrupt flag when dropped.
///
/// This is useful for ensuring the interrupt flag is cleaned up even if
/// an operation panics or returns early.
///
/// # Examples
///
/// ```
/// use wagyu_rs::interrupt::{InterruptGuard, interrupt_request, is_interrupt_requested};
///
/// interrupt_request();
/// assert!(is_interrupt_requested());
///
/// {
///     let _guard = InterruptGuard::new();
///     // Do work...
/// } // Flag is reset here
///
/// assert!(!is_interrupt_requested());
/// ```
pub struct InterruptGuard;

impl InterruptGuard {
    /// Creates a new interrupt guard.
    #[inline]
    pub fn new() -> Self {
        Self
    }
}

impl Default for InterruptGuard {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for InterruptGuard {
    fn drop(&mut self) {
        interrupt_reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Note: Tests run in parallel in separate threads, so thread-local state
    // won't interfere between tests.

    // =========================================================================
    // Basic functionality tests
    // =========================================================================

    #[test]
    fn test_interrupt_check_returns_ok_when_not_requested() {
        interrupt_reset(); // Ensure clean state
        assert!(interrupt_check().is_ok());
    }

    #[test]
    fn test_interrupt_request_and_check() {
        interrupt_reset(); // Ensure clean state
        interrupt_request();
        assert!(interrupt_check().is_err());
    }

    #[test]
    fn test_interrupt_check_resets_flag() {
        interrupt_reset();
        interrupt_request();

        // First check returns error
        assert!(interrupt_check().is_err());

        // Second check returns ok (flag was reset)
        assert!(interrupt_check().is_ok());
    }

    #[test]
    fn test_interrupt_reset() {
        interrupt_request();
        assert!(is_interrupt_requested());

        interrupt_reset();
        assert!(!is_interrupt_requested());
    }

    #[test]
    fn test_is_interrupt_requested_does_not_reset() {
        interrupt_reset();
        interrupt_request();

        // Check multiple times - flag should remain set
        assert!(is_interrupt_requested());
        assert!(is_interrupt_requested());
        assert!(is_interrupt_requested());

        // Clean up
        interrupt_reset();
    }

    // =========================================================================
    // Error type tests
    // =========================================================================

    #[test]
    fn test_interrupt_error_display() {
        let err = InterruptError;
        assert_eq!(format!("{}", err), "Wagyu interrupted");
    }

    #[test]
    fn test_interrupt_error_debug() {
        let err = InterruptError;
        assert_eq!(format!("{:?}", err), "InterruptError");
    }

    // =========================================================================
    // Guard tests
    // =========================================================================

    #[test]
    fn test_interrupt_guard_resets_on_drop() {
        interrupt_reset();
        interrupt_request();
        assert!(is_interrupt_requested());

        {
            let _guard = InterruptGuard::new();
            // Flag is still set while guard is alive
            assert!(is_interrupt_requested());
        }

        // Flag is reset after guard is dropped
        assert!(!is_interrupt_requested());
    }

    #[test]
    fn test_interrupt_guard_default() {
        interrupt_request();
        {
            let _guard = InterruptGuard;
        }
        assert!(!is_interrupt_requested());
    }

    // =========================================================================
    // Multiple request tests
    // =========================================================================

    #[test]
    fn test_multiple_requests_single_check() {
        interrupt_reset();

        // Multiple requests
        interrupt_request();
        interrupt_request();
        interrupt_request();

        // Single check clears
        assert!(interrupt_check().is_err());
        assert!(interrupt_check().is_ok());
    }
}
