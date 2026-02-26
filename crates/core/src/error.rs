//! Error types for wagyu operations

use thiserror::Error;

/// Errors that can occur during wagyu operations
#[derive(Error, Debug)]
pub enum WagyuError {
    /// Invalid geometry input
    #[error("Invalid geometry: {0}")]
    InvalidGeometry(String),

    /// Operation failed
    #[error("Operation failed: {0}")]
    OperationFailed(String),
}
