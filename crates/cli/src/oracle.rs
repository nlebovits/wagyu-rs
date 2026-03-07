//! wagyu-oracle: CLI tool for oracle testing against C++ wagyu
//!
//! This tool reads a JSON file containing subject and clip polygons,
//! runs the specified boolean operation, and outputs the result as JSON.
//!
//! Usage: wagyu-oracle <input.json> <operation> [fill_type]
//!
//! Input format:
//! {
//!   "subject": [[[[x,y], [x,y], ...]]], // MultiPolygon: array of polygons
//!   "clip": [[[[x,y], [x,y], ...]]]     // Optional
//! }

use anyhow::{Context, Result};
use clap::Parser;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;

use wagyu_rs::{
    config::{FillType, PolygonType},
    point::Point,
    wagyu::Wagyu,
    Operation,
};

#[derive(Parser)]
#[command(name = "wagyu-oracle")]
#[command(about = "Oracle CLI for comparing Rust wagyu output against C++ wagyu")]
struct Cli {
    /// Input JSON file containing subject and clip polygons
    input: PathBuf,

    /// Boolean operation: union, intersection, difference, xor
    operation: String,

    /// Fill type: evenodd (default), nonzero, positive, negative
    #[arg(default_value = "evenodd")]
    fill_type: String,
}

/// Input format for oracle testing
#[derive(Debug, Deserialize)]
struct OracleInput {
    /// Subject polygons: array of polygons, each polygon is array of rings
    subject: Vec<Vec<Vec<[i64; 2]>>>,

    /// Clip polygons (optional)
    #[serde(default)]
    clip: Vec<Vec<Vec<[i64; 2]>>>,
}

/// Output format: MultiPolygon as nested arrays
#[derive(Debug, Serialize)]
struct OracleOutput(Vec<Vec<Vec<[i64; 2]>>>);

fn parse_operation(s: &str) -> Result<Operation> {
    match s.to_lowercase().as_str() {
        "union" => Ok(Operation::Union),
        "intersection" => Ok(Operation::Intersection),
        "difference" => Ok(Operation::Difference),
        "xor" => Ok(Operation::Xor),
        _ => anyhow::bail!(
            "Unknown operation: {}. Use: union, intersection, difference, xor",
            s
        ),
    }
}

fn parse_fill_type(s: &str) -> Result<FillType> {
    match s.to_lowercase().as_str() {
        "evenodd" => Ok(FillType::EvenOdd),
        "nonzero" => Ok(FillType::NonZero),
        "positive" => Ok(FillType::Positive),
        "negative" => Ok(FillType::Negative),
        _ => anyhow::bail!(
            "Unknown fill type: {}. Use: evenodd, nonzero, positive, negative",
            s
        ),
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Parse operation and fill type
    let operation = parse_operation(&cli.operation)?;
    let fill_type = parse_fill_type(&cli.fill_type)?;

    // Read input file
    let content = fs::read_to_string(&cli.input)
        .with_context(|| format!("Failed to read input file: {:?}", cli.input))?;

    let input: OracleInput =
        serde_json::from_str(&content).with_context(|| "Failed to parse input JSON")?;

    // Create wagyu instance
    let mut wagyu: Wagyu<i64> = Wagyu::new();

    // Add subject polygons
    for polygon in &input.subject {
        for ring in polygon {
            let points: Vec<Point<i64>> = ring.iter().map(|[x, y]| Point::new(*x, *y)).collect();
            wagyu.add_ring(&points, PolygonType::Subject);
        }
    }

    // Add clip polygons
    for polygon in &input.clip {
        for ring in polygon {
            let points: Vec<Point<i64>> = ring.iter().map(|[x, y]| Point::new(*x, *y)).collect();
            wagyu.add_ring(&points, PolygonType::Clip);
        }
    }

    // Execute operation
    let result = wagyu
        .execute(operation, fill_type, fill_type)
        .with_context(|| "Wagyu execution failed")?;

    // Convert result to output format
    // result is a geo_types::MultiPolygon<i64>
    let output: Vec<Vec<Vec<[i64; 2]>>> = result
        .iter()
        .map(|polygon: &geo_types::Polygon<i64>| {
            // Each polygon has exterior + interiors
            let mut rings = Vec::new();

            // Exterior ring
            let exterior: Vec<[i64; 2]> = polygon.exterior().coords().map(|c| [c.x, c.y]).collect();
            rings.push(exterior);

            // Interior rings (holes)
            for interior in polygon.interiors() {
                let hole: Vec<[i64; 2]> = interior.coords().map(|c| [c.x, c.y]).collect();
                rings.push(hole);
            }

            rings
        })
        .collect();

    // Output as JSON
    println!("{}", serde_json::to_string(&output)?);

    Ok(())
}
