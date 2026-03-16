//! Simple asynchronous logger with level filtering and multiple outputs.
//!
//! This module provides a lightweight logging system designed for VEX robotics
//! applications. It runs a background thread to handle log output asynchronously,
//! preventing logging from blocking time-critical robot code.
//!
//! # Features
//!
//! - **Level filtering**: Filter messages by severity (Trace, Debug, Info, Warn, Error)
//! - **Multiple outputs**: Log to stdout, stderr, or a file
//! - **Async design**: Background thread handles I/O to avoid blocking
//! - **Colored output**: Terminal output includes ANSI colors for readability
//! - **Thread-safe**: Logger can be cloned and used from multiple threads
//!
//! # Usage
//!
//! ```ignore
//! use kernelvex::util::logger::{init, Level};
//!
//! // Initialize the logger
//! let (logger, handle) = init();
//!
//! // Configure logging level and output
//! let logger = logger.level(Level::Debug).stdout();
//!
//! // Log messages
//! logger.info("Robot initialized");
//! logger.debug("Motor voltages: left=5.0, right=5.0");
//! logger.warn("Battery low");
//!
//! // Logger thread stops when handle is dropped
//! drop(handle);
//! ```
//!
//! # Output Format
//!
//! Log messages are formatted as:
//! ```text
//! [timestamp] [LEVEL] [tid:thread_id] message
//! ```

use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::{self, BufWriter, Write};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::SystemTime;

/// Log level severity.
///
/// Levels are ordered from least to most severe. When a minimum level is set,
/// only messages at that level or higher will be logged.
///
/// # Ordering
///
/// `Trace < Debug < Info < Warn < Error`
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Finest-grained information, typically for debugging specific issues
    Trace,
    /// Detailed information useful during development
    Debug,
    /// General operational information
    Info,
    /// Potentially problematic situations
    Warn,
    /// Error conditions that may allow continued operation
    Error,
}

impl fmt::Display for Level {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Level::Trace => write!(f, "TRACE"),
            Level::Debug => write!(f, "DEBUG"),
            Level::Info => write!(f, "INFO"),
            Level::Warn => write!(f, "WARN"),
            Level::Error => write!(f, "ERROR"),
        }
    }
}

impl Level {
    /// Returns the ANSI color code for this log level.
    ///
    /// Used for colored terminal output:
    /// - Trace: Cyan
    /// - Debug: Blue
    /// - Info: Green
    /// - Warn: Yellow
    /// - Error: Red
    fn color(&self) -> &'static str {
        match self {
            Level::Trace => "\x1b[36m", // cyan
            Level::Debug => "\x1b[34m", // blue
            Level::Info => "\x1b[32m",  // green
            Level::Warn => "\x1b[33m",  // yellow
            Level::Error => "\x1b[31m", // red
        }
    }
}

/// Output destination for log messages.
///
/// Logs can be directed to standard output, standard error, or a file.
/// The output can be changed at runtime using the [`Logger`] builder methods.
#[derive(Clone)]
pub enum Output {
    /// Write to standard output (with colors)
    Stdout,
    /// Write to standard error (with colors)
    Stderr,
    /// Write to a file (no colors)
    File(Arc<Mutex<BufWriter<File>>>),
}

impl Output {
    /// Writes bytes to this output destination.
    ///
    /// Errors are printed to stderr but do not propagate.
    fn write(&mut self, buf: &[u8]) {
        let result = match self {
            Output::Stdout => io::stdout().write_all(buf),
            Output::Stderr => io::stderr().write_all(buf),
            Output::File(f) => {
                let mut guard = f.lock().unwrap();
                guard.write_all(buf).and_then(|_| guard.flush())
            }
        };
        if let Err(e) = result {
            eprintln!("[logger] write error: {e}");
        }
    }

    /// Flushes any buffered output.
    ///
    /// Ensures all pending bytes are written to the underlying destination.
    fn flush(&mut self) {
        let result = match self {
            Output::Stdout => io::stdout().flush(),
            Output::Stderr => io::stderr().flush(),
            Output::File(f) => f.lock().unwrap().flush(),
        };
        if let Err(e) = result {
            eprintln!("[logger] flush error: {e}");
        }
    }
}

/// Internal command sent to the logger background thread.
enum LogCommand {
    /// A log message to be written
    Message {
        level: Level,
        body: String,
        timestamp: SystemTime,
    },
    /// Request to flush buffered output
    Flush,
    /// Request to shut down the logger thread
    Shutdown,
}

/// A cloneable handle for logging messages.
///
/// `Logger` provides methods for logging at various levels and configuring
/// the logging behavior. It can be freely cloned and shared across threads.
///
/// Messages are sent to a background thread for async I/O, so logging
/// calls return immediately without blocking.
///
/// # Configuration
///
/// Use the builder-style methods to configure the logger:
/// - [`level()`](Self::level): Set minimum log level
/// - [`stdout()`](Self::stdout): Route output to stdout
/// - [`stderr()`](Self::stderr): Route output to stderr
/// - [`file()`](Self::file): Route output to a file
///
/// # Example
///
/// ```ignore
/// let (logger, handle) = init();
/// let logger = logger.level(Level::Info).stdout();
///
/// logger.info("Starting autonomous");
/// logger.debug("This won't print because level is Info");
/// ```
#[derive(Clone)]
pub struct Logger {
    level: Level,
    tx: Sender<LogCommand>,
    output: Arc<Mutex<Output>>,
}

impl Logger {
    /// Logs a message at the given level if it meets the current threshold.
    pub fn log(&self, level: Level, message: &str) {
        if level < self.level {
            return;
        }
        let _ = self.tx.send(LogCommand::Message {
            level,
            body: message.to_owned(),
            timestamp: SystemTime::now(),
        });
    }

    /// Flushes any buffered log output.
    pub fn flush(&self) {
        let _ = self.tx.send(LogCommand::Flush);
    }

    /// Logs a trace-level message.
    pub fn trace(&self, msg: &str) {
        self.log(Level::Trace, msg);
    }
    /// Logs a debug-level message.
    pub fn debug(&self, msg: &str) {
        self.log(Level::Debug, msg);
    }
    /// Logs an info-level message.
    pub fn info(&self, msg: &str) {
        self.log(Level::Info, msg);
    }
    /// Logs a warning-level message.
    pub fn warn(&self, msg: &str) {
        self.log(Level::Warn, msg);
    }
    /// Logs an error-level message.
    pub fn error(&self, msg: &str) {
        self.log(Level::Error, msg);
    }

    /// Sets the minimum log level for this logger.
    pub fn level(mut self, level: Level) -> Self {
        self.level = level;
        self
    }

    /// Routes output to stdout.
    pub fn stdout(self) -> Self {
        let mut guard = self.output.lock().unwrap();
        *guard = Output::Stdout;
        drop(guard);
        self
    }

    /// Routes output to stderr.
    pub fn stderr(self) -> Self {
        let mut guard = self.output.lock().unwrap();
        *guard = Output::Stderr;
        drop(guard);
        self
    }

    /// Routes output to a file at the provided path.
    pub fn file(self, path: &str) -> io::Result<Self> {
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        let mut guard = self.output.lock().unwrap();
        *guard = Output::File(Arc::new(Mutex::new(BufWriter::new(file))));
        drop(guard);
        Ok(self)
    }
}

/// Handle to the logger background thread.
///
/// `LoggerHandle` owns the logger thread and ensures proper cleanup on drop.
/// When the handle is dropped (or [`shutdown()`](Self::shutdown) is called),
/// the background thread is signaled to flush and terminate.
///
/// # Ownership
///
/// Keep this handle alive for the duration you want logging to work.
/// The [`Logger`] instances will continue to send messages, but they
/// won't be written once the handle is dropped.
///
/// # Example
///
/// ```ignore
/// let (logger, handle) = init();
///
/// // ... use logger throughout your program ...
///
/// // Explicit shutdown (optional - happens automatically on drop)
/// handle.shutdown();
/// ```
pub struct LoggerHandle {
    handle: Option<JoinHandle<()>>,
    tx: Sender<LogCommand>,
}

impl LoggerHandle {
    /// Signals the logger thread to flush and stop.
    ///
    /// This method:
    /// 1. Sends a shutdown command to the background thread
    /// 2. Waits for the thread to finish processing pending messages
    /// 3. Joins the thread to ensure clean termination
    ///
    /// Called automatically when the handle is dropped.
    pub fn shutdown(&mut self) {
        let _ = self.tx.send(LogCommand::Shutdown);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for LoggerHandle {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Initializes the logging system and returns a logger and its handle.
///
/// This function spawns a background thread named "kernelvex::logger" that
/// processes log messages asynchronously. The thread runs until the
/// [`LoggerHandle`] is dropped or [`shutdown()`](LoggerHandle::shutdown) is called.
///
/// # Returns
///
/// A tuple of:
/// - [`Logger`]: Cloneable handle for logging messages (default: Trace level, stdout)
/// - [`LoggerHandle`]: Ownership handle for the background thread
///
/// # Example
///
/// ```ignore
/// let (logger, handle) = init();
/// let logger = logger.level(Level::Info);
///
/// logger.info("Logger initialized");
/// ```
///
/// # Panics
///
/// Panics if the background thread cannot be spawned.
pub fn init() -> (Logger, LoggerHandle) {
    let (tx, rx) = mpsc::channel::<LogCommand>();

    let output = Arc::new(Mutex::new(Output::Stdout));
    let thread_output = Arc::clone(&output);

    let handle = thread::Builder::new()
        .name("kernelvex::logger".into())
        .spawn(move || {
            for cmd in rx {
                match cmd {
                    LogCommand::Message {
                        level,
                        body,
                        timestamp,
                    } => {
                        let ts = humantime::format_rfc3339_seconds(timestamp);
                        let tid = thread_id::get();
                        let mut guard = thread_output.lock().unwrap();
                        let line = if !matches!(*guard, Output::File(_)) {
                            let color = level.color();
                            let reset = "\x1b[0m";
                            format!("[{ts}] [{color}{level}{reset}] [tid:{tid}] {body}\n")
                        } else {
                            format!("[{ts}] [{level}] [tid:{tid}] {body}\n")
                        };

                        guard.write(line.as_bytes());
                    }
                    LogCommand::Flush => {
                        let mut guard = thread_output.lock().unwrap();
                        guard.flush();
                    }
                    LogCommand::Shutdown => {
                        let mut guard = thread_output.lock().unwrap();
                        guard.flush();
                        break;
                    }
                }
            }
        })
        .expect("failed to spawn logger thread");

    let logger = Logger {
        level: Level::Trace,
        tx: tx.clone(),
        output,
    };
    let owner = LoggerHandle {
        handle: Some(handle),
        tx,
    };

    (logger, owner)
}
