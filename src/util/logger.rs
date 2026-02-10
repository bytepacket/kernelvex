//! Simple asynchronous logger with level filtering and multiple outputs.

use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::{self, BufWriter, Write};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::SystemTime;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Trace,
    Debug,
    Info,
    Warn,
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

#[derive(Clone)]
pub enum Output {
    Stdout,
    Stderr,
    File(Arc<Mutex<BufWriter<File>>>),
}

impl Output {
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

enum LogCommand {
    Message {
        level: Level,
        body: String,
        timestamp: SystemTime,
    },
    Flush,
    Shutdown,
}

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

pub struct LoggerHandle {
    handle: Option<JoinHandle<()>>,
    tx: Sender<LogCommand>,
}

impl LoggerHandle {
    /// Signals the logger thread to flush and stop.
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

/// Starts the logger thread and returns a logger and its handle.
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
