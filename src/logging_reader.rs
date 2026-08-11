use std::fs::File;
use std::io::{BufWriter, Read, Result as IoResult, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const LOG_QUEUE_CAPACITY: usize = 1024;
const LOG_FLUSH_INTERVAL: Duration = Duration::from_secs(1);

#[derive(Default)]
pub struct LoggingStats {
    raw_bytes: AtomicU64,
    drops: AtomicU64,
    errors: AtomicU64,
}

impl LoggingStats {
    pub fn raw_bytes(&self) -> u64 {
        self.raw_bytes.load(Ordering::Relaxed)
    }
    pub fn drops(&self) -> u64 {
        self.drops.load(Ordering::Relaxed)
    }
    pub fn errors(&self) -> u64 {
        self.errors.load(Ordering::Relaxed)
    }
}

pub struct LoggingReader<R: Read> {
    inner: R,
    log_tx: Option<SyncSender<Vec<u8>>>,
    stats: Arc<LoggingStats>,
}

impl<R: Read + Send + 'static> LoggingReader<R> {
    pub fn new(inner: R) -> Self {
        Self {
            inner,
            log_tx: None,
            stats: Arc::new(LoggingStats::default()),
        }
    }

    pub fn with_session_log<P>(inner: R, log_dir: P) -> Self
    where
        P: AsRef<Path>,
    {
        Self::new_session_log(inner, Some(log_dir))
    }

    pub fn new_session_log<P>(inner: R, log_dir: Option<P>) -> Self
    where
        P: AsRef<Path>,
    {
        let log_tx = log_dir.map(|dir| {
            let log_path = PathBuf::from(dir.as_ref()).join(session_log_filename());
            let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(LOG_QUEUE_CAPACITY);
            let stats = Arc::new(LoggingStats::default());
            let writer_stats = stats.clone();

            thread::spawn(move || {
                if let Some(parent) = log_path.parent() {
                    if std::fs::create_dir_all(parent).is_err() {
                        writer_stats.errors.fetch_add(1, Ordering::Relaxed);
                        return;
                    }
                }

                let file = match File::create(log_path) {
                    Ok(file) => file,
                    Err(_) => {
                        writer_stats.errors.fetch_add(1, Ordering::Relaxed);
                        return;
                    }
                };
                let mut file = BufWriter::new(file);
                let mut last_flush = Instant::now();
                while let Ok(data) = rx.recv() {
                    if file.write_all(&data).is_err() {
                        writer_stats.errors.fetch_add(1, Ordering::Relaxed);
                    }
                    if last_flush.elapsed() >= LOG_FLUSH_INTERVAL {
                        if file.flush().is_err() {
                            writer_stats.errors.fetch_add(1, Ordering::Relaxed);
                        }
                        last_flush = Instant::now();
                    }
                }
                if file.flush().is_err() {
                    writer_stats.errors.fetch_add(1, Ordering::Relaxed);
                }
            });

            (tx, stats)
        });

        match log_tx {
            Some((log_tx, stats)) => Self {
                inner,
                log_tx: Some(log_tx),
                stats,
            },
            None => Self::new(inner),
        }
    }

    pub fn stats(&self) -> Arc<LoggingStats> {
        self.stats.clone()
    }
}

impl<R: Read> Read for LoggingReader<R> {
    fn read(&mut self, buf: &mut [u8]) -> IoResult<usize> {
        let bytes_read = self.inner.read(buf)?;
        if bytes_read > 0 {
            self.stats
                .raw_bytes
                .fetch_add(bytes_read as u64, Ordering::Relaxed);
            if let Some(tx) = &self.log_tx {
                match tx.try_send(buf[..bytes_read].to_vec()) {
                    Ok(()) => {}
                    Err(TrySendError::Full(_)) => {
                        self.stats.drops.fetch_add(1, Ordering::Relaxed);
                    }
                    Err(TrySendError::Disconnected(_)) => {
                        self.stats.errors.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
        Ok(bytes_read)
    }
}

fn session_log_filename() -> String {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    format!("sentilog_{}_{:09}.bin", now.as_secs(), now.subsec_nanos())
}
