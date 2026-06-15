use std::fs::File;
use std::io::{Read, Result as IoResult, Write};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Sender};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};

pub struct LoggingReader<R: Read> {
    inner: R,
    log_tx: Option<Sender<Vec<u8>>>,
}

impl<R: Read + Send + 'static> LoggingReader<R> {
    pub fn new(inner: R) -> Self {
        Self {
            inner,
            log_tx: None,
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
            let (tx, rx) = mpsc::channel::<Vec<u8>>();

            thread::spawn(move || {
                if let Some(parent) = log_path.parent() {
                    std::fs::create_dir_all(parent).expect("Failed to create log directory");
                }

                let mut file = File::create(log_path).expect("Failed to open log file");
                while let Ok(data) = rx.recv() {
                    let _ = file.write_all(&data);
                    let _ = file.flush();
                }
            });

            tx
        });

        Self { inner, log_tx }
    }
}

impl<R: Read> Read for LoggingReader<R> {
    fn read(&mut self, buf: &mut [u8]) -> IoResult<usize> {
        let bytes_read = self.inner.read(buf)?;
        if bytes_read > 0 {
            if let Some(tx) = &self.log_tx {
                let _ = tx.send(buf[..bytes_read].to_vec());
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
