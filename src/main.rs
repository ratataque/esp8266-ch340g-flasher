use config::Config;
use nix::sys::termios::{self, BaudRate};
use serde::Deserialize;
use std::io::{self, Read, Write};

#[derive(Debug, Deserialize)]
struct Settings {
    serial_interface: String,
}

#[cfg(any(target_os = "linux", target_os = "android"))]
fn speed_to_baudrate(speed: termios::BaudRate) -> Option<termios::BaudRate> {
    Some(speed)
}

#[cfg(any(
    target_os = "freebsd",
    target_os = "netbsd",
    target_os = "openbsd",
    target_os = "dragonfly",
    target_os = "macos"
))]
fn speed_to_baudrate(speed: u32) -> Option<termios::BaudRate> {
    match speed {
        0 => Some(BaudRate::B0),
        50 => Some(BaudRate::B50),
        75 => Some(BaudRate::B75),
        110 => Some(BaudRate::B110),
        134 => Some(BaudRate::B134),
        150 => Some(BaudRate::B150),
        200 => Some(BaudRate::B200),
        300 => Some(BaudRate::B300),
        600 => Some(BaudRate::B600),
        1200 => Some(BaudRate::B1200),
        1800 => Some(BaudRate::B1800),
        2400 => Some(BaudRate::B2400),
        4800 => Some(BaudRate::B4800),
        9600 => Some(BaudRate::B9600),
        19200 => Some(BaudRate::B19200),
        38400 => Some(BaudRate::B38400),
        57600 => Some(BaudRate::B57600),
        115200 => Some(BaudRate::B115200),
        230400 => Some(BaudRate::B230400),
        // Add more as needed
        _ => None,
    }
}

fn configure_and_test_serial_port(port_path: &str, speed: termios::BaudRate) -> io::Result<()> {
    println!("Opening serial port: {}", port_path);

    let mut port = std::fs::File::options()
        .read(true)
        .write(true)
        .open(port_path)?;

    // Configure the port
    println!("Configuring serial port...");
    let mut termios =
        termios::tcgetattr(&port).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    // Set baud rate to 9600 (Note: ESP8266 bootloader typically uses 115200)
    termios::cfsetospeed(&mut termios, speed)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    termios::cfsetispeed(&mut termios, speed)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    // Set timeout (e.g., 1 second = 10 deciseconds)
    termios.control_chars[termios::SpecialCharacterIndices::VTIME as usize] = 10; // 1 second timeout
    termios.control_chars[termios::SpecialCharacterIndices::VMIN as usize] = 0; // Non-blocking read

    // Configure other settings
    termios.control_flags |= termios::ControlFlags::CLOCAL | termios::ControlFlags::CREAD; // Enable receiver, local mode
    termios.control_flags &= !termios::ControlFlags::PARENB; // No parity
    termios.control_flags &= !termios::ControlFlags::CSTOPB; // 1 stop bit
    termios.control_flags |= termios::ControlFlags::CS8; // 8 data bits
    termios.input_flags &= !(termios::InputFlags::IGNBRK
        | termios::InputFlags::BRKINT
        | termios::InputFlags::PARMRK
        | termios::InputFlags::ISTRIP
        | termios::InputFlags::INLCR
        | termios::InputFlags::IGNCR
        | termios::InputFlags::ICRNL
        | termios::InputFlags::IXON);
    termios.output_flags &= !termios::OutputFlags::OPOST;
    termios.local_flags &= !(termios::LocalFlags::ECHO
        | termios::LocalFlags::ECHONL
        | termios::LocalFlags::ICANON
        | termios::LocalFlags::ISIG
        | termios::LocalFlags::IEXTEN);

    // Apply settings
    termios::tcsetattr(&port, termios::SetArg::TCSANOW, &termios)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    println!("Serial port configured successfully!");

    // 1. Verify configuration by reading it back
    test_configuration_readback(&port, speed)?;

    // 2. Test sync command
    test_sync_command(&mut port)?;

    Ok(())
}

// Test 1: Verify the configuration was applied
fn test_configuration_readback(port: &std::fs::File, speed: termios::BaudRate) -> io::Result<()> {
    println!("\n=== Configuration Verification ===");

    let termios = termios::tcgetattr(port).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    let ospeed_raw = termios::cfgetospeed(&termios);
    let ispeed_raw = termios::cfgetispeed(&termios);

    let ospeed = speed_to_baudrate(ospeed_raw).unwrap_or(BaudRate::B0);
    let ispeed = speed_to_baudrate(ispeed_raw).unwrap_or(BaudRate::B0);

    println!("Output speed: {:?}", ospeed);
    println!("Input speed: {:?}", ispeed);

    // Check if speeds match what we set
    if ospeed == speed && ispeed == speed {
        println!("✓ Baud rate configuration verified!");
    } else {
        println!("✗ Baud rate mismatch!");
    }

    Ok(())
}

fn test_sync_command(port: &mut std::fs::File) -> io::Result<()> {
    println!("\n=== ESP8266 Bootloader Sync Test ===");

    // SLIP-encoded ESP_SYNC command
    // https://docs.espressif.com/projects/esptool/en/latest/esp8266/advanced-topics/serial-protocol.html
    // https://docs.espressif.com/projects/esptool/en/latest/esp8266/advanced-topics/serial-protocol.html#commands
    let sync_frame = vec![
        0xC0, // SLIP start
        0x00, // Always 0x00 for requests
        0x08, // command sync
        0x24, 0x00, // 0x24 is the length of the payload (0x00)
        0x00, 0x00, 0x00, 0x00, // checksum
        0x07, 0x07, 0x12, 0x20, // Sync header
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, // 32 bytes of 0x55 0x55,
        0xC0, // SLIP end
    ];
    // Send sync frame
    port.write_all(&sync_frame)?;
    port.flush()?;
    println!("Sent sync frame: {:02X?}", sync_frame);

    let mut buffer = [0u8; 128];
    let mut response = Vec::new();

    for _ in 0..3 {
        match port.read(&mut buffer) {
            Ok(0) => break, // EOF or no more data
            Ok(n) => {
                response.extend_from_slice(&buffer[..n]);
                if buffer[0] == 0xC0 && n > 1 && buffer[1] == 0x00 {
                    println!("✓ ESP8266 responded to sync command!");
                }
                // break;
            }
            Err(e) => {
                println!("Error reading: {:?}", e);
                return Err(e);
            }
        }
    }

    println!("Received: {:02X?}", response);

    Ok(())
}

fn config_setup() -> Result<Settings, config::ConfigError> {
    Config::builder()
        .add_source(config::File::with_name("config/default.toml").required(true))
        .add_source(config::File::with_name("config/local.toml").required(false))
        // .add_source(Environment::default().separator("__"))
        .build()?
        .try_deserialize()
}

fn main() -> io::Result<()> {
    // Load configuration
    let settings: Settings = match config_setup() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Error loading configuration: {}", e);
            return Ok(());
        }
    };

    // Check if port exists
    if !std::path::Path::new(&settings.serial_interface).exists() {
        eprintln!(
            "Error: Serial port {} does not exist",
            &settings.serial_interface
        );
        return Ok(());
    }

    configure_and_test_serial_port(&settings.serial_interface, termios::BaudRate::B115200)?;

    Ok(())
}
