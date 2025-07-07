use std::fs::File;
use std::io::{self};

use nix::sys::termios::BaudRate;

fn u32_to_baudrate(val: u32) -> Option<BaudRate> {
    match val {
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
        _ => None,
    }
}

fn configure_and_test_serial_port(port_path: &str) -> io::Result<()> {
    use nix::sys::termios;

    println!("Opening serial port: {}", port_path);

    let port = File::options().read(true).write(true).open(port_path)?;

    // Configure the port
    println!("Configuring serial port...");
    let mut termios =
        termios::tcgetattr(&port).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    // Set baud rate to 9600
    termios::cfsetospeed(&mut termios, termios::BaudRate::B9600)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    termios::cfsetispeed(&mut termios, termios::BaudRate::B9600)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    // Apply settings
    termios::tcsetattr(&port, termios::SetArg::TCSANOW, &termios)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    println!("Serial port configured successfully!");

    // Test methods:

    // 1. Verify configuration by reading it back
    test_configuration_readback(&port)?;

    Ok(())
}

// Test 1: Verify the configuration was applied
fn test_configuration_readback(port: &File) -> io::Result<()> {
    use nix::sys::termios;

    println!("\n=== Configuration Verification ===");

    let termios = termios::tcgetattr(port).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    let ospeed_raw = termios::cfgetospeed(&termios);
    let ispeed_raw = termios::cfgetispeed(&termios);

    let ospeed = u32_to_baudrate(ospeed_raw as u32);
    let ispeed = u32_to_baudrate(ispeed_raw as u32);

    println!("Output speed: {:?}", ospeed);
    println!("Input speed: {:?}", ispeed);

    // Check if speeds match what we set
    if ospeed == Some(termios::BaudRate::B9600) && ispeed == Some(termios::BaudRate::B9600) {
        println!("✓ Baud rate configuration verified!");
    } else {
        println!("✗ Baud rate mismatch!");
    }

    Ok(())
}

fn main() -> io::Result<()> {
    let port_path = "/dev/ttyUSB0"; // Change this to your actual port

    // Check if port exists
    if !std::path::Path::new(port_path).exists() {
        eprintln!("Error: Serial port {} does not exist", port_path);
        return Ok(());
    }

    configure_and_test_serial_port(port_path)?;

    Ok(())
}
