use nix::sys::termios;
use nix::sys::termios::BaudRate;
use std::io;
use std::fs::File;
#[cfg(any(target_os = "linux", target_os = "android"))]
fn speed_to_baudrate(speed: termios::BaudRate) -> Option<termios::BaudRate> {
    Some(speed)
}

#[cfg(any(target_os = "freebsd", target_os = "netbsd", target_os = "openbsd", target_os = "dragonfly", target_os = "macos"))]
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
fn configure_and_test_serial_port(port_path: &str) -> io::Result<()> {
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
    println!("\n=== Configuration Verification ===");

    let termios = termios::tcgetattr(port).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    let ospeed_raw = termios::cfgetospeed(&termios);
    let ispeed_raw = termios::cfgetispeed(&termios);

    let ospeed = speed_to_baudrate(ospeed_raw).unwrap_or(BaudRate::B0);
    let ispeed = speed_to_baudrate(ispeed_raw).unwrap_or(BaudRate::B0);

    println!("Output speed: {:?}", ospeed);
    println!("Input speed: {:?}", ispeed);

    // Check if speeds match what we set
    if ospeed == BaudRate::B9600 && ispeed == BaudRate::B9600 {
        println!("✓ Baud rate configuration verified!");
    } else {
        println!("✗ Baud rate mismatch!");
    }

    Ok(())
}

fn main() -> io::Result<()> {
    let port_path = "/dev/cu.wchusbserial10"; // Change this to your actual port

    // Check if port exists
    if !std::path::Path::new(port_path).exists() {
        eprintln!("Error: Serial port {} does not exist", port_path);
        return Ok(());
    }

    configure_and_test_serial_port(port_path)?;

    Ok(())
}
