use std::fs::File;
use std::io::{self};

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

    let ospeed = termios::cfgetospeed(&termios);
    let ispeed = termios::cfgetispeed(&termios);

    println!("Output speed: {:?}", ospeed);
    println!("Input speed: {:?}", ispeed);

    // Check if speeds match what we set
    if ospeed == termios::BaudRate::B9600 && ispeed == termios::BaudRate::B9600 {
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
