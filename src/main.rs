use std::fs::File;
use std::io::{self, Read, Write};

fn configure_and_test_serial_port(port_path: &str) -> io::Result<()> {
    use nix::sys::termios;

    println!("Opening serial port: {}", port_path);

    let mut port = File::options().read(true).write(true).open(port_path)?;

    // Configure the port
    println!("Configuring serial port...");
    let mut termios =
        termios::tcgetattr(&port).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    // Set baud rate to 9600 (Note: ESP8266 bootloader typically uses 115200)
    termios::cfsetospeed(&mut termios, termios::BaudRate::B1152000)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    termios::cfsetispeed(&mut termios, termios::BaudRate::B1152000)
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

    // Test methods:

    // 1. Verify configuration by reading it back
    test_configuration_readback(&port)?;

    // 2. Test sync command
    test_sync_command(&mut port)?;

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
    if ospeed == termios::BaudRate::B1152000 && ispeed == termios::BaudRate::B1152000 {
        println!("✓ Baud rate configuration verified!");
    } else {
        println!("✗ Baud rate mismatch!");
    }

    Ok(())
}

fn test_sync_command(port: &mut File) -> io::Result<()> {
    println!("\n=== ESP8266 Bootloader Sync Test ===");

    // SLIP-encoded ESP_SYNC command
    let sync_frame = vec![
        0xC0, // SLIP start
        0x00, // Command: ESP_SYNC
        0x08, // Length
        0x07, 0x07, 0x12, 0x20, // Sync header
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, // 32 bytes of 0x55
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xC0, // SLIP end
    ];

    // Send sync frame
    port.write_all(&sync_frame)?;
    port.flush()?;
    println!("Sent sync frame: {:?}", sync_frame);

    // Wait for response
    std::thread::sleep(std::time::Duration::from_millis(500));

    // Read response with retries
    let mut buffer = [0u8; 128];
    for _ in 0..3 {
        match port.read(&mut buffer) {
            Ok(bytes_read) if bytes_read > 0 => {
                println!("Received: {:?}", &buffer[..bytes_read]);
                // Check for valid SLIP response
                if buffer[0] == 0xC0 && bytes_read > 1 && buffer[1] == 0x00 {
                    println!("✓ ESP8266 responded to sync command!");
                } else {
                    println!("✗ Unexpected response format");
                }
                return Ok(());
            }
            Ok(_) => {
                println!("No data received, retrying...");
                std::thread::sleep(std::time::Duration::from_millis(500));
            }
            Err(e) => {
                println!("Error reading: {:?}", e);
                return Err(e);
            }
        }
    }

    println!("No data received after retries");
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
