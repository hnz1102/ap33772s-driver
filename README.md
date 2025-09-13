# AP33772S Driver

A platform-agnostic Rust driver for the AP33772S USB Power Delivery (USB-PD) Sink Controller.

[![Crates.io](https://img.shields.io/crates/v/ap33772s-driver.svg)](https://crates.io/crates/ap33772s-driver)
[![Documentation](https://docs.rs/ap33772s-driver/badge.svg)](https://docs.rs/ap33772s-driver)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Features

- **Platform-agnostic**: Built on `embedded-hal` traits for maximum compatibility
- **USB-PD Support**: Full support for USB Power Delivery negotiation
- **SPR and EPR**: Supports both Standard Power Range (SPR) and Extended Power Range (EPR) modes
- **Programmable Power**: Support for Programmable Power Supply (PPS) and Adjustable Voltage Supply (AVS)
- **Protection Features**: Built-in over-voltage, over-current, over-temperature, and under-voltage protection
- **no_std Compatible**: Works in embedded environments without heap allocation (with `alloc` feature)

## Supported Voltages

The AP33772S supports negotiation for the following standard voltages:
- 5V (USB-PD default)
- 9V
- 12V
- 15V
- 20V (and higher with EPR)

Custom voltages and currents are also supported through programmable PDOs.

## Quick Start

Add this to your `Cargo.toml`:

```toml
[dependencies]
ap33772s-driver = "0.1"
```

For `std` environments, enable the `std` feature:

```toml
[dependencies]
ap33772s-driver = { version = "0.1", features = ["std"] }
```

## Example Usage

```rust
use ap33772s_driver::{AP33772S, PDVoltage};

// Create driver instance
let mut pd_controller = AP33772S::new();

// Initialize the controller (requires I2C implementation)
pd_controller.init(&mut i2c)?;

// Request 12V from the USB-PD source
pd_controller.request_voltage(&mut i2c, PDVoltage::V12)?;

// Read current status
let status = pd_controller.get_status(&mut i2c)?;
println!("Voltage: {}mV, Current: {}mA", status.voltage_mv, status.current_ma);

// Request custom voltage/current (if supported by source)
pd_controller.request_custom_voltage(&mut i2c, 15000, 2000)?; // 15V, 2A

// Get available power capabilities
for pdo in pd_controller.get_pdo_list() {
    println!("PDO {}: {}mV, {}mA, {}mW", 
        pdo.pdo_index, pdo.voltage_mv, pdo.current_ma, pdo.max_power_mw);
}
```

## Platform Integration

This driver uses the `embedded-hal` I2C traits, making it compatible with any platform that provides an I2C implementation. Here are examples for popular platforms:

### ESP32 (esp-idf-hal)

```rust
use esp_idf_hal::i2c::*;
use ap33772s_driver::AP33772S;

// Set up I2C
let i2c_config = I2cConfig::new().baudrate(100.kHz().into());
let mut i2c = I2cDriver::new(peripherals.i2c0, sda_pin, scl_pin, &i2c_config)?;

// Use the driver
let mut pd_controller = AP33772S::new();
pd_controller.init(&mut i2c)?;
```

### STM32 (stm32f4xx-hal)

```rust
use stm32f4xx_hal::i2c::I2c;
use ap33772s_driver::AP33772S;

let mut i2c = I2c::new(/* I2C peripheral setup */);
let mut pd_controller = AP33772S::new();
pd_controller.init(&mut i2c)?;
```

### Raspberry Pi (rppal)

```rust
use rppal::i2c::I2c;
use ap33772s_driver::AP33772S;

let mut i2c = I2c::new()?;
let mut pd_controller = AP33772S::new();
pd_controller.init(&mut i2c)?;
```

## Error Handling

The driver uses a comprehensive error type that wraps the underlying I2C errors:

```rust
match pd_controller.request_voltage(&mut i2c, PDVoltage::V12) {
    Ok(()) => println!("Voltage request successful"),
    Err(ap33772s_driver::Error::Timeout) => println!("PD negotiation timed out"),
    Err(ap33772s_driver::Error::NegotiationFailed) => println!("PD negotiation failed"),
    Err(ap33772s_driver::Error::ProtectionFault) => println!("Protection fault occurred"),
    Err(e) => println!("Other error: {:?}", e),
}
```

## Register Map

The driver provides access to all AP33772S registers through constants:

- **Status and Control**: `REG_STATUS`, `REG_CONFIG`, `REG_SYSTEM`
- **Measurements**: `REG_VOLTAGE`, `REG_CURRENT`, `REG_TEMP`
- **Power Delivery**: `REG_VREQ`, `REG_IREQ`, `REG_PD_REQMSG`
- **Source Capabilities**: `REG_SRCPDO`, `REG_SRC_SPR_PDO1`..`REG_SRC_EPR_PDO13`

## Protection Features

Configure the built-in protection features:

```rust
// Enable all protections
pd_controller.configure_protections(
    &mut i2c,
    true,  // Under-voltage protection
    true,  // Over-voltage protection  
    true,  // Over-current protection
    true,  // Over-temperature protection
    true   // De-rating function
)?;
```

## Power Data Objects (PDOs)

The driver automatically enumerates all available PDOs from the connected source:

```rust
for pdo in pd_controller.get_pdo_list() {
    println!("PDO {}: {}V @ {}A ({}W) - {}", 
        pdo.pdo_index,
        pdo.voltage_mv as f32 / 1000.0,
        pdo.current_ma as f32 / 1000.0,
        pdo.max_power_mw as f32 / 1000.0,
        if pdo.is_fixed { "Fixed" } else { "Programmable" }
    );
}
```

## Advanced Features

### VOUT Control

Control the output voltage manually:

```rust
// Auto control (default)
pd_controller.set_vout_auto_control(&mut i2c)?;

// Force output off
pd_controller.force_vout_off(&mut i2c)?;

// Force output on
pd_controller.force_vout_on(&mut i2c)?;
```

### Hard Reset

Perform a USB-PD hard reset:

```rust
pd_controller.hard_reset(&mut i2c)?;
```

## Specifications

- **I2C Address**: 0x52 (7-bit)
- **I2C Speed**: Up to 400kHz
- **Supply Voltage**: 3.3V or 5V
- **Temperature Range**: -40°C to +85°C
- **USB-PD Compliance**: USB-PD 3.1 compatible

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## References

- [AP33772S Datasheet](https://www.diodes.com/assets/Datasheets/AP33772S.pdf)
- [USB Power Delivery Specification](https://www.usb.org/documents)
- [embedded-hal Documentation](https://docs.rs/embedded-hal/)
