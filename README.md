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
- 20V
- 28V (EPR) - Maximum supported by AP33772S

**Note**: Previous versions incorrectly indicated support for 36V and 48V. The AP33772S hardware actually supports a maximum of 28V.

The driver supports both Standard Power Range (SPR) and Extended Power Range (EPR) modes.
Custom voltages and currents are also supported through programmable PDOs up to the 28V maximum.

## Quick Start

Add this to your `Cargo.toml`:

```toml
[dependencies]
ap33772s-driver = "0.1.4"
```

For `std` environments, enable the `std` feature:

```toml
[dependencies]
ap33772s-driver = { version = "0.1.4", features = ["std"] }
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

## Changelog

### Version 0.1.4

#### Breaking Changes
- **Corrected Maximum Voltage Support**: Removed support for voltages above 28V to accurately reflect AP33772S hardware limitations
- **Updated PDVoltage Enum**: Removed `V36`, `V40`, and `V48` variants from the `PDVoltage` enum
- **Hardware Specification Compliance**: Driver now correctly represents that AP33772S supports a maximum of 28V, not 48V as previously documented

#### Changes
- **PDVoltage::V36 Removed**: The 36V option has been removed from the voltage enumeration
- **PDVoltage::V40 Removed**: The 40V option has been removed from the voltage enumeration  
- **PDVoltage::V48 Removed**: The 48V option has been removed from the voltage enumeration
- **Updated Documentation**: All references to voltages above 28V have been corrected
- **Voltage Mapping Updated**: Request voltage mapping now correctly handles only supported voltages

#### Migration Guide
If your code was using unsupported voltage options:
```rust
// Before (will no longer compile)
pd_controller.request_voltage(&mut i2c, &mut delay, PDVoltage::V36)?;
pd_controller.request_voltage(&mut i2c, &mut delay, PDVoltage::V48)?;

// After (use maximum supported voltage or custom voltage)
pd_controller.request_voltage(&mut i2c, &mut delay, PDVoltage::V28)?;
// Or use custom voltage up to 28V
pd_controller.request_custom_voltage(&mut i2c, &mut delay, 28000, 3000)?;
```

This update ensures the driver accurately represents the actual hardware capabilities of the AP33772S controller and prevents attempts to negotiate unsupported voltages.

### Version 0.1.3

#### New Features
- **Enhanced Variable PDO Support**: Improved `request_custom_voltage` method to prioritize Variable Power Data Objects (APDOs) over Fixed PDOs when available
- **Smart PDO Selection**: Added intelligent PDO selection algorithm with the following priority:
  1. Variable PDO that can provide exact requested voltage
  2. Variable PDO with smallest voltage difference
  3. Fixed PDO with smallest voltage difference (voltage >= requested)
- **Custom Voltage Encoding**: Variable PDOs now support precise voltage requests through proper message encoding (100mV or 200mV resolution)

#### Improvements
- **Better Power Matching**: When multiple PDOs have the same voltage difference, the driver now selects the one with higher power capability
- **Enhanced Documentation**: Added detailed comments explaining the PDO selection logic and priority system
- **Voltage Request Accuracy**: For Variable PDOs, the driver can now request voltages closer to the exact requirement rather than defaulting to maximum PDO voltage

#### Technical Details
- Variable PDOs (APDO - Adjustable Power Data Objects) are now properly detected using the `is_fixed` flag
- Request messages for Variable PDOs include the specific voltage requirement encoded in 50mV units
- Fixed PDOs continue to use standard maximum voltage encoding for compatibility

#### Example Usage
```rust
// The driver will now automatically select the best PDO type
// If a Variable PDO can provide 15V, it will be chosen over a 20V Fixed PDO
pd_controller.request_custom_voltage(&mut i2c, &mut delay, 15000, 2000)?; // 15V, 2A
```

This update significantly improves power efficiency and voltage accuracy when working with modern USB-PD sources that support Variable PDOs.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## References

- [AP33772S Datasheet](https://www.diodes.com/assets/Datasheets/AP33772S.pdf)
- [USB Power Delivery Specification](https://www.usb.org/documents)
- [embedded-hal Documentation](https://docs.rs/embedded-hal/)
