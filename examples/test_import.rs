//! # AP33772S Driver Usage Examples
//! 
//! This example demonstrates how to use the ap33772s-driver crate
//! with different embedded platforms and scenarios.

use ap33772s_driver::{AP33772S, PDVoltage, PDOInfo, PDStatus};
use embedded_hal::i2c::I2c;
use embedded_hal::delay::DelayNs;

/// Mock I2C implementation for demonstration purposes
/// In real applications, use your platform's I2C driver
struct MockI2c {
    _simulated_pdos: Vec<(u16, u16)>, // (voltage_mv, current_ma) pairs - for future use
}

impl MockI2c {
    fn new() -> Self {
        Self {
            // Simulate a typical USB-PD source with EPR support
            _simulated_pdos: vec![
                (5000, 3000),   // 5V, 3A - Standard USB
                (9000, 3000),   // 9V, 3A - QC compatible
                (12000, 3000),  // 12V, 3A
                (15000, 3000),  // 15V, 3A
                (20000, 5000),  // 20V, 5A - 100W
                (28000, 5000),  // 28V, 5A - EPR 140W
                (36000, 3000),  // 36V, 3A - EPR 108W
            ],
        }
    }
}

#[derive(Debug)]
struct MockI2cError;

impl embedded_hal::i2c::Error for MockI2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl embedded_hal::i2c::ErrorType for MockI2c {
    type Error = MockI2cError;
}

impl I2c for MockI2c {
    fn write(&mut self, _address: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
        // Mock implementation - always succeeds
        Ok(())
    }

    fn read(&mut self, _address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        // Mock implementation - fills buffer with simulated data
        for (i, byte) in buffer.iter_mut().enumerate() {
            *byte = (i % 256) as u8;
        }
        Ok(())
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                embedded_hal::i2c::Operation::Write(bytes) => {
                    self.write(address, bytes)?;
                }
                embedded_hal::i2c::Operation::Read(buffer) => {
                    self.read(address, buffer)?;
                }
            }
        }
        Ok(())
    }
}

/// Mock delay implementation
struct MockDelay;

impl DelayNs for MockDelay {
    fn delay_ns(&mut self, _ns: u32) {
        // Mock implementation - no actual delay needed for simulation
    }
}

fn main() {
    println!("AP33772S USB-PD Driver Examples");
    println!("==================================\n");

    // Example 1: Basic Driver Creation and Constants
    example_1_basic_usage();
    
    // Example 2: PDO Information and Voltage Selection
    example_2_pdo_handling();
    
    // Example 3: Voltage Request Scenarios
    example_3_voltage_requests();
    
    // Example 4: Status Monitoring
    example_4_status_monitoring();
    
    // Example 5: Error Handling
    example_5_error_handling();
    
    println!("\nAll examples completed successfully!");
}

/// Example 1: Basic driver creation and available constants
fn example_1_basic_usage() {
    println!("[Example 1] Basic Driver Usage");
    println!("AP33772S I2C Address: 0x{:02X}", ap33772s_driver::AP33772S_ADDR);
    println!("Maximum PDO Count: {}", ap33772s_driver::MAX_PDO_COUNT);
    
    // Create a new driver instance
    let _driver = AP33772S::new();
    println!("OK: AP33772S driver instance created");
    
    // Show available voltage options
    println!("\nAvailable PDVoltage options:");
    let voltages = [
        PDVoltage::V5, PDVoltage::V9, PDVoltage::V12, PDVoltage::V15, PDVoltage::V20,
        PDVoltage::V28, PDVoltage::V36, PDVoltage::V40, PDVoltage::V48
    ];
    
    for voltage in voltages {
        println!("  {:?}", voltage);
    }
    println!();
}

/// Example 2: Working with PDO information
fn example_2_pdo_handling() {
    println!("[Example 2] PDO Information Handling");
    
    // This would typically be done after driver initialization
    // For this example, we'll simulate some PDO data
    let simulated_pdos = vec![
        PDOInfo {
            voltage_mv: 5000,
            current_ma: 3000,
            max_power_mw: 15000,
            is_fixed: true,
            pdo_index: 1,
        },
        PDOInfo {
            voltage_mv: 20000,
            current_ma: 5000,
            max_power_mw: 100000,
            is_fixed: true,
            pdo_index: 5,
        },
        PDOInfo {
            voltage_mv: 28000,
            current_ma: 5000,
            max_power_mw: 140000,
            is_fixed: true,
            pdo_index: 8,
        },
    ];
    
    println!("Example PDO Information:");
    for pdo in &simulated_pdos {
        println!("  PDO {}: {}V, {}A, {}W, {}",
            pdo.pdo_index,
            pdo.voltage_mv as f32 / 1000.0,
            pdo.current_ma as f32 / 1000.0,
            pdo.max_power_mw as f32 / 1000.0,
            if pdo.is_fixed { "Fixed" } else { "Variable" }
        );
    }
    
    // Find maximum available voltage
    let max_voltage = simulated_pdos.iter()
        .map(|pdo| pdo.voltage_mv)
        .max()
        .unwrap_or(0);
    println!("Maximum available voltage: {}V", max_voltage as f32 / 1000.0);
    println!();
}

/// Example 3: Different voltage request scenarios
fn example_3_voltage_requests() {
    println!("[Example 3] Voltage Request Scenarios");
    
    let _mock_i2c = MockI2c::new();
    let _mock_delay = MockDelay;
    let _driver = AP33772S::new();
    
    // Simulate driver initialization
    // In real usage: driver.init(&mut i2c)?;
    println!("OK: Driver initialized (simulated)");
    
    // Example voltage requests
    let voltage_scenarios = [
        (PDVoltage::V5, "Standard 5V for basic circuits"),
        (PDVoltage::V12, "12V for Arduino/development boards"),
        (PDVoltage::V20, "20V for laptop charging"),
        (PDVoltage::V28, "28V EPR for high-power applications"),
        (PDVoltage::V48, "48V EPR for industrial applications"),
    ];
    
    for (voltage, description) in voltage_scenarios {
        println!("Requesting {:?} - {}", voltage, description);
        
        // In real usage, this would be:
        // match driver.request_voltage(&mut i2c, &mut delay, voltage) {
        //     Ok(()) => println!("  OK: Voltage request successful"),
        //     Err(e) => println!("  ERROR: Voltage request failed: {:?}", e),
        // }
        
        // Simulated success for demo
        println!("  OK: Voltage request successful (simulated)");
    }
    
    // Custom voltage request example
    println!("\nCustom voltage request example:");
    println!("Requesting 15V (15000mV) with 2A current limit");
    // In real usage:
    // match driver.request_custom_voltage(&mut i2c, &mut delay, 15000, 2000) {
    //     Ok(()) => println!("  OK: Custom voltage request successful"),
    //     Err(e) => println!("  ERROR: Custom voltage request failed: {:?}", e),
    // }
    println!("  OK: Custom voltage request successful (simulated)");
    println!();
}

/// Example 4: Status monitoring
fn example_4_status_monitoring() {
    println!("[Example 4] Status Monitoring");
    
    // Simulate a typical status reading
    let simulated_status = PDStatus {
        status: 0x02, // STATUS_READY
        op_mode: 0x00,
        voltage_mv: 20000,
        current_ma: 3500,
        temperature: 45,
        requested_voltage_mv: 20000,
        requested_current_ma: 5000,
        is_attached: true,
        is_busy: false,
        has_fault: false,
        fault_type: ap33772s_driver::PDFault::None,
        cc_status: 0x20,
        pdp_limit_w: 100,
    };
    
    println!("Current Status:");
    println!("  Attached: {}", simulated_status.is_attached);
    println!("  Output Voltage: {}V", simulated_status.voltage_mv as f32 / 1000.0);
    println!("  Output Current: {}A", simulated_status.current_ma as f32 / 1000.0);
    println!("  Output Power: {}W", 
        (simulated_status.voltage_mv as f32 * simulated_status.current_ma as f32) / 1_000_000.0);
    println!("  Temperature: {}°C", simulated_status.temperature);
    println!("  Power Limit: {}W", simulated_status.pdp_limit_w);
    println!("  Has Fault: {}", simulated_status.has_fault);
    println!("  Fault Type: {:?}", simulated_status.fault_type);
    println!();
}

/// Example 5: Error handling patterns
fn example_5_error_handling() {
    println!("[Example 5] Error Handling");
    
    // Different error scenarios that might occur
    println!("Common error scenarios:");
    
    println!("  1. Device not initialized:");
    println!("     Error::NotInitialized - Call init() first");
    
    println!("  2. I2C communication failure:");
    println!("     Error::I2c(e) - Check connections and I2C configuration");
    
    println!("  3. Voltage request timeout:");
    println!("     Error::Timeout - PD negotiation took too long");
    
    println!("  4. Negotiation failed:");
    println!("     Error::NegotiationFailed - Requested voltage not supported");
    
    println!("  5. Invalid parameter:");
    println!("     Error::InvalidParameter - Check voltage/current values");
    
    println!("  6. Protection fault:");
    println!("     Error::ProtectionFault - UVP/OVP/OCP/OTP triggered");
    
    // Example error handling pattern
    println!("\nRecommended error handling pattern:");
    println!("```rust");
    println!("match driver.request_voltage(&mut i2c, &mut delay, PDVoltage::V20) {{");
    println!("    Ok(()) => {{");
    println!("        log::info!(\"Voltage request successful\");");
    println!("        // Continue with normal operation");
    println!("    }},");
    println!("    Err(Error::Timeout) => {{");
    println!("        log::warn!(\"Voltage request timed out, retrying...\");");
    println!("        // Implement retry logic");
    println!("    }},");
    println!("    Err(Error::ProtectionFault) => {{");
    println!("        log::error!(\"Protection fault detected, stopping operation\");");
    println!("        // Emergency shutdown");
    println!("    }},");
    println!("    Err(e) => {{");
    println!("        log::error!(\"Voltage request failed: {{:?}}\", e);");
    println!("        // Handle other errors");
    println!("    }}");
    println!("}}");
    println!("```");
    println!();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_driver_creation() {
        let driver = AP33772S::new();
        // Driver should be created successfully
        assert_eq!(std::mem::size_of_val(&driver), std::mem::size_of::<AP33772S>());
    }

    #[test]
    fn test_voltage_variants() {
        // Test that all voltage variants can be created
        let voltages = [
            PDVoltage::V5, PDVoltage::V9, PDVoltage::V12, PDVoltage::V15, PDVoltage::V20,
            PDVoltage::V28, PDVoltage::V36, PDVoltage::V40, PDVoltage::V48, PDVoltage::Custom
        ];
        
        for voltage in voltages {
            // Each voltage variant should be valid
            assert_ne!(format!("{:?}", voltage), "");
        }
    }

    #[test]
    fn test_mock_i2c() {
        let mut mock_i2c = MockI2c::new();
        
        // Test write operation
        assert!(mock_i2c.write(0x52, &[0x01, 0x02]).is_ok());
        
        // Test read operation
        let mut buffer = [0u8; 4];
        assert!(mock_i2c.read(0x52, &mut buffer).is_ok());
    }
}
