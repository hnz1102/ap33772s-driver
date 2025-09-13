#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

//! # AP33772S USB-PD Sink Controller Driver
//!
//! This crate provides a platform-agnostic driver for the AP33772S USB Power Delivery (USB-PD)
//! Sink Controller. The driver is built on top of the [`embedded-hal`] traits to ensure
//! compatibility with various microcontroller platforms.
//!
//! ## Features
//!
//! - Platform-agnostic implementation using `embedded-hal` traits
//! - Support for both Standard Power Range (SPR) and Extended Power Range (EPR) modes
//! - Power Data Object (PDO) enumeration and negotiation
//! - Voltage and current monitoring
//! - Protection features (UVP, OVP, OCP, OTP)
//! - Programmable Power Supply (PPS) and Adjustable Voltage Supply (AVS) support

use embedded_hal::i2c::I2c;
use heapless::Vec;

/// AP33772S I2C slave address
pub const AP33772S_ADDR: u8 = 0x52;

/// Maximum number of PDOs supported by AP33772S
pub const MAX_PDO_COUNT: usize = 13;

// Register definitions
pub const REG_STATUS: u8 = 0x01;
pub const REG_MASK: u8 = 0x02;
pub const REG_OPMODE: u8 = 0x03;
pub const REG_CONFIG: u8 = 0x04;
pub const REG_SYSTEM: u8 = 0x06;
pub const REG_VOLTAGE: u8 = 0x11;
pub const REG_CURRENT: u8 = 0x12;
pub const REG_TEMP: u8 = 0x13;
pub const REG_VREQ: u8 = 0x14;
pub const REG_IREQ: u8 = 0x15;
pub const REG_SRCPDO: u8 = 0x20;
pub const REG_PD_REQMSG: u8 = 0x31;
pub const REG_PD_CMDMSG: u8 = 0x32;
pub const REG_PD_MSGRLT: u8 = 0x33;

// Status register bits
pub const STATUS_STARTED: u8 = 0x01;
pub const STATUS_READY: u8 = 0x02;
pub const STATUS_NEWPDO: u8 = 0x04;
pub const STATUS_UVP: u8 = 0x08;
pub const STATUS_OVP: u8 = 0x10;
pub const STATUS_OCP: u8 = 0x20;
pub const STATUS_OTP: u8 = 0x40;

// Config register bits
pub const CONFIG_UVP_EN: u8 = 0x08;
pub const CONFIG_OVP_EN: u8 = 0x10;
pub const CONFIG_OCP_EN: u8 = 0x20;
pub const CONFIG_OTP_EN: u8 = 0x40;
pub const CONFIG_DR_EN: u8 = 0x80;

// System register bits
pub const SYSTEM_VOUTCTL_AUTO: u8 = 0x00;
pub const SYSTEM_VOUTCTL_OFF: u8 = 0x01;
pub const SYSTEM_VOUTCTL_ON: u8 = 0x02;
pub const CC_STATUS_MASK: u8 = 0x30;

// Command message bits
pub const CMDMSG_HRST: u8 = 0x01;

// Message result bits
pub const MSGRLT_BUSY: u8 = 0x00;
pub const MSGRLT_SUCCESS: u8 = 0x01;
pub const MSGRLT_INVALID: u8 = 0x02;
pub const MSGRLT_UNSUPPORTED: u8 = 0x03;
pub const MSGRLT_FAILED: u8 = 0x04;
pub const MSGRLT_MASK: u8 = 0x0F;

// PDO parsing constants
pub const SRCPDO_DETECT: u16 = 0x8000;
pub const SRCPDO_TYPE: u16 = 0x4000;
pub const SRCPDO_CURRENT_MAX_MASK: u16 = 0x3C00;
pub const SRCPDO_CURRENT_MAX_SHIFT: u8 = 10;
pub const SRCPDO_VOLTAGE_MAX_MASK: u16 = 0x00FF;

// Request message bits
pub const REQMSG_VOLTAGE_SEL_MASK: u16 = 0x00FF;
pub const REQMSG_MAX_CURRENT: u8 = 0x0F;
pub const REQMSG_MAX_VOLTAGE: u8 = 0xFF;

// Current values in mA
pub const SRCPDO_CURRENT_VALUES: [u16; 16] = [
    1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750,
    3000, 3250, 3500, 3750, 4000, 4250, 4500, 5000
];

/// Error types for AP33772S operations
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Error<E> {
    /// I2C communication error
    I2c(E),
    /// PD negotiation timeout
    Timeout,
    /// PD negotiation failed
    NegotiationFailed,
    /// Invalid parameter
    InvalidParameter,
    /// Device not initialized
    NotInitialized,
    /// Protection fault occurred
    ProtectionFault,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::I2c(error)
    }
}

/// PD Voltage options
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PDVoltage {
    /// 5V output
    V5 = 0,
    /// 9V output
    V9 = 1,
    /// 12V output
    V12 = 2,
    /// 15V output
    V15 = 3,
    /// 20V output
    V20 = 4,
    /// Custom voltage
    Custom = 0xFF,
}

/// Fault types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PDFault {
    /// No fault
    None,
    /// Under Voltage Protection triggered
    UnderVoltage,
    /// Over Voltage Protection triggered
    OverVoltage,
    /// Over Current Protection triggered
    OverCurrent,
    /// Over Temperature Protection triggered
    OverTemperature,
    /// Unknown fault
    Unknown,
}

/// Source Power Data Object information
#[derive(Debug, Clone, Copy)]
pub struct PDOInfo {
    /// Voltage in millivolts
    pub voltage_mv: u16,
    /// Current in milliamps
    pub current_ma: u16,
    /// Maximum power in milliwatts
    pub max_power_mw: u32,
    /// True if fixed voltage PDO
    pub is_fixed: bool,
    /// PDO index (1-13)
    pub pdo_index: u8,
}

/// Current status of the PD controller
#[derive(Debug, Clone, Copy)]
pub struct PDStatus {
    /// Raw status register value
    pub status: u8,
    /// Raw operation mode register value
    pub op_mode: u8,
    /// Current output voltage in millivolts
    pub voltage_mv: u16,
    /// Current output current in milliamps
    pub current_ma: u16,
    /// Temperature in degrees Celsius
    pub temperature: i8,
    /// Requested voltage in millivolts
    pub requested_voltage_mv: u16,
    /// Requested current in milliamps
    pub requested_current_ma: u16,
    /// True if USB-PD source is attached and ready
    pub is_attached: bool,
    /// True if controller is busy processing a request
    pub is_busy: bool,
    /// True if any protection fault is active
    pub has_fault: bool,
    /// Type of fault if any
    pub fault_type: PDFault,
    /// CC line status
    pub cc_status: u8,
    /// Power delivery limit in watts
    pub pdp_limit_w: u8,
}

impl Default for PDStatus {
    fn default() -> Self {
        PDStatus {
            status: 0,
            op_mode: 0,
            voltage_mv: 0,
            current_ma: 0,
            temperature: 0,
            requested_voltage_mv: 0,
            requested_current_ma: 0,
            is_attached: false,
            is_busy: false,
            has_fault: false,
            fault_type: PDFault::None,
            cc_status: 0,
            pdp_limit_w: 0,
        }
    }
}

/// AP33772S USB-PD Sink Controller Driver
pub struct AP33772S {
    initialized: bool,
    pdo_list: Vec<PDOInfo, MAX_PDO_COUNT>,
}

impl AP33772S {
    /// Create a new AP33772S driver instance
    pub fn new() -> Self {
        AP33772S {
            initialized: false,
            pdo_list: Vec::new(),
        }
    }

    /// Initialize the AP33772S controller
    pub fn init<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        // Check if device is available by reading status register
        let _status = self.read_register(i2c, REG_STATUS)?;
        
        // Read all PDOs
        self.read_full_srcpdo(i2c)?;
        self.initialized = true;
        Ok(())
    }

    /// Request specific voltage from the USB PD source
    pub fn request_voltage<I2C, E>(&self, i2c: &mut I2C, voltage: PDVoltage) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        let voltage_mv: u16 = match voltage {
            PDVoltage::V5 => 5000,
            PDVoltage::V9 => 9000,
            PDVoltage::V12 => 12000,
            PDVoltage::V15 => 15000,
            PDVoltage::V20 => 20000,
            PDVoltage::Custom => 0,
        };

        let mut pdo_index = 0;
        for pdo in &self.pdo_list {
            if pdo.voltage_mv == voltage_mv {
                pdo_index = pdo.pdo_index;
                break;
            }
        }

        if pdo_index == 0 {
            return Err(Error::InvalidParameter);
        }

        // Build request message
        let mut request_msg = (pdo_index as u16) << 12;
        request_msg |= (REQMSG_MAX_VOLTAGE as u16) | ((REQMSG_MAX_CURRENT as u16) << 8);
        
        self.write_word(i2c, REG_PD_REQMSG, request_msg)?;
        self.wait_for_negotiation(i2c)
    }

    /// Read the current status of the PD controller
    pub fn get_status<I2C, E>(&self, i2c: &mut I2C) -> Result<PDStatus, Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        if !self.initialized {
            return Err(Error::NotInitialized);
        }

        let mut status = PDStatus::default();
        
        status.status = self.read_register(i2c, REG_STATUS)?;
        status.op_mode = self.read_register(i2c, REG_OPMODE)?;
        
        let system_reg = self.read_register(i2c, REG_SYSTEM)?;
        status.cc_status = system_reg & CC_STATUS_MASK;
        
        status.is_attached = (status.status & STATUS_READY) != 0;
        status.is_busy = (status.status & STATUS_NEWPDO) != 0;
        
        let has_uvp = (status.status & STATUS_UVP) != 0;
        let has_ovp = (status.status & STATUS_OVP) != 0;
        let has_ocp = (status.status & STATUS_OCP) != 0;
        let has_otp = (status.status & STATUS_OTP) != 0;
        
        status.has_fault = has_uvp || has_ovp || has_ocp || has_otp;
        
        if status.has_fault {
            if has_uvp {
                status.fault_type = PDFault::UnderVoltage;
            } else if has_ovp {
                status.fault_type = PDFault::OverVoltage;
            } else if has_ocp {
                status.fault_type = PDFault::OverCurrent;
            } else if has_otp {
                status.fault_type = PDFault::OverTemperature;
            } else {
                status.fault_type = PDFault::Unknown;
            }
        } else {
            status.fault_type = PDFault::None;
        }
        
        let voltage_word = self.read_word(i2c, REG_VOLTAGE)?;
        status.voltage_mv = voltage_word * 80;
        
        let current_byte = self.read_register(i2c, REG_CURRENT)?;
        status.current_ma = (current_byte as u16) * 24;
        
        status.temperature = self.read_register(i2c, REG_TEMP)? as i8;
        
        let req_voltage = self.read_word(i2c, REG_VREQ)?;
        status.requested_voltage_mv = req_voltage * 50;
        
        let req_current = self.read_word(i2c, REG_IREQ)?;
        status.requested_current_ma = req_current * 10;
        
        let pdp = ((status.requested_voltage_mv as u32) * (status.requested_current_ma as u32)) / 1_000_000;
        status.pdp_limit_w = pdp as u8;
        
        Ok(status)
    }

    /// Get available PDO information
    pub fn get_pdo_list(&self) -> &[PDOInfo] {
        &self.pdo_list
    }

    /// Get the maximum voltage available from the source
    pub fn get_max_voltage(&self) -> u16 {
        self.pdo_list.iter()
            .map(|pdo| pdo.voltage_mv)
            .max()
            .unwrap_or(0)
    }

    /// Perform a hard reset of the PD connection
    pub fn hard_reset<I2C, E>(&self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        self.write_register(i2c, REG_PD_CMDMSG, CMDMSG_HRST)?;
        Ok(())
    }

    /// Set VOUT to auto control
    pub fn set_vout_auto_control<I2C, E>(&self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        self.write_register(i2c, REG_SYSTEM, SYSTEM_VOUTCTL_AUTO)
    }
    
    /// Force VOUT OFF
    pub fn force_vout_off<I2C, E>(&self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        self.write_register(i2c, REG_SYSTEM, SYSTEM_VOUTCTL_OFF)
    }
    
    /// Force VOUT ON
    pub fn force_vout_on<I2C, E>(&self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        self.write_register(i2c, REG_SYSTEM, SYSTEM_VOUTCTL_ON)
    }

    /// Configure protection features
    pub fn configure_protections<I2C, E>(
        &self,
        i2c: &mut I2C,
        enable_uvp: bool,
        enable_ovp: bool,
        enable_ocp: bool,
        enable_otp: bool,
        enable_dr: bool,
    ) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        let mut config = self.read_register(i2c, REG_CONFIG)?;
        
        config &= !(CONFIG_UVP_EN | CONFIG_OVP_EN | CONFIG_OCP_EN | CONFIG_OTP_EN | CONFIG_DR_EN);
        
        if enable_uvp { config |= CONFIG_UVP_EN; }
        if enable_ovp { config |= CONFIG_OVP_EN; }
        if enable_ocp { config |= CONFIG_OCP_EN; }
        if enable_otp { config |= CONFIG_OTP_EN; }
        if enable_dr { config |= CONFIG_DR_EN; }
        
        self.write_register(i2c, REG_CONFIG, config)
    }

    // Private methods

    /// Wait for PD negotiation to complete
    fn wait_for_negotiation<I2C, E>(&self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        for _ in 0..20 {
            let result = self.read_register(i2c, REG_PD_MSGRLT)?;
            let result_code = result & MSGRLT_MASK;
            
            let status = self.read_register(i2c, REG_STATUS)?;
            
            let has_fault = (status & (STATUS_UVP | STATUS_OVP | STATUS_OCP | STATUS_OTP)) != 0;
            if has_fault {
                return Err(Error::ProtectionFault);
            }
            
            if result_code == MSGRLT_SUCCESS {
                return Ok(());
            } else if result_code != 0 {
                return Err(Error::NegotiationFailed);
            }
        }
        
        Err(Error::Timeout)
    }

    /// Read the full SRCPDO register and parse all PDOs
    fn read_full_srcpdo<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        self.pdo_list.clear();
        
        let mut buffer = [0u8; MAX_PDO_COUNT * 2];
        i2c.write(AP33772S_ADDR, &[REG_SRCPDO])?;
        i2c.read(AP33772S_ADDR, &mut buffer)?;
        
        for i in 0..MAX_PDO_COUNT {
            let offset = i * 2;
            let word = ((buffer[offset + 1] as u16) << 8) | buffer[offset] as u16;
            
            let is_epr = i >= 7;
            let pdo_index = if is_epr { (i - 7) as u8 + 8 } else { i as u8 + 1 };
            
            if let Some(pdo_info) = self.parse_pdo_word(word, is_epr, pdo_index) {
                let _ = self.pdo_list.push(pdo_info); // Ignore error if Vec is full
            }
        }
        
        Ok(())
    }

    /// Parse a PDO word
    fn parse_pdo_word(&self, pdo_word: u16, is_epr: bool, pdo_index: u8) -> Option<PDOInfo> {
        if (pdo_word & SRCPDO_DETECT) == 0 {
            return None;
        }

        let is_apdo = (pdo_word & SRCPDO_TYPE) != 0;
        let is_fixed = !is_apdo;

        let current_idx = ((pdo_word & SRCPDO_CURRENT_MAX_MASK) >> SRCPDO_CURRENT_MAX_SHIFT) as usize;
        let current_ma = if current_idx < SRCPDO_CURRENT_VALUES.len() {
            SRCPDO_CURRENT_VALUES[current_idx]
        } else {
            5000
        };

        let voltage_unit = (pdo_word & SRCPDO_VOLTAGE_MAX_MASK) as u16;
        let voltage_mv = if is_epr {
            voltage_unit * 200
        } else {
            voltage_unit * 100
        };

        let max_power_mw = (voltage_mv as u32 * current_ma as u32) / 1000;

        Some(PDOInfo {
            voltage_mv,
            current_ma,
            max_power_mw,
            is_fixed,
            pdo_index,
        })
    }

    /// Read a single register
    fn read_register<I2C, E>(&self, i2c: &mut I2C, reg: u8) -> Result<u8, Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        let mut buf = [0u8; 1];
        i2c.write(AP33772S_ADDR, &[reg])?;
        i2c.read(AP33772S_ADDR, &mut buf)?;
        Ok(buf[0])
    }

    /// Read a 16-bit word
    fn read_word<I2C, E>(&self, i2c: &mut I2C, reg: u8) -> Result<u16, Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        let mut buf = [0u8; 2];
        i2c.write(AP33772S_ADDR, &[reg])?;
        i2c.read(AP33772S_ADDR, &mut buf)?;
        Ok(((buf[1] as u16) << 8) | (buf[0] as u16))
    }

    /// Write a single register
    fn write_register<I2C, E>(&self, i2c: &mut I2C, reg: u8, value: u8) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        i2c.write(AP33772S_ADDR, &[reg, value])?;
        Ok(())
    }

    /// Write a 16-bit word
    fn write_word<I2C, E>(&self, i2c: &mut I2C, reg: u8, value: u16) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        let lsb = (value & 0xFF) as u8;
        let msb = ((value >> 8) & 0xFF) as u8;
        i2c.write(AP33772S_ADDR, &[reg, lsb, msb])?;
        Ok(())
    }
}

impl Default for AP33772S {
    fn default() -> Self {
        Self::new()
    }
}