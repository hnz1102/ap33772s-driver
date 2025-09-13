use ap33772s_driver::{AP33772S, PDVoltage, AP33772S_ADDR, MAX_PDO_COUNT};

fn main() {
    println!("AP33772S_ADDR: 0x{:02X}", AP33772S_ADDR);
    println!("MAX_PDO_COUNT: {}", MAX_PDO_COUNT);
    
    let driver = AP33772S::new();
    let voltage = PDVoltage::V12;
    
    println!("Test successful! Created driver and voltage variant.");
}
