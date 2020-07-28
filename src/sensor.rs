//! # MH-Z19B Sensor Library
//!
//! Functions and constants to interface with the MH-Z19B CO2 sensor.

pub fn calculate_checksum(packet: [u8; 9]) -> u8 {
    let mut checksum = 0u8;
    for c in &packet[1..9] {
        checksum = checksum.wrapping_add(*c);
    }
    checksum = 0xff - checksum;
    checksum.wrapping_add(0x01)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_calculate_checksum() {
        assert_eq!(calculate_checksum([0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff]), 0x79);
    }
}
