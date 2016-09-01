extern crate rustgpio;

use rustgpio::pigpio;

fn test_write() {
    let pi = pigpio::Pi::new();
    pi.set_mode(18, pigpio::OUTPUT);

    loop {
        pi.write(18, 1);
        pigpio::sleep_ms(100);
        pi.write(18, 0);
        pigpio::sleep_ms(100);
    }
}


fn main() {
    test_write();
}
