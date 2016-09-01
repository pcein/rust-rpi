#![allow(dead_code)]

extern crate rustgpio;

use rustgpio::pigpio;

fn pwm_up_down() {
    let pi = pigpio::Pi::new();
    let mut pwm_val = 0;
    loop {
        while pwm_val < 256 {
            pi.set_pwm_dutycycle(18, pwm_val);
            pwm_val += 1;
            pigpio::sleep_ms(5);
        }
        pwm_val = 255;
        while pwm_val > 0 {
            pi.set_pwm_dutycycle(18, pwm_val);
            pwm_val -= 1;
            pigpio::sleep_ms(5);
        }
    }
}


fn cbf(gpio: u32, level: u32, tick: u32) {
    println!("handler called, gpio: {}, tick: {} ...", 
             gpio, tick);
}

fn test_callback() {
    let pi= pigpio::Pi::new();
    pi.set_mode(18, pigpio::INPUT);
    pi.set_pull_up_down(18, pigpio::PUD_UP);
    let _ = pi.callback(18, pigpio::FALLING_EDGE, cbf);
    
    loop {
        pi.set_pull_up_down(18, pigpio::PUD_DOWN);
        pigpio::sleep_ms(500);
        pi.set_pull_up_down(18, pigpio::PUD_UP);
        pigpio::sleep_ms(500);
    }
}

fn test_read() {
    let pi = pigpio::Pi::new();
    pi.set_mode(18, pigpio::INPUT);
    //enable internal pull-up
    pi.set_pull_up_down(18, pigpio::PUD_DOWN);
    
    loop {
        println!("{}", pi.read(18));
        pigpio::sleep_ms(100);
    }
}

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
    pwm_up_down();
}
