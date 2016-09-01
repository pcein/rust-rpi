/*
 * pigpio.rs is a Rust port of the excellent pigpio.py 
 * library http://abyz.co.uk/rpi/pigpio/python.html 
 *
 * A program using pigpio.rs runs on a client machine
 * and communicates with a Raspberry Pi server which runs
 * the program "pigpiod". The communication is through TCP
 * sockets.
 *
 * The low-level code which performs gpio manipulation on the
 * Rpi is present in "pigpiod". This library (pigpio.rs) simply 
 * sends commands  to "pigpiod" over a TCP socket requesting it 
 * to perform  operations like: set a pin HIGH, read the logic 
 * level on a pin etc.
 * 
 * pigpio.rs does *not* implement all the features available in
 * pigpio.py. The features currently supported are: digital I/O,
 * callbacks on I/O pin status change, PWM and SPI.
 */

extern crate byteorder;

use pigpio::byteorder::{LittleEndian, WriteBytesExt, ReadBytesExt};
use std::env;
use std::net::TcpStream;
use std::io::{Read, Write, Cursor};
use std::time::Duration;
use std::thread;
use std::sync::{Arc,Mutex};

pub type CallbackFn = fn(u32, u32, u32);

fn to_bytevector(a: u32, b: u32, 
                c: u32, d: u32) -> Vec<u8> {
    let mut v = vec![];

    v.write_u32::<LittleEndian>(a).unwrap();
    v.write_u32::<LittleEndian>(b).unwrap();
    v.write_u32::<LittleEndian>(c).unwrap();
    v.write_u32::<LittleEndian>(d).unwrap();

    v
}

fn to_bytevector_ext(a: u32, b: u32,
                    c: u32, d: u32, 
                    ext: Vec<Vec<u8>>) -> Vec<u8> {

    let mut v = to_bytevector(a, b, c, d);
        
    for item in &ext {
        v.extend(item);
    }

    v
}

fn unpack_data_in_callback_thread(a: &[u8]) -> 
            (u16, u16, u32, u32) {

    let mut rdr = Cursor::new(a);
    
    (rdr.read_u16::<LittleEndian>().unwrap(),
     rdr.read_u16::<LittleEndian>().unwrap(),
     rdr.read_u32::<LittleEndian>().unwrap(),
     rdr.read_u32::<LittleEndian>().unwrap(),)
}

fn to_i32(r: u32) -> i32 {
    if (r as i32) < 0 {
        panic!("Command not executed successfully");
    }
    r as i32
}

fn to_u32(mut a: &[u8]) -> u32 {
    let r = a.read_u32::<LittleEndian>().
            expect("to_i32: error in packing");
    r
}

pub struct Pi {
    control_stream: Arc<Mutex<TcpStream>>,
    _notify: Arc<_callback_thread>,
}

#[derive(Clone)]
#[derive(PartialEq)]
#[allow(non_camel_case_types)]
struct _callback_ADT {
    gpio: u32,
    edge: u32,
    bit: u32,
    func: CallbackFn,
}

#[allow(non_camel_case_types)]
struct _callback_thread {
    callbacks: Mutex<Vec<_callback_ADT>>,
    monitor: Mutex<u32>,
    go: Mutex<bool>,
    control_stream: Arc<Mutex<TcpStream>>,
    notify_stream: Mutex<TcpStream>,
    handle: u32,
}

#[allow(non_camel_case_types)]
pub struct _callback{
    _notify: Arc<_callback_thread>,
    //count: u32,          
    _reset: bool,
    callb: _callback_ADT,
}

//To Fix: Not all consts need to be pub 

pub const INPUT: u32 = 0;
pub const OUTPUT: u32 = 1;

pub const PUD_OFF: u32 = 0;
pub const PUD_DOWN: u32 = 1;
pub const PUD_UP: u32 = 2;

pub const _PI_CMD_MODES: u32 = 0;
pub const _PI_CMD_MODEG: u32 = 1;
pub const _PI_CMD_PUD: u32 = 2;
pub const _PI_CMD_READ: u32 = 3;
pub const _PI_CMD_WRITE: u32 = 4;
pub const _PI_CMD_PWM: u32 = 5;
pub const _PI_CMD_SERVO: u32 = 8;

pub const _PI_CMD_NB: u32 = 19;
pub const _PI_CMD_NC: u32 = 21;

pub const _PI_CMD_I2CO: u32 = 54;
pub const _PI_CMD_I2CC: u32 = 55;
pub const _PI_CMD_I2CRD: u32 = 56;
pub const _PI_CMD_I2CWD: u32 = 57;
pub const _PI_CMD_I2CWQ: u32 = 58;
pub const _PI_CMD_I2CRS: u32 = 59;
pub const _PI_CMD_I2CWS: u32 = 60;
pub const _PI_CMD_I2CRB: u32 = 61;
pub const _PI_CMD_I2CWB: u32 = 62;
pub const _PI_CMD_I2CRW: u32 = 63;
pub const _PI_CMD_I2CWW: u32 = 64;
pub const _PI_CMD_I2CRK: u32 = 65;
pub const _PI_CMD_I2CWK: u32 = 66;
pub const _PI_CMD_I2CRI: u32 = 67;
pub const _PI_CMD_I2CWI: u32 = 68;
pub const _PI_CMD_I2CPC: u32 = 69;
pub const _PI_CMD_I2CPK: u32 = 70;

pub const _PI_CMD_SPIO: u32 = 71;
pub const _PI_CMD_SPIC: u32 = 72;
pub const _PI_CMD_SPIR: u32 = 73;
pub const _PI_CMD_SPIW: u32 = 74;
pub const _PI_CMD_SPIX: u32 = 75;

pub const _PI_CMD_GDC: u32 = 83;
pub const _PI_CMD_GPW: u32 = 84;

pub const _PI_CMD_NOIB: u32 = 99;
pub const _PI_CMD_BR1: u32 = 10;
pub const NTFY_FLAGS_WDOG: u32 = (1 << 5);
pub const NTFY_FLAGS_GPIO: u32 = 31;
pub const TIMEOUT: u32 = 2;

pub const RISING_EDGE: u32 = 0;
pub const FALLING_EDGE: u32 = 1;
pub const EITHER_EDGE: u32 = 2;

impl _callback {
    fn new(notify: Arc<_callback_thread>, 
           user_gpio: u32, edge: u32, 
           func: CallbackFn) -> _callback {

        let callb = _callback_ADT
                    ::new(user_gpio, edge, func);
        
        notify.append(callb.clone());

        _callback {
            _notify: notify, 
            //count: 0,
            _reset: false, 
            callb: callb,
        } 
    }

    pub fn remove(&self) {
        self._notify.remove(&self.callb);
    }
}

impl _callback_ADT {
    fn new(gpio: u32, edge: u32, 
           func: CallbackFn) -> _callback_ADT {

        _callback_ADT {
            gpio: gpio, 
            edge: edge, 
            func: func, 
            bit: (1 << gpio),
        }
    }
}

impl _callback_thread {
    fn new(control_stream: Arc<Mutex<TcpStream>>,
           host: String, port: String) -> Arc<_callback_thread> {
        let serv_addr = format!("{}:{}", host, port);
        let notify_stream = TcpStream::connect(&*serv_addr).unwrap();
        let notify_stream = Mutex::new(notify_stream);
        let handle = _pigpio_command(& notify_stream, 
                                     _PI_CMD_NOIB, 0, 0);


        let r = _callback_thread {
                    callbacks: Mutex::new(vec![]),
                    monitor: Mutex::new(0),
                    go: Mutex::new(true),
                    control_stream: control_stream,
                    notify_stream: notify_stream,
                    handle: handle
                };
        
        let r = Arc::new(r);
        let r1 = r.clone();
        thread::spawn(move || notification_thread(r));
    
        r1
        
    }    
    
    fn append(&self, callb: _callback_ADT) {
        *(self.monitor.lock().unwrap()) |= callb.bit;
        _pigpio_command(& self.control_stream, _PI_CMD_NB, 
                        self.handle, *(self.monitor.lock().unwrap()));
        self.callbacks.lock().unwrap().push(callb);
    }

    /*
    fn stop(&self) {
        let mut g = self.go.lock().unwrap();
        if *g {
            *g = false;
        }    
        let _ = self.notify_stream.lock().unwrap()
                .write(&to_bytevector(_PI_CMD_NC, self.handle, 0, 0));
    }
    */

    fn remove(&self, callb: &_callback_ADT) {  
        let mut callbacks = self.callbacks.lock().unwrap();
        let mut new_monitor: u32 = 0;

        let r = callbacks.iter()
                    .position(|x| *x == *callb);
        if let Some(index) = r {
            callbacks.remove(index);
                      
            for c in callbacks.iter() {
                new_monitor |= c.bit;
            }
            let mut monitor = self.monitor.lock().unwrap();
            if new_monitor != *monitor {
                *monitor = new_monitor;
                _pigpio_command(&self.control_stream,
                                _PI_CMD_NB, self.handle,
                                *monitor);
            }
        }
    }
}

fn notification_thread(t: Arc<_callback_thread>) {
    let mut last_level:u32;
    let mut changed:u32;
    let mut new_level:u32;
    let mut gpio:u32;
    const MSG_SIZE:usize = 12;
    let mut recvd_data: [u8; MSG_SIZE] = [0;MSG_SIZE];


    last_level = _pigpio_command(& t.control_stream, 
                    _PI_CMD_BR1, 0, 0);     
    while *(t.go.lock().unwrap()) {

        let r = t.notify_stream.lock().unwrap()
                .read(&mut recvd_data[..]);
        let bytes_recvd = r.expect("fn run: socket read failed");
        if bytes_recvd != MSG_SIZE {
            panic!("fn run: too few bytes received");
        }

        if *(t.go.lock().unwrap()) {

            let (_, flags, tick, level) = 
            unpack_data_in_callback_thread(&recvd_data[..]);

            if flags == 0 {
                changed = level ^ last_level;
                last_level = level;
                let callbacks = t.callbacks.lock().unwrap();
                for cb in &(*callbacks) {
                    if (cb.bit  & changed) != 0{
                        new_level = 0;
                        if (cb.bit & level) != 0 {
                            new_level = 1;
                        }
                        if (cb.edge ^ new_level) != 0 {
                            (cb.func)(cb.gpio, new_level, tick);
                        }
                    }
                }

            } else {
                if (flags as u32 & NTFY_FLAGS_WDOG) != 0 {
                    gpio = flags as u32 & NTFY_FLAGS_GPIO;
                    let callbacks = t.callbacks.lock().unwrap();
                    for cb in &(*callbacks) {
                        if cb.gpio == gpio {
                            (cb.func)(cb.gpio, TIMEOUT, tick);
                        }
                    }
                }
            }
           
        }

    }
}

fn _pigpio_command_send(stream: & Mutex<TcpStream>, 
                        data: &[u8]) -> u32 {
    let mut stream = stream.lock().unwrap();
    let r = stream.write(data);
    let bytes_written = r.expect("Write failed");
    if bytes_written != 16 {
        panic!("Write failed: did not write 16 bytes");
    }
    let mut recvd_data: [u8; 16] = [0; 16];
    let r = stream.read(&mut recvd_data[..]);
    let bytes_recvd = r.expect("Read failed");
    if bytes_recvd != 16 {
        panic!("Read failed: did not read 16 bytes");
    }
    to_u32(&recvd_data[12..16])
}

fn _pigpio_command(stream: & Mutex<TcpStream>, 
                   cmd: u32, p1: u32, p2: u32) -> u32 {
    let data = &to_bytevector(cmd, p1, p2, 0);
    _pigpio_command_send(stream, data)
}

fn _pigpio_command_ext(stream: &Mutex<TcpStream>,
                       cmd: u32, p1: u32, p2: u32,
                       p3: u32, extents: Vec<Vec<u8>>) -> u32 {
    let data = &to_bytevector_ext(cmd, p1, p2, p3, extents);
    _pigpio_command_send(stream, data)
}

impl Pi {
    pub fn new() -> Pi {
        let host = env::var("PIGPIO_ADDR")
                   .unwrap_or("localhost".to_string());
        let port = env::var("PIGPIO_PORT")
                   .unwrap_or("8888".to_string());
        let serv_addr = format!("{}:{}", host, port);
        let control_stream = TcpStream::connect(&*serv_addr).unwrap();
        let control_stream = Arc::new(Mutex::new(control_stream));
        let control_stream_cloned = control_stream.clone();
        Pi { 
            control_stream: control_stream, 
            _notify: _callback_thread
                    ::new(control_stream_cloned,
                          host, port),
        }

    }

    fn _rxbuf(&self, count: u32) -> Vec<u8> {
    
        let mut v:Vec<u8> = vec![];
        v.resize(count as usize, 0);

        let mut stream = self.control_stream.lock().unwrap();
        stream.read_exact(&mut v[..]).unwrap();

        v
    }

    pub fn set_mode(&self, gpio: u32, mode: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream, 
                               _PI_CMD_MODES, gpio, mode))
    }

    pub fn get_mode(&self, gpio: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream, 
                               _PI_CMD_MODEG, gpio, 0))
    }

    pub fn set_pull_up_down(&self, gpio: u32, pud: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream, 
                               _PI_CMD_PUD, gpio, pud))
    }

    pub fn read(&self, gpio: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream, 
                               _PI_CMD_READ, gpio, 0))
    }

    pub fn write(&self, gpio: u32, level: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream, 
                               _PI_CMD_WRITE, gpio, level))
    }

    pub fn set_pwm_dutycycle(&self, user_gpio: u32, 
                         dutycycle: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream,
                               _PI_CMD_PWM, user_gpio, 
                               dutycycle))
    }

    pub fn get_pwm_dutycycle(&self, user_gpio: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream,
                               _PI_CMD_GDC, user_gpio, 0))
    }

    pub fn set_servo_pulsewidth(&self, user_gpio: u32, 
                            pulsewidth: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream,
                               _PI_CMD_SERVO, user_gpio,
                               pulsewidth))
    }

    pub fn get_servo_pulsewidth(&self, 
                            user_gpio: u32) -> i32 {
        to_i32(_pigpio_command(&self.control_stream,
                               _PI_CMD_GPW, user_gpio, 0))

    }

    pub fn spi_open(&self, spi_channel: u32, baud: u32,
                spi_flags: u32) -> i32 {
        
        let mut v:Vec<u8> = vec![];        
        v.write_u32::<LittleEndian>(spi_flags).unwrap();
        
        to_i32(_pigpio_command_ext(&self.control_stream, _PI_CMD_SPIO,
                                   spi_channel, baud, 4, 
                                   vec![v]))   
    }

    pub fn spi_close(&self, handle: i32) -> i32 {
        
        to_i32(_pigpio_command(&self.control_stream, _PI_CMD_SPIC,
                               handle as u32, 0))
    }

    pub fn spi_read(&self, handle: u32, 
                count: u32) -> (u32, Vec<u8>) {

        let mut stream = self.control_stream.lock().unwrap();
        let data = &to_bytevector(_PI_CMD_SPIR,
                                  handle, count,
                                  0);
        let r = stream.write(data);
        let bytes_written = r.expect("Write failed");
        if bytes_written != 16 {
            panic!("Write failed: did not write 16 bytes");
        }
        let mut recvd_data: [u8; 16] = [0; 16];
        let r = stream.read(&mut recvd_data[..]);
        let bytes_recvd = r.expect("Read failed");
        if bytes_recvd != 16 {
            panic!("Read failed: did not read 16 bytes");
        }
                    
        let spi_read_data_len = to_u32(&recvd_data[12..16]);
        let spi_data:Vec<u8>;

        if to_i32(spi_read_data_len) > 0 {
            spi_data = self._rxbuf(spi_read_data_len);
        } else {
            spi_data = vec![];
        }
        (spi_read_data_len, spi_data)
    }
        
    pub fn spi_write(&self, handle: u32, data: Vec<u8>) -> i32 {

        to_i32(_pigpio_command_ext(
                &self.control_stream,
                _PI_CMD_SPIW,
                handle, 0, data.len() as u32,
                vec![data]
              ))

    }

    pub fn callback(&self, user_gpio: u32, edge: u32, 
                func: CallbackFn) -> _callback {
        _callback::new(self._notify.clone(), 
                       user_gpio, edge, func)
    } 

}

pub fn sleep_ms(ms: u64) {
    thread::sleep(Duration::from_millis(ms));
}

