use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::I2C1;
use embassy_rp::i2c::InterruptHandler as I2cInterruptHandler;

bind_interrupts!(
    pub struct Irqs {
        I2C1_IRQ => I2cInterruptHandler<I2C1>;
        
    }
);