use embedded_hal::{
    i2c::blocking::{Write, WriteRead},    
    digital::OutputPin,       
    pwm::PwmPin               
};
// Color formats
pub const OV7670_COLOR_RGB: u8 = 0; //not working cuz bw
pub const OV7670_COLOR_YUV: u8 = 1;

// Sizes
pub const OV7670_SIZE_DIV2: usize = 1;   // 320x240
pub const OV7670_SIZE_DIV4: usize = 2;   // 160x120
pub const OV7670_SIZE_DIV8: usize = 3;   //  80x 60
pub const OV7670_SIZE_DIV16: usize = 4;  //  40x 30

// patterns
pub const OV7670_TEST_PATTERN_NONE: u8 = 0;
pub const OV7670_TEST_PATTERN_SHIFTING_1: u8 = 1;
pub const OV7670_TEST_PATTERN_COLOR_BAR: u8 = 2;
pub const OV7670_TEST_PATTERN_COLOR_BAR_FADE: u8 = 3;

// Night mode
pub const OV7670_NIGHT_MODE_OFF: u8 = 0;
pub const OV7670_NIGHT_MODE_2: u8 = 0b1010_0000;
pub const OV7670_NIGHT_MODE_4: u8 = 0b1100_0000;
pub const OV7670_NIGHT_MODE_8: u8 = 0b1110_0000;


pub const OV7670_ADDR: u8 = 0x21; // I2C 

// Core registers (A lot of them)
pub const REG_GAIN: u8             = 0x00;
pub const REG_BLUE: u8             = 0x01;
pub const REG_RED: u8              = 0x02;
pub const REG_VREF: u8             = 0x03;
pub const REG_COM1: u8             = 0x04;
pub const REG_BAVE: u8             = 0x05;
pub const REG_GBAVE: u8            = 0x06;
pub const REG_AECHH: u8            = 0x07;
pub const REG_RAVE: u8             = 0x08;
pub const REG_COM2: u8             = 0x09;
pub const REG_PID: u8              = 0x0A;
pub const REG_VER: u8              = 0x0B;
pub const REG_COM3: u8             = 0x0C;
pub const REG_COM4: u8             = 0x0D;
pub const REG_COM5: u8             = 0x0E;
pub const REG_COM6: u8             = 0x0F;
pub const REG_AECH: u8             = 0x10;
pub const REG_CLKRC: u8            = 0x11;
pub const REG_COM7: u8             = 0x12;
pub const REG_COM8: u8             = 0x13;
pub const REG_COM9: u8             = 0x14;
pub const REG_COM10: u8            = 0x15;
pub const REG_HSTART: u8           = 0x17;
pub const REG_HSTOP: u8            = 0x18;
pub const REG_VSTART: u8           = 0x19;
pub const REG_VSTOP: u8            = 0x1A;
pub const REG_PSHFT: u8            = 0x1B;
pub const REG_MIDH: u8             = 0x1C;
pub const REG_MIDL: u8             = 0x1D;
pub const REG_MVFP: u8             = 0x1E;
pub const REG_LAEC: u8             = 0x1F;
pub const REG_ADCCTR0: u8          = 0x20;
pub const REG_ADCCTR1: u8          = 0x21;
pub const REG_ADCCTR2: u8          = 0x22;
pub const REG_ADCCTR3: u8          = 0x23;
pub const REG_AEW: u8              = 0x24;
pub const REG_AEB: u8              = 0x25;
pub const REG_VPT: u8              = 0x26;
pub const REG_BBIAS: u8            = 0x27;
pub const REG_GBBIAS: u8           = 0x28;
pub const REG_EXHCH: u8            = 0x2A;
pub const REG_EXHCL: u8            = 0x2B;
pub const REG_RBIAS: u8            = 0x2C;
pub const REG_ADVFL: u8            = 0x2D;
pub const REG_ADVFH: u8            = 0x2E;
pub const REG_YAVE: u8             = 0x2F;
pub const REG_HSYST: u8            = 0x30;
pub const REG_HSYEN: u8            = 0x31;
pub const REG_HREF: u8             = 0x32;
pub const REG_CHLF: u8             = 0x33;
pub const REG_ARBLM: u8            = 0x34;
pub const REG_ADC: u8              = 0x37;
pub const REG_ACOM: u8             = 0x38;
pub const REG_OFON: u8             = 0x39;
pub const REG_TSLB: u8             = 0x3A;
pub const REG_COM11: u8            = 0x3B;
pub const REG_COM12: u8            = 0x3C;
pub const REG_COM13: u8            = 0x3D;
pub const REG_COM14: u8            = 0x3E;
pub const REG_EDGE: u8             = 0x3F;
pub const REG_COM15: u8            = 0x40;
pub const REG_COM16: u8            = 0x41;
pub const REG_COM17: u8            = 0x42;
pub const REG_AWBC1: u8            = 0x43;
pub const REG_AWBC2: u8            = 0x44;
pub const REG_AWBC3: u8            = 0x45;
pub const REG_AWBC4: u8            = 0x46;
pub const REG_AWBC5: u8            = 0x47;
pub const REG_AWBC6: u8            = 0x48;
pub const REG_REG4B: u8            = 0x4B;
pub const REG_DNSTH: u8            = 0x4C;
pub const REG_MTX1: u8             = 0x4F;
pub const REG_MTX2: u8             = 0x50;
pub const REG_MTX3: u8             = 0x51;
pub const REG_MTX4: u8             = 0x52;
pub const REG_MTX5: u8             = 0x53;
pub const REG_MTX6: u8             = 0x54;
pub const REG_BRIGHT: u8           = 0x55;
pub const REG_CONTRAS: u8           = 0x56;
pub const REG_CONTRAS_CENTER: u8   = 0x57;
pub const REG_MTXS: u8             = 0x58;
pub const REG_LCC1: u8             = 0x62;
pub const REG_LCC2: u8             = 0x63;
pub const REG_LCC3: u8             = 0x64;
pub const REG_LCC4: u8             = 0x65;
pub const REG_LCC5: u8             = 0x66;
pub const REG_MANU: u8             = 0x67;
pub const REG_MANV: u8             = 0x68;
pub const REG_GFIX: u8             = 0x69;
pub const REG_GGAIN: u8            = 0x6A;
pub const REG_DBLV: u8             = 0x6B;
pub const REG_AWBCTR3: u8          = 0x6C;
pub const REG_AWBCTR2: u8          = 0x6D;
pub const REG_AWBCTR1: u8          = 0x6E;
pub const REG_AWBCTR0: u8          = 0x6F;
pub const REG_SCALING_XSC: u8      = 0x70;
pub const REG_SCALING_YSC: u8      = 0x71;
pub const REG_SCALING_DCWCTR: u8   = 0x72;
pub const REG_SCALING_PCLK_DIV: u8 = 0x73;
pub const REG_REG74: u8            = 0x74;
pub const REG_REG76: u8            = 0x76;
pub const REG_SLOP: u8             = 0x7A;
pub const REG_GAM_BASE: u8         = 0x7B;
pub const REG_RGB444: u8           = 0x8C;
pub const REG_DM_LNL: u8           = 0x92;
pub const REG_LCC6: u8             = 0x94;
pub const REG_LCC7: u8             = 0x95;
pub const REG_HAECC1: u8           = 0x9F;
pub const REG_HAECC2: u8           = 0xA0;
pub const REG_SCALING_PCLK_DELAY: u8 = 0xA2;
pub const REG_BD50MAX: u8          = 0xA5;
pub const REG_HAECC3: u8           = 0xA6;
pub const REG_HAECC4: u8           = 0xA7;
pub const REG_HAECC5: u8           = 0xA8;
pub const REG_HAECC6: u8           = 0xA9;
pub const REG_HAECC7: u8           = 0xAA;
pub const REG_BD60MAX: u8          = 0xAB;
pub const REG_ABLC1: u8            = 0xB1;
pub const REG_THL_ST: u8           = 0xB3;
pub const REG_SATCTR: u8           = 0xC9;

pub const COM1_R656: u8        = 0x40;
pub const COM2_SSLEEP: u8      = 0x10;
pub const COM3_SWAP: u8        = 0x40;
pub const COM3_SCALEEN: u8     = 0x08;
pub const COM3_DCWEN: u8       = 0x04;
pub const CLK_EXT: u8          = 0x40;
pub const CLK_SCALE: u8        = 0x3F;
pub const COM7_RESET: u8       = 0x80;
pub const COM7_SIZE_MASK: u8   = 0x38;
pub const COM7_PIXEL_MASK: u8  = 0x05;
pub const COM7_SIZE_VGA: u8    = 0x00;
pub const COM7_SIZE_CIF: u8    = 0x20;
pub const COM7_SIZE_QVGA: u8   = 0x10;
pub const COM7_SIZE_QCIF: u8   = 0x08;
pub const COM7_RGB: u8         = 0x04;
pub const COM7_YUV: u8         = 0x00;
pub const COM7_BAYER: u8       = 0x01;
pub const COM7_PBAYER: u8      = 0x05;
pub const COM7_COLORBAR: u8    = 0x02;
pub const COM8_FASTAEC: u8     = 0x80;
pub const COM8_AECSTEP: u8     = 0x40;
pub const COM8_BANDING: u8     = 0x20;
pub const COM8_AGC: u8         = 0x04;
pub const COM8_AWB: u8         = 0x02;
pub const COM8_AEC: u8         = 0x01;
pub const COM10_HSYNC: u8      = 0x40;
pub const COM10_PCLK_HB: u8    = 0x20;
pub const COM10_HREF_REV: u8   = 0x08;
pub const COM10_VS_EDGE: u8    = 0x04;
pub const COM10_VS_NEG: u8     = 0x02;
pub const COM10_HS_NEG: u8     = 0x01;
pub const MVFP_MIRROR: u8      = 0x20;
pub const MVFP_VFLIP: u8       = 0x10;
pub const TSLB_NEG: u8         = 0x20;
pub const TSLB_YLAST: u8       = 0x04;
pub const TSLB_AOW: u8         = 0x01;
pub const COM11_NIGHT: u8      = 0x80;
pub const COM11_NMFR: u8       = 0x60;
pub const COM11_HZAUTO: u8     = 0x10;
pub const COM11_BAND: u8       = 0x08;
pub const COM11_EXP: u8        = 0x02;
pub const COM12_HREF: u8       = 0x80;
pub const COM13_GAMMA: u8      = 0x80;
pub const COM13_UVSAT: u8      = 0x40;
pub const COM13_UVSWAP: u8      = 0x01;
pub const COM14_DCWEN: u8      = 0x10;
pub const COM15_RMASK: u8      = 0xC0;
pub const COM15_R10F0: u8      = 0x00;
pub const COM15_R01FE: u8      = 0x80;
pub const COM15_R00FF: u8      = 0xC0;
pub const COM15_RGBMASK: u8    = 0x30;
pub const COM15_RGB: u8        = 0x00;
pub const COM15_RGB565: u8     = 0x10;
pub const COM15_RGB555: u8     = 0x30;
pub const COM16_AWBGAIN: u8    = 0x08;
pub const COM17_AECWIN: u8     = 0xC0;
pub const COM17_CBAR: u8       = 0x08;
pub const R76_BLKPCOR: u8      = 0x80;
pub const R76_WHTPCOR: u8      = 0x40;
pub const R444_ENABLE: u8      = 0x02;
pub const R444_RGBX: u8        = 0x01;

pub const OV7670_RGB: [u8; 6] = [
    REG_COM7, 0x04, // RGB -- not working cuz bw
    0x8C, 0x00,    
    0x40, 0x10,     
];

pub const OV7670_YUV: [u8; 4] = [
    REG_COM7, 0x00, // YUV
    0x40, 0x00,   
];

// Init registers
pub const OV7670_INIT: &[u8] = &[
    REG_TSLB,           TSLB_YLAST,                                        
    REG_COM10,          COM10_VS_NEG,                                 
    REG_SLOP,           0x20,                                      
    REG_GAM_BASE,       0x1C,
    REG_GAM_BASE + 1,   0x28,
    REG_GAM_BASE + 2,   0x3C,
    REG_GAM_BASE + 3,   0x55,
    REG_GAM_BASE + 4,   0x68,
    REG_GAM_BASE + 5,   0x76,
    REG_GAM_BASE + 6,   0x80,
    REG_GAM_BASE + 7,   0x88,
    REG_GAM_BASE + 8,   0x8F,
    REG_GAM_BASE + 9,   0x96,
    REG_GAM_BASE + 10,  0xA3,
    REG_GAM_BASE + 11,  0xAF,
    REG_GAM_BASE + 12,  0xC4,
    REG_GAM_BASE + 13,  0xD7,
    REG_GAM_BASE + 14,  0xE8,
    REG_COM8,           COM8_FASTAEC | COM8_AECSTEP | COM8_BANDING,    
    REG_GAIN,           0x00,                                              
    COM2_SSLEEP,        0x00,                                         
    REG_COM4,           0x00,
    REG_COM9,           0x20,                              
    REG_BD50MAX,        0x05,
    REG_BD60MAX,        0x07,
    REG_AEW,            0x75,
    REG_AEB,            0x63,
    REG_VPT,            0xA5,
    REG_HAECC1,         0x78,
    REG_HAECC2,         0x68,
    0xA1,               0x03,                         
    REG_HAECC3,         0xDF,
    REG_HAECC4,         0xDF,
    REG_HAECC5,         0xF0,
    REG_HAECC6,         0x90,
    REG_HAECC7,         0x94,
    REG_COM8,           COM8_FASTAEC | COM8_AECSTEP | COM8_BANDING | COM8_AGC | COM8_AEC, 
    REG_COM5,           0x61,
    REG_COM6,           0x4B,
    0x16,               0x02,                 
    REG_MVFP,           0x07,           
    REG_ADCCTR1,        0x02,
    REG_ADCCTR2,        0x91,
    0x29,               0x07,                           
    REG_CHLF,           0x0B,
    0x35,               0x0B,                                         
    REG_ADC,            0x1D,
    REG_ACOM,           0x71,
    REG_OFON,           0x2A,
    REG_COM12,          0x78,
    0x4D,               0x40,                                    
    0x4E,               0x20,                                            
    REG_GFIX,           0x5D,
    REG_REG74,          0x19,
    0x8D,               0x4F,                                            
    0x8E,               0x00,  
    0x8F,               0x00,                                          
    0x90,               0x00,  
    0x91,               0x00,                                                
    REG_DM_LNL,         0x00,
    0x96,               0x00,                                                
    0x9A,               0x80,                                                
    0xB0,               0x84,                                                
    REG_ABLC1,          0x0C,
    0xB2,               0x0E,                                              
    REG_THL_ST,         0x82,
    0xB8,               0x0A,                                                
    REG_AWBC1,          0x14,
    REG_AWBC2,          0xF0,
    REG_AWBC3,          0x34,
    REG_AWBC4,          0x58,
    REG_AWBC5,          0x28,
    REG_AWBC6,          0x3A,
    0x59,               0x88,                                              
    0x5A,               0x88,                                            
    0x5B,               0x44,                                               
    0x5C,               0x67,                                                
    0x5D,               0x49,                                                 
    0x5E,               0x0E,                                                
    REG_LCC3,           0x04,
    REG_LCC4,           0x20,
    REG_LCC5,           0x05,
    REG_LCC6,           0x04,
    REG_LCC7,           0x08,
    REG_AWBCTR3,        0x0A,
    REG_AWBCTR2,        0x55,
    REG_MTX1,           0x80,
    REG_MTX2,           0x80,
    REG_MTX3,           0x00,
    REG_MTX4,           0x22,
    REG_MTX5,           0x5E,
    REG_MTX6,           0x80,
    REG_AWBCTR1,        0x11,
    REG_AWBCTR0,        0x9F,                                              
    REG_BRIGHT,         0x00,
    REG_CONTRAS,        0x40,
    REG_CONTRAS_CENTER, 0x80,
];


/// Window parameters: [vstart, hstart, edge_offset, pclk_delay]
pub const WINDOW: [[u8; 4]; 5] = [
    [9, 162, 2, 2],   
    [10, 174, 0, 2],  
    [11, 186, 2, 2],  
    [12, 210, 0, 2],  
    [15, 252, 3, 2],  
];

pub trait ParallelImageCapture {
    /// lenght>= 2*width*height.
    fn capture(&mut self, buf: &mut [u8]);
    fn deinit(&mut self);
}

/// OV7670 camera driver structure.
pub struct OV7670<I2C, CAP, SHD, RST, MCLK>
where
    I2C: Write + WriteRead,
    CAP: ParallelImageCapture,
    SHD: OutputPin,
    RST: OutputPin,
    MCLK: PwmPin,
{
    i2c: I2C,
    capture: CAP,
    shutdown: Option<SHD>,
    reset: Option<RST>,
    mclk: Option<MCLK>,
    colorspace: u8,
    size: usize,
    test_pattern: u8,
    flip_x: bool,
    flip_y: bool,
    night: u8,
}

impl<I2C, CAP, SHD, RST, MCLK> OV7670<I2C, CAP, SHD, RST, MCLK>
where
    I2C: Write + WriteRead,
    CAP: ParallelImageCapture,
    SHD: OutputPin,
    RST: OutputPin,
    MCLK: PwmPin,
{
    /// new OV7670 instance.
    pub fn new(
        mut i2c: I2C,
        capture: CAP,
        mut shutdown: Option<SHD>,
        mut reset: Option<RST>,
        mut mclk: Option<MCLK>,
        mclk_freq: Option<u32>,
    ) -> Result<Self, ()> {
        // Initialize MCLK if provided
        if let (Some(ref mut pwm), Some(freq)) = (mclk.as_mut(), mclk_freq) {
            pwm.set_duty(pwm.get_max_duty() / 2);
            pwm.set_period(freq);
        }
        // shutdown pin
        if let Some(ref mut pin) = shutdown {
            pin.set_low().ok();
            cortex_m::asm::delay(1000);
            pin.set_high().ok();
        }
        // reset pin
        if let Some(ref mut pin) = reset {
            pin.set_low().ok();
            cortex_m::asm::delay(1000);
            pin.set_high().ok();
        }
        // reset registers via COM7
        let mut dev = OV7670 { i2c, capture, shutdown, reset, mclk,
            colorspace: 0, size: 0, test_pattern: 0,
            flip_x: false, flip_y: false, night: OV7670_NIGHT_MODE_OFF,
        };
        dev.write_register(REG_COM7, COM7_RESET)?;
        cortex_m::asm::delay(1000);
        dev.colorspace = OV7670_COLOR_RGB;
        dev.write_list(OV7670_INIT)?;
        dev.size = OV7670_SIZE_DIV8;
        dev.frame_control(dev.size)?;
        dev.test_pattern = OV7670_TEST_PATTERN_NONE;
        Ok(dev)
    }

    /// Capture image into buffer (aka framebuffer)
    pub fn capture(&mut self, buf: &mut [u8]) {
        self.capture.capture(buf);
    }

    /// deinit camera & pins
    pub fn deinit(&mut self) {
        self.capture.deinit();
        if let Some(ref mut pwm) = self.mclk {
            pwm.disable();
        }
        if let Some(ref mut pin) = self.shutdown {
            let _ = pin.set_low();
        }
        if let Some(ref mut pin) = self.reset {
            let _ = pin.set_low();
        }
    }

    /// img <w>
    pub fn width(&self) -> usize {
        640 >> self.size
    }

    /// img h^
    pub fn height(&self) -> usize {
        480 >> self.size
    }

    /// set/get colorspace
    pub fn set_colorspace(&mut self, cs: u8) -> Result<(), ()> {
        self.colorspace = cs;
        let seq = if cs == OV7670_COLOR_RGB { &OV7670_RGB } else { &OV7670_YUV };
        self.write_list(seq)
    }

    /// set/get size
    pub fn set_size(&mut self, size: usize) -> Result<(), ()> {
        self.size = size;
        self.frame_control(size)
    }

    /// Priv: send list of register writes
    fn write_list(&mut self, regs: &[u8]) -> Result<(), ()> {
        for chunk in regs.chunks(2) {
            self.write_register(chunk[0], chunk[1])?;
            cortex_m::asm::delay(1000);
        }
        Ok(())
    }
//write read registers in priv
    // Priv: write
    fn write_register(&mut self, reg: u8, val: u8) -> Result<(), ()> {
        self.i2c.write(OV7670_ADDR, &[reg, val]).map_err(|_| ())
    }

    // Priv: read 
    fn read_register(&mut self, reg: u8) -> Result<u8, ()> {
        let mut buf = [0u8];
        self.i2c.write_read(OV7670_ADDR, &[reg], &mut buf).map_err(|_| ())?;
        Ok(buf[0])
    }

    // Priv: configure frame control for sizing
    fn frame_control(&mut self, size: usize) -> Result<(), ()> {
        let win = &WINDOW[size];
        let (vstart, hstart, edge, pclk) = (win[0], win[1], win[2], win[3]);

        let mut val = if size > OV7670_SIZE_DIV1 { 0x04 } else { 0 };
        if size == OV7670_SIZE_DIV16 { val |= 0x08; }
        self.write_register(0x0C, val)?;

        let val = if size > OV7670_SIZE_DIV1 { 0x18 + size as u8 } else { 0x00 };
        self.write_register(0x3E, val)?;
    
        let sc = if size <= OV7670_SIZE_DIV8 { size as u8 } else { OV7670_SIZE_DIV8 as u8 };
        self.write_register(0x72, sc * 0x11)?;

        let val = if size > OV7670_SIZE_DIV1 { 0xF0 + size as u8 } else { 0x08 };
        self.write_register(0x73, val)?;
  
        let z = if size == OV7670_SIZE_DIV16 { 0x40 } else { 0x20 };
        let xsc = (self.read_register(0x70)? & 0x80) | z;
        let ysc = (self.read_register(0x71)? & 0x80) | z;
        self.write_register(0x70, xsc)?;
        self.write_register(0x71, ysc)?;
        // window registers
        let vstop = (vstart as u16 + 480) as u8;
        let hstop = ((hstart as u16 + 640) % 784) as u8;
        self.write_register(0x17, hstart >> 3)?;
        self.write_register(0x18, hstop >> 3)?;
        let href = ((edge << 6) | ((hstop & 0x07) << 3) | (hstart & 0x07)) as u8;
        self.write_register(0x32, href)?;
        self.write_register(0x19, vstart >> 2)?;
        self.write_register(0x1A, (vstop >> 2) as u8)?;
        let vref = (((vstop & 0x03) << 2) | (vstart & 0x03)) as u8;
        self.write_register(0x03, vref)?;
        self.write_register(0xA2, pclk)?;
        Ok(())
    }

    /// prod Id
    pub fn product_id(&mut self) -> Result<u8, ()> {
        self.read_register(REG_PID)
    }

    /// prod version
    pub fn product_version(&mut self) -> Result<u8, ()> {
        self.read_register(REG_VER)
    }
}
