//! Pwm

use core::marker::PhantomData;
use gpio;
use hal;
use timer;

/// pwm
pub struct PwmBinding<P: gpio::GPIOPin,
 C: timer::TimerChannel,
 AF: gpio::AltFnNum>
{
    pin: P,
    channel: C,
    _af: PhantomData<AF>,
}

impl<P: gpio::GPIOPin, C: timer::TimerChannel, AF: gpio::AltFnNum>
    PwmBinding<P, C, AF>
{
    /// opun
    pub fn release(self) -> (P, C) {
        (self.pin, self.channel)
    }
}

impl<P: gpio::GPIOPin, C: timer::TimerChannel, AF: gpio::AltFnNum> hal::PwmPin
    for PwmBinding<P, C, AF>
{
    type Duty = u32;
    fn disable(&mut self) {
        self.channel.disable()
    }

    fn enable(&mut self) {
        self.channel.enable()
    }

    fn get_duty(&self) -> u32 {
        self.channel.read_ccr()
    }

    fn get_max_duty(&self) -> u32 {
        self.channel.read_arr()
    }

    fn set_duty(&mut self, duty: u32) {
        self.channel.write_ccr(duty)
    }
}

/// PwmExtension trait
pub trait PwmExt {
    /// type of pin
    type P: gpio::GPIOPin;
    /// type of channel
    type C: timer::TimerChannel;
    /// type for AF
    type AF: gpio::AltFnNum;
    /// binding
    type Output;
    /// Configures pin and channel to create pwm binding
    fn new(pin: Self::P, channel: Self::C) -> Self::Output;
}

macro_rules! pwm {
    (
        $CRFN:ident,($PINMOD:ident, $PIN:ident),($TIM:ident, $CHN:ident, $CHPE:expr),($AF:ident, $PP:ident, $SPEED:ident)
    ) => {
        impl<PT: gpio::PullType, PM: gpio::PinMode, CM: timer::ChMode> PwmExt
            for PwmBinding<gpio::$PINMOD::$PIN<PT, PM>,
                           timer::$TIM::Channel<timer::$CHN, CM>,
                           gpio::$AF>
        {
            type P = gpio::$PINMOD::$PIN<PT, PM>;
            type C = timer::$TIM::Channel<timer::$CHN, CM>;
            type AF = gpio::$AF;
            type Output =
                PwmBinding<gpio::$PINMOD::$PIN<PT,
                                               gpio::AltFn<gpio::$AF,
                                                           gpio::$PP,
                                                           gpio::$SPEED>>,
                           timer::$TIM::Channel<timer::$CHN, timer::Pwm1>,
                           gpio::$AF>;
            fn new(pin: Self::P, channel: Self::C) -> Self::Output {
                let pin = pin.alternating(gpio::$AF)
                             .output_speed(gpio::$SPEED)
                             .output_type(gpio::$PP)
                             .alt_fn(gpio::$AF);
                let mut channel = channel.mode(timer::Pwm1);
                channel.preload($CHPE);
                PwmBinding { pin,
                             channel,
                             _af: PhantomData, }
            }
        }

        impl<PT: gpio::PullType, PM: gpio::PinMode, CM: timer::ChMode>
            PwmBinding<gpio::$PINMOD::$PIN<PT, PM>,
                       timer::$TIM::Channel<timer::$CHN, CM>,
                       gpio::$AF>
        {
            /// Binds pin to channel to init pwm
            pub fn $CRFN(
                pin: gpio::$PINMOD::$PIN<PT, PM>,
                channel: timer::$TIM::Channel<timer::$CHN, CM>)
                -> PwmBinding<gpio::$PINMOD::$PIN<PT,
                                                  gpio::AltFn<gpio::$AF,
                                                              gpio::$PP,
                                                              gpio::$SPEED>>,
                              timer::$TIM::Channel<timer::$CHN, timer::Pwm1>,
                              gpio::$AF> {
                let pin = pin.alternating(gpio::$AF)
                             .output_speed(gpio::$SPEED)
                             .output_type(gpio::$PP)
                             .alt_fn(gpio::$AF);
                let mut channel = channel.mode(timer::Pwm1);
                channel.preload($CHPE);
                PwmBinding { pin,
                             channel,
                             _af: PhantomData, }
            }
        }
    };
}

// XXX: don't force speed?
// XXX: don't reset preload?
// XXX: don't force Pwm1? allow Pwm2 as well?

pwm!(bind_pa0_tim2_ch1,
     (gpioa, PA0),
     (tim2, CH1, true),
     (AF1, PushPull, MediumSpeed));

pwm!(bind_pa1_tim2_ch2,
     (gpioa, PA1),
     (tim2, CH2, true),
     (AF1, PushPull, MediumSpeed));

pwm!(bind_pa2_tim2_ch3,
     (gpioa, PA2),
     (tim2, CH3, true),
     (AF1, PushPull, MediumSpeed));

pwm!(bind_pa3_tim2_ch4,
     (gpioa, PA3),
     (tim2, CH4, true),
     (AF1, PushPull, MediumSpeed));

pwm!(bind_pa0_tim2_ch1_af2,
     (gpioa, PA0),
     (tim2, CH1, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pc6_tim3_ch1,
     (gpioc, PC6),
     (tim3, CH1, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pc7_tim3_ch1,
     (gpioc, PC7),
     (tim3, CH2, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pc8_tim3_ch1,
     (gpioc, PC8),
     (tim3, CH3, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pc9_tim3_ch1,
     (gpioc, PC9),
     (tim3, CH4, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pb0_tim3_ch3,
     (gpiob, PB0),
     (tim3, CH3, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pb1_tim3_ch4,
     (gpiob, PB1),
     (tim3, CH4, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pb6_tim4_ch1,
     (gpiob, PB6),
     (tim4, CH1, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pb7_tim4_ch2,
     (gpiob, PB7),
     (tim4, CH2, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pb8_tim4_ch3,
     (gpiob, PB8),
     (tim4, CH3, true),
     (AF2, PushPull, MediumSpeed));

pwm!(bind_pb9_tim4_ch4,
     (gpiob, PB9),
     (tim4, CH4, true),
     (AF2, PushPull, MediumSpeed));
