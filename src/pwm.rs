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
    /// Consumes PwmBinding returning pin and channel
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
pub trait PwmExt<SP: gpio::OutputSpeed,
                 C: timer::TimerChannel,
                 AF: gpio::AltFnNum>: gpio::GPIOPin
{
    /// type
    type OutputPin: gpio::GPIOPin;
    /// binding
    type Output: hal::PwmPin;
    /// Configures pin and channel to create pwm binding
    fn to_pwm(self, channel: C, sp: SP) -> Self::Output;
}

macro_rules! pwm {
    (
        $CRFN:ident, $PIN:ident,($TIM:ident, $CHN:ident, $CHPE:expr),($AF:ident, $PP:ident)
    ) => {
        impl<SP: gpio::OutputSpeed,
              PT: gpio::PullType,
              PM: gpio::PinMode,
              CM: timer::ChMode>
            PwmExt<SP, timer::$TIM::Channel<timer::$CHN, CM>, gpio::$AF>
            for gpio::$PIN<PT, PM>
        {
            type OutputPin =
                gpio::$PIN<PT, gpio::AltFn<gpio::$AF, gpio::$PP, SP>>;
            type Output = PwmBinding<Self::OutputPin,
                                     timer::$TIM::Channel<timer::$CHN,
                                                          timer::Pwm1>,
                                     gpio::$AF>;
            fn to_pwm(self,
                      channel: timer::$TIM::Channel<timer::$CHN, CM>,
                      sp: SP)
                      -> Self::Output {
                let pin = self.alternating(gpio::$AF)
                              .output_speed(sp)
                              .output_type(gpio::$PP)
                              .alt_fn(gpio::$AF);
                let mut channel = channel.mode(timer::Pwm1);
                channel.preload($CHPE);
                PwmBinding { pin,
                             channel,
                             _af: PhantomData }
            }
        }

        impl<PT: gpio::PullType, PM: gpio::PinMode, CM: timer::ChMode>
            PwmBinding<gpio::$PIN<PT, PM>,
                       timer::$TIM::Channel<timer::$CHN, CM>,
                       gpio::$AF>
        {
            /// Modify channel's preload
            pub fn channel_preload(&mut self, enabled: bool) {
                self.channel.preload(enabled)
            }
        }
    };
}

// XXX: don't force Pwm1? allow Pwm2 as well?

pwm!(bind_pa0_tim2_ch1, PA0, (tim2, CH1, true), (AF1, PushPull));

pwm!(bind_pa1_tim2_ch2, PA1, (tim2, CH2, true), (AF1, PushPull));

pwm!(bind_pa2_tim2_ch3, PA2, (tim2, CH3, true), (AF1, PushPull));

pwm!(bind_pa3_tim2_ch4, PA3, (tim2, CH4, true), (AF1, PushPull));

pwm!(bind_pa6_tim3_ch1, PA6, (tim3, CH1, true), (AF2, PushPull));

pwm!(bind_pa7_tim3_ch2, PA7, (tim3, CH2, true), (AF2, PushPull));

pwm!(bind_pc6_tim3_ch1, PC6, (tim3, CH1, true), (AF2, PushPull));

pwm!(bind_pc7_tim3_ch1, PC7, (tim3, CH2, true), (AF2, PushPull));

pwm!(bind_pc8_tim3_ch1, PC8, (tim3, CH3, true), (AF2, PushPull));

pwm!(bind_pc9_tim3_ch1, PC9, (tim3, CH4, true), (AF2, PushPull));

pwm!(bind_pb0_tim3_ch3, PB0, (tim3, CH3, true), (AF2, PushPull));

pwm!(bind_pb1_tim3_ch4, PB1, (tim3, CH4, true), (AF2, PushPull));

pwm!(bind_pb6_tim4_ch1, PB6, (tim4, CH1, true), (AF2, PushPull));

pwm!(bind_pb7_tim4_ch2, PB7, (tim4, CH2, true), (AF2, PushPull));

pwm!(bind_pb8_tim4_ch3, PB8, (tim4, CH3, true), (AF2, PushPull));

pwm!(bind_pb9_tim4_ch4, PB9, (tim4, CH4, true), (AF2, PushPull));
