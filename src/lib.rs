#![no_std]

use cortex_m::peripheral::SYST;
use rtic_monotonic::{
    embedded_time::{clock::Error, fraction::Fraction},
    Clock, Instant, Monotonic
};
use cortex_m::peripheral::syst::SystClkSource;
use embedded_hal::blocking::delay::{DelayUs, DelayMs};

#[cfg(feature = "l0x1-tim21-tim22")]
use stm32l0::stm32l0x1 as pac;
#[cfg(feature = "f0x1-tim15-tim17")]
use stm32f0::stm32f0x1 as pac;
#[cfg(feature = "f0x2-tim15-tim17")]
use stm32f0::stm32f0x2 as pac;
#[cfg(feature = "f0x1-tim15-tim16")]
use stm32f0::stm32f0x1 as pac;
#[cfg(feature = "h743-tim15-tim17")]
use stm32h7::stm32h743 as pac;
use core::fmt::Formatter;

#[cfg(feature = "l0x1-tim21-tim22")]
pub type TimMsb = pac::TIM21;
#[cfg(feature = "l0x1-tim21-tim22")]
pub type TimLsb = pac::TIM22;

#[cfg(feature = "f0x1-tim15-tim17")]
pub type TimMsb = pac::TIM15;
#[cfg(feature = "f0x1-tim15-tim17")]
pub type TimLsb = pac::TIM17;

#[cfg(feature = "f0x2-tim15-tim17")]
pub type TimMsb = pac::TIM15;
#[cfg(feature = "f0x2-tim15-tim17")]
pub type TimLsb = pac::TIM17;

#[cfg(feature = "f0x1-tim15-tim16")]
pub type TimMsb = pac::TIM15;
#[cfg(feature = "f0x1-tim15-tim16")]
pub type TimLsb = pac::TIM16;

#[cfg(feature = "h743-tim15-tim17")]
pub type TimMsb = pac::TIM15;
#[cfg(feature = "h743-tim15-tim17")]
pub type TimLsb = pac::TIM17;

pub struct TimSystickMonotonic<const FREQ: u32> {
    systick: SYST,
    pub tim_msb: TimMsb,
    pub tim_lsb: TimLsb,
}

impl<const FREQ: u32> core::fmt::Debug for TimSystickMonotonic<FREQ> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "TimSystickMonotonic({})", FREQ)
    }
}

impl<const FREQ: u32> TimSystickMonotonic<FREQ> {
    pub fn new(systick: SYST, mut tim_msb: TimMsb, mut tim_lsb: TimLsb, sysclk: u32) -> Self {
        assert_eq!(FREQ, sysclk);
        Self::timers_init(&mut tim_msb, &mut tim_lsb);

        TimSystickMonotonic {
            systick,
            tim_msb,
            tim_lsb,
        }
    }

    fn timers_init(tim_msb: &mut TimMsb, tim_lsb: &mut TimLsb) {
        let device = unsafe { pac::Peripherals::steal() };
        #[cfg(any(feature = "f0x1-tim15-tim17", feature = "f0x2-tim15-tim17", feature = "h743-tim15-tim17"))] {
            device.RCC.apb2enr.modify(|_, w| w.tim15en().enabled().tim17en().enabled());
            device.RCC.apb2rstr.modify(|_, w| w.tim15rst().reset().tim17rst().reset());
            device.RCC.apb2rstr.modify(|_, w| w.tim15rst().clear_bit().tim17rst().clear_bit());
        }
        #[cfg(feature = "f0x1-tim15-tim16")] {
            device.RCC.apb2enr.modify(|_, w| w.tim15en().enabled().tim16en().enabled());
            device.RCC.apb2rstr.modify(|_, w| w.tim15rst().reset().tim16rst().reset());
            device.RCC.apb2rstr.modify(|_, w| w.tim15rst().clear_bit().tim16rst().clear_bit());
        }
        #[cfg(feature = "l0x1-tim21-tim22")] {
            device.RCC.apb2enr.modify(|_, w| w.tim21en().set_bit().tim22en().set_bit());
            device.RCC.apb2rstr.modify(|_, w| w.tim21rst().set_bit().tim22rst().set_bit());
            device.RCC.apb2rstr.modify(|_, w| w.tim21rst().clear_bit().tim22rst().clear_bit());
        }

        // LSB timer init, F0: TIM16/17, L0: TIM22, H7: TIM17
        tim_lsb.cr1.modify(|_, w| w.cen().clear_bit());
        #[cfg(feature = "l0x1-tim21-tim22")]
        tim_lsb.cr2.modify(|_, w| w.mms().update()); // update event as trigger ouput (TRGO)
        #[cfg(any(feature = "f0x1-tim15-tim17", feature = "f0x2-tim15-tim17", feature = "f0x1-tim15-tim16", feature = "h743-tim15-tim17"))] {
            tim_lsb.ccmr1_output_mut().modify(|_, w| unsafe { w.oc1m().bits(0b110) }); // PWM Mode 1
            tim_lsb.ccer.modify(|_, w| w.cc1e().set_bit()); // Output compare enable
            tim_lsb.ccr1.write(|w| unsafe { w.bits(0xffff) }); // Clock msb timer on overflow
            tim_lsb.bdtr.modify(|_, w| w.moe().set_bit()); // Enable
        }
        tim_lsb.cnt.reset();
        tim_lsb.psc.write(|w| w.psc().bits(0)); // Run at the same freq as SysClk
        tim_lsb.cr1.modify(|_, w| w.urs().set_bit());
        tim_lsb.egr.write(|w| w.ug().set_bit());

        // MSB timer init, F0: TIM15, L0: TIM21, H7: TIM15
        tim_msb.cr1.modify(|_, w| w.cen().clear_bit());
        #[cfg(feature = "f0x1-tim15-tim16")]
        tim_msb.smcr.modify(|_, w| unsafe { w.ts().bits(0b010).sms().bits(0b111) }); // F0: clock from TIM16_OC
        #[cfg(any(feature = "f0x1-tim15-tim17", feature = "f0x2-tim15-tim17"))]
        tim_msb.smcr.modify(|_, w| unsafe { w.ts().bits(0b011).sms().bits(0b111) }); // F0: clock from TIM17_OC
        #[cfg(feature = "l0x1-tim21-tim22")]
        tim_msb.smcr.modify(|_, w| w.ts().itr1().sms().ext_clock_mode()); // L0: clock from TIM22
        #[cfg(feature = "h743-tim15-tim17")]
        tim_msb.smcr.modify(|_, w| unsafe { w.ts_2_0().bits(0b011).ts_4_3().bits(0b00).sms().bits(0b111) }); //ITR3: 0b0011, ext_clock_mode: 0111

        tim_msb.cr1.modify(|_, w| w.cen().set_bit());
        tim_lsb.cr1.modify(|_, w| w.cen().set_bit());
    }

    pub fn tim_now(&self) -> u32 {
        let ticks = self.tim_msb.cnt.read().bits() << 16 | self.tim_lsb.cnt.read().bits();
        ticks
    }

    pub fn new_handle(&self) -> MonotonicHandle {
        MonotonicHandle {
            lsb_cnt: &self.tim_lsb.cnt as *const _ as *const u32,
            msb_cnt: &self.tim_msb.cnt as *const _ as *const u32,
            sysclk: FREQ
        }
    }
}

impl<const FREQ: u32> Clock for TimSystickMonotonic<FREQ> {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(1, FREQ);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        Ok(Instant::new(self.tim_now()))
    }
}

impl<const FREQ: u32> Monotonic for TimSystickMonotonic<FREQ> {
    const DISABLE_INTERRUPT_ON_EMPTY_QUEUE: bool = true;

    unsafe fn reset(&mut self) {
        self.systick.set_clock_source(SystClkSource::Core);
        self.systick.set_reload(0x00ff_ffff);
        self.systick.clear_current();
        self.systick.enable_counter();
        self.systick.enable_interrupt();
    }

    fn set_compare(&mut self, instant: &Instant<Self>) {
        // The input `instant` is in the timer, but the SysTick is a down-counter.
        // We need to convert into its domain.
        let now: Instant<Self> = Instant::new(self.tim_now());

        let max = 0x00ff_ffff;

        let dur = match instant.checked_duration_since(&now) {
            None => 1, // In the past

            // ARM Architecture Reference Manual says:
            // "Setting SYST_RVR to zero has the effect of
            // disabling the SysTick counter independently
            // of the counter enable bit.", so the min is 1
            Some(x) => max.min(x.integer()).max(1),
        };

        self.systick.set_reload(dur);
        self.systick.clear_current();
    }

    fn clear_compare_flag(&mut self) {
        // NOOP with SysTick interrupt
    }
}

pub struct MonotonicHandle {
    lsb_cnt: *const u32,
    msb_cnt: *const u32,
    sysclk: u32
}
impl MonotonicHandle {
    pub fn tim_now(&self) -> u32 {
        unsafe { (*self.msb_cnt) << 16 | (*self.lsb_cnt) }
    }
}
unsafe impl Send for MonotonicHandle {}

impl DelayUs<u32> for MonotonicHandle {
    fn delay_us(&mut self, us: u32) {
        let delay_cycles = self.sysclk as u64 * us as u64 / 1_000_000;
        let mut elapsed = 0;
        let mut count_prev = self.tim_now();
        while elapsed < delay_cycles {
            let count_now = self.tim_now();
            let dt = count_now.wrapping_sub(count_prev);
            count_prev = count_now;
            elapsed += dt as u64;
            cortex_m::asm::delay(10);
        }
    }
}

impl DelayMs<u32> for MonotonicHandle {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}
