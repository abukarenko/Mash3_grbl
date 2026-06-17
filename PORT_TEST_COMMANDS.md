# Mash3_grbl port test commands

Firmware exposes a USB CDC virtual COM port.

Open the COM port with any baud rate and send commands terminated by Enter.

```text
?                  print help
s                  print input/output status
idc                print only 10-pin IDC/MPG connector status
led 0|1            set PC2 LED
en 0|1             set PB10 buffer enable
out 1..4 0|1       set OUT1..OUT4
outtest 1..4       blink one OUT pin slowly 10 times
dir x|y|z|a 0|1    set axis direction pin
step x|y|z|a N     send N step pulses, max 10000
pwm 0..1000        set PA8 TIM1_CH1 PWM duty, 1000 = 100%
spindle 0|1        shortcut for pwm 0 or pwm 1000
usb 0|1            disconnect/connect USB D+ pull-up on PC11
cycle              switch OUT1..OUT4 one by one
```

First safe test order:

1. Send `?`.
2. Send `s`.
3. Check limit inputs with `s`.
4. Check `out 1 1`, `out 1 0`, etc.
5. Keep `en 0` until motor outputs are safe to test.
6. Test one axis with `dir x 1`, then `step x 10`.

`PA8` is configured as `TIM1_CH1` PWM by the test firmware at runtime. The current PWM frequency is about 1 kHz.
