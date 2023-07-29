This is a hacky way to generate a vehicle speed signal (VSS) where none is available using the Freematics ONE+ OBD adapter, running on an ESP32. 

Roughly once per second, it reads the OBD speed and uses the `ledc` library to output a varying frequency, fixed duty cycle signal to GPIO26. 

This signal can then be connected to a radio head unit or other required device. Note that the output voltage is 3.3 or 5v given it's coming from ESP32 (not sure, it didn't matter too mujch here) but most VSS signals are usually conditioned to this anyways so it should still work.

The code is very heavily based on [Telelogger](https://github.com/stanleyhuangyc/Freematics/tree/master/firmware_v5/telelogger), but with all the logging and communication aspects removed -- I just wanted the deep sleep and adaptive movement control functionality, but my C++ is very rusty.

If you want to use this, all you'll need to do is:
1. Flash this firmware to an Freematics ONE+ (I used the cheaper ONE+ Model A, but any would work in theory).
2. Plug the ONE+ plug this into your OBD port
3. Wire from GPIO26 to your required VSS signal connector (see ONE+ wiring diagram [here](https://freematics.com/pages/products/freematics-one-plus/guide/#:~:text=from%20car%20battery.-,GPIO%20Socket,-Freematics%20ONE%2B%20has))
4. Done!