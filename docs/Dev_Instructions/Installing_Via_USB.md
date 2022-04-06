# Installing F4 BlueSCSI Binaries via USB

**Important note: These instructions haven't always worked for me.  In some cases, the firmware updates happily over USB and in other cases - hard nope, STLink only.  Very low success rate.** 

You need: A USB-C cable.  That's it.

1. Remove the SD card from your BlueSCSI unit.
2. Install STM32CubeProgrammer
   - They want you to give them your email address.  Just use temp-mail.org to get a temp one for the download
3. For F4: Hold down the "Boot0" button and plug the USB cable into your computer
   - Note that there should be only the red power LED lit.  If there's blinking of any kind from the other LED, stop and try again.
4. For F4Lite, switch the DIP position 3 to 'on' before plugging into USB
   - Make sure to switch back to 'off' after programming or you'll be confused why it isn't working at all
5. Start STM32CubeProgrammer
6. In the top right, switch from ST-Link to USB
7. Your dev board should appear when you click the little 'refresh' circle
   - If there's flashing from the activity LED, go back to 3 and run through the hardware setup again.  Only the red power LED should be on.
   - If your dev board does not appear, despite following these steps - sorry, it's not going to work.  I am unable to determine why this happens.
8. Click the 'Open File' tab, select your firmware, and click the blue 'Download' button that appears over on the right
9. Hopefully this works for you, because otherwise you need to buy an STLink V2 clone (yes, clone) for programming updated firmware.