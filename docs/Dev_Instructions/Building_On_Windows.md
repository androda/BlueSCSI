# Building F4 BlueSCSI From Source On Windows

These instructions are targeted at Windows, because that's what I used to originally do the port work.

1. Install Visual C++ 2010 redistributable
   - Download and install the Visual C++ 2010 Redistributables (*both* x86 and x64) from here: https://www.microsoft.com/en-us/download/details.aspx?id=26999
2. Download the F4 BlueSCSI code from: https://github.com/androda/F4_BlueSCSI
3. Install Arduino IDE 1.8.latest
4. Go to https://github.com/androda/Arduino_STM32 and download the source code zip. (this repo was forked just to ensure I can curate new changes for errors)
5. Extract the Arduino_STM32_F4-master.zip zip file you downloaded.
   - Open the folder and you might see another folder named the same thing.
   - Keep drilling down until you see the folders named drivers, STM32F1, STM32F4, etc.  Keep this folder open.
6. Open Arduino IDE
   - Go to Board Manager and install "Arduino SAM Boards" (1.6.12 has been used here)
   - Open Preferences and find your Sketchbook Location.  Mine was "C:\Users\Username\Documents\Arduino"
7. Close Arduino IDE and navigate to that Sketchbook Location
   - In this folder you will probably only see a folder named 'libraries'.
   - Create a sibling folder (sibling to libraries) named 'hardware'.
   - Inside the hardware folder, create another new folder named 'Arduino_STM32'.
   - Move (or copy) everything from the previous folder (which you kept open, right?) into this Arduino_STM32 folder.
8. Reopen Arduino IDE and you will find under the Boards menu that there are choices for STM32F103, and others.
   - In the board dropdown menu, select "Blackpiill STM32F401CCU6"
   - Also in this dropdown menu select Optimize: "Faster (-O2)"
     - If you don't select this, your resulting bluescsi program will run slower
9. In the Library Manager, install SDFat (by Bill Greiman) version 2.0.6
10. If you now open the F4_BlueSCSI.ino file and try to build... it will fail.  SdSpiSTM32.cpp will appear near the end of the errors, something about control reaching the end of a non-void function.
11. Now the really obtuse part.  We get to modify SDFat.
    - Back in your Arduino Sketchbook folder, open libraries/SdFat/src/SpiDriver/SdSpiSTM32.cpp in a text editor
    - Modify line 59 to remove the word "return" and add a new line underneath saying "return 0;"
12. Keep your text editor open, and open the F4_BlueSCSI.ino arduino project.  If you build now, it should work just fine.
13. Now for another obtuse part: Setting the clock speed.  The difference between STM32F401 (F4 Lite) and STM32F411 (full F4) is nothing more than clock speed.  F4Lite runs at 84MHz, and full F4 runs at 96MHz.
    - To change this, in your arduino sketchbook folder open the file hardware/Arduino_STM32/STM32F4/variants.blackpill_f401/blackpill_f401.h
    - In here you will see a line that #defines CLOCK_SPEED_MHZ.  You can simply change this to 96 for the F411.
    - I need to make a new build target for the maple framework, but every time I've tried there has been some sort of error.
    - F4Lite (F401) reaches 1800k read and 1300k write on my beige g3.  Full F4 (F411) reaches 1800k read and 1500k write.
14. Open the F4_BlueSCSI.ino project file.
15. The code should build successfully!
16. STLink V2 hardware setup
    - Download STLink drivers from: https://www.st.com/en/development-tools/stsw-link009.html
    - They reeeeeally want you to register or at least give them your email address to download this software.  Just use temp-mail.org to get a temporary disposable email and use the download link they email to the temporary email address.
    - Extract the zip and run 'dpinst_amd64.exe'
    - Optionally, use stsw-link-007.  This is a firmware update to make the stlink v2 device compatible with the new stm32cubeprogrammer.  Only necessary if you intend to export binaries from Arduino IDE and program separately.
17. Now you need an STLink V2 clone. 
    - Yes, I said clone - not the actual STLink V2 because they tend not to work for some reason.
    - Get one of the USB stick looking ones, hook up the pins (SWclk, SWdio, 3.3v, gnd) and you can upload from the Arduino IDE straight to your STM32F4 dev board.
18. Connect the STLink to the module via the four necesssary pins (3.3v, GND, SWD, SWCLK) and click the upload button in Arduino IDE.
    - Theoretically, all is well and programming succeeds.
    - If your STLink is not recognized, check the device manager to be sure it appears there as an STLink and not 'unknown device'.
    - Try unplugging and re-plugging the ST-Link.  Some of these clones have issues from time to time.