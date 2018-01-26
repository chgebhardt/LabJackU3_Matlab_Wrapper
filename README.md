# LabJackU3

The goal is to create a fairly comprehensive MATLAB wrapper class for controlling a LabJack U3 via the Exodriver under Linux/MacOSX  

(Note: class currently opens LabJack if only one device was found and if this device is a U3! changes to the class are necessary if you want to run multiple Labjacks and/or a U6)

inspired by iandol, M.A. Hopcroft and other people in the LabJack forum, you guys are great :-)  

U3 manual: https://labjack.com/support/datasheets/u3  
U3 exodriver: https://github.com/labjack/exodriver  

2017/03/08   
implemented basic exodriver functions, checksum generation, no StreamTO function yet!  

2017/03/09   
added low-level LED function (manual 5.2.5.4)  
added low-level ConfigU3 function (manual 5.2.2)  
added low-level ConfigIO function (manual 5.2.3)  
added low-level ConfigTimerClock function, (manual: 5.2.4)  
added FPuint8ArrayToFPDouble for calibration data conversion  
added low-level ReadMem function  
added get_calibration_constants  
added low-level AIN function (manual 5.2.5.1)  
added low-level WaitShort function (manual 5.2.5.2)  
added low-level WaitLong function (manual 5.2.5.3)  
added low-level BitStateRead function (manual 5.2.5.5)  
added low-level BitStateWrite function (manual 5.2.5.6)  
added low-level BitDirRead function (manual 5.2.5.7)  
added low-level BitDirWrite function (manual 5.2.5.8)  
added low-level PortStateRead function (manual 5.2.5.9)  
added low-level PortStateWrite function (manual 5.2.5.10)  
added low-level PortDirRead function (manual 5.2.5.11)  
added low-level PortDirWrite function (manual 5.2.5.12)  
added low-level DAC#(8-bit) function (manual 5.2.5.13)  
added low-level DAC#(16-bit) function (manual 5.2.5.14)  
added low-level Timer function (manual 5.2.5.15)  
added low-level TimerConfig function (manual 5.2.5.16)  
added low-level Counter function (manual 5.2.5.17)  
added low-level Reset function, (manual: 5.2.9)  
                
2017/03/10        
added error_handling function (called from within each function)   
added generic feedback function     
                 
2017/03/15       
finished feedback function  

To-Do:  

       - are DirWrite and StateWrite opposites? i.e. is it possible to have State = 0 and Dir = In at the same time??? -> Forum?       
       - test all functions if they work  
       - stream  
       - enumeration class: error handling/messaging (need to know which function threw the error and the message)  
       - verbose and debug modes  
       - ConfigU3: no write functionality yet  
       - ConfigIO: * no DAC1enable or UART Pins option implemented  
                   * write better documentation (what is FIOAnalog etc)  
       - ConfigTimerClock: * put vararg for read_write and the ClockDivisor because its not necessary or even working  
                           if read_write is 0 and ClockRate 0-2  
                           * check if port was configured as timer first  
                           * Note that Counter0 is not available with certain timer clock base frequencies  
                            (those that support a divisor). In such a case, it does not use an external FIO/EIO pin.  
                            An error will result if an attempt is made to enable Counter0 when one of these frequencies is configured.  
                            Similarly, an error will result if an attempt is made to configure one of these frequencies 
                            when Counter0 is enabled.                     
       - AIN: data calibration not fully tested, also some parameter combinations might not work   
       - PortStateWrite: writes always to all portbits! (writeMask = [255,255,255])  
       - PortDirWrite:  writes always to all portbits! (writeMask = [255,255,255])  
       - move methods (in private maybe etc) and properties access  
       - what is dev_descriptor_release_number?  
       - check the timer function !!!

Usage (assumes Exodriver is installed): 
 
lj = LabJack;   % class initialization, loads calibration constants, device info and sets inputs to default values

lj.get_feedback(lj.LED(0)) % switches LED off

lj.get_feedback(lj.LED(1)) % switches LED back on

