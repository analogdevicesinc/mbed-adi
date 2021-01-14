Evaluation Boards/Products Supported
------------------------------------ 
EVAL-AD1234 (AD1234)
EVAL-AD1256 (AD1256)
<< add more here >>

Overview
--------
These code files provide drivers to interface with AD1234 and communicate with 
EVAL-AD1234 board. This code was developed and tested on SDP-K1 controller board 
https://os.mbed.com/platforms/SDP_K1/. 

Product details: https://www.analog.com/en/products/ad1234.html
Eval board details: https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD1234.html


Hardware Setup
--------------
Required: SDP-K1, EVAL-AD1234, USB cable, 12 V power supply, 60 MHz external 
clock supply.
Plug in the EVAL-AD124 board on SDP-K1 board (or any other Mbed enabled 
controller board) using the SDP connector and screws.
Connect SDP-K1 board to the PC using the USB cable.


How to Get Started
------------------
Open Mbed online compiler. https://ide.mbed.com/compiler
Import Code into compiler from here: https://os.mbed.com/teams/AnalogDevices/code/EVAL-AD1234/
instructions on how to import code are here: https://os.mbed.com/docs/mbed-os/v5.12/tools/importing-code.html
Compile code. Drag and drop binary into SDP-K1 controller board. Find detailed instructions 
here: https://os.mbed.com/docs/mbed-os/v5.12/tools/getting-your-program-on-your-board.html
Open Tera Term (or alternative), select 9600 baud rate, and the applicable COM port to see the 
list of options.

A detailed user guide on how to use SDP-K1 board on Mbed platform is available 
here: https://wiki.analog.com/resources/tools-software/mbed


Notes
-----
If using Win 7, install serial drivers for Mbed. https://os.mbed.com/docs/mbed-os/v5.12/tutorials/windows-serial-driver.html
A detailed user guide on SDP-K1 controller board is available here https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/SDP-K1.html.


License
-------
Copyright (c) 2021 Analog Devices, Inc.  All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice, 
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.  
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors/products 
  manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner 
  that would cause the software to become subject to terms and conditions which 
  differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its contributors 
  may be used to endorse or promote products derived from this software without 
  specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one or 
  more patent holders.  This license does not release you from the requirement 
  that you obtain separate licenses from these patent holders to use this software.
 
THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, 
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN 
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL 
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
2019-01-10-7CBSD SLA
