# Central device part for Master Thesis


* Eclipse setup: https://devzone.nordicsemi.com/tutorials/7/ 

## Using

Everything should work, given that the SDK_ROOT variables are set correctly. This needs to be set in project settings in two places (create it if it doesn't exist): 

1. Resource->Linked Resources->Path Variables
2. C/C++ Build->Build Variables

All other variables should update themselves/should be updated by eclipse.

## Flashing

To flash the chip, use the already created Build Targets. To use bluetooth, `flash_softdevice` has to be used first (but only the first time), then you can use only `flash` and `run` until you overwrite range between `0x00000` and `0x22000` (softdevice location), after which the `flash_softdevice` hast to be used. You need to use `run` after every flash to start the chip, it can also be used as a reset.

## Changing

To change settings, go to Project settings->C/C++ Build->Settings->Tool Settings. Be sure to include the right Preprocessor defines and the correct linker script (under Linker-General). 

## Eclipse Needs:
*  [nRF5 SDK v13.0.0](http://www.nordicsemi.com/eng/nordic/Products/nRF5-SDK/nRF5-SDK-zip/59011) 
* nrfjprog from Nordic for flashing
* [GNU Arm Eclipse Cross compiler plugin](http://gnuarmeclipse.github.io/plugins/install/). Needed packages:
	* GNU ARM C/C++ Cross Compiler
	* GNU ARM C/C++ Packs
	* GNU ARM C/C++ J-link Debugging 
* 	Patch for SDK - implements required workaround for CDT build output parser. Replace makefile.common in in `$(SDK_ROOT)/components/toolchain/gcc` with one of the files below:
	*  [makefile.common for SDK 12.2.0 and SDK 13.0.0](https://devzone.nordicsemi.com/attachment/05f509ed4864da5b84af0a302766688b)
*   Enter project properties -> C/C++->Preprocessor Include Paths,etc.->Providers
* Click on CDT GCC Build Output Parser and change the compiler command pattern from `(gcc)|([gc]->+->+)|(clang)` to `(.*gcc)|(.*[gc]->+->+)` then apply changes.
* Click on CDT Built-in Compiler Settings Cross ARM and replace `${COMMAND}` with `arm-none-eabi-gcc` and click Apply.
* Add to PATH:
	* nRF5 SDK root folder
	* nrfjprog folder
	* gcc-arm-none-eabi/bin folder
* For flashing using the Build targets, eclipse needs to access the systemwide PATH variable (problem on Mac, if you start eclipse from terminal using `open /Applications/Eclipse.app/` that is taken care of)
	* Alternatively, open Eclipse preferences, under C/C++ -> Build -> Environment, and add Variable `PATH` set to `${PATH}:/usr/local/nrfjprog/`
* To get eclipse to know where to get system headers from (stdlib header files), go to Project Properties->C/C++ Build->Preprocessor Include Paths,..->Entries->GNU C and add the include paths to the CDT User Setting Entries. The include paths are (they sadly aren't carried with the project settings):
	* `{path to gcc-arm-none-eabi}/arm-none-eabi/include`
	* `{path to gcc-arm-none-eabi}/lib/gcc/arm-none-eabi/5.4.1/include`

	
## Debugging output with RTT

If we want to debug multiple devices we need to use the SEGGER_RTT tools.

1. Install [J-Link Software and Documentation Pack](https://www.segger.com/downloads/jlink)
2. Add the installation folder to PATH variable
3. run `JLinkexe -device <DeviceName> -speed <SpeedInkHz> -if <SWD|JTAG> -autoconnect 1 -SelectEmuBySN<RTT chip serial number> -RTTTelnetPort <Port>` (for NRF52840_XXAA run `JLinkexe -device NRF52840_XXAA -speed 4000 -if SWD -autoconnect 1 -SelectEmuBySN 683381009 -RTTTelnetPort 19024`). **You need to keep this terminal window open**, as this acts as the server the RTT Client connects to.
4. You should get some output similar to:

```
SEGGER J-Link Commander V6.32f (Compiled Jun 12 2018 14:49:32)
DLL version V6.32f, compiled Jun 12 2018 14:49:26

RTT Telnet Port set to 19024
Connecting to J-Link via USB...O.K.
Firmware: J-Link OB-SAM3U128-V2-NordicSemi compiled Jul 12 2018 11:44:41
Hardware version: V1.00
S/N: 683570733
VTref=3.300V
Device "NRF52840_XXAA" selected.


Connecting to target via SWD
Found SW-DP with ID 0x2BA01477
Found SW-DP with ID 0x2BA01477
Scanning AP map to find all available APs
AP[2]: Stopped AP scan as end of AP map has been reached
AP[0]: AHB-AP (IDR: 0x24770011)
AP[1]: JTAG-AP (IDR: 0x02880000)
Iterating through AP map to find AHB-AP to use
AP[0]: Core found
AP[0]: AHB-AP ROM base: 0xE00FF000
CPUID register: 0x410FC241. Implementer code: 0x41 (ARM)
Found Cortex-M4 r0p1, Little endian.
FPUnit: 6 code (BP) slots and 2 literal slots
CoreSight components:
ROMTbl[0] @ E00FF000
ROMTbl[0][0]: E000E000, CID: B105E00D, PID: 000BB00C SCS-M7
ROMTbl[0][1]: E0001000, CID: B105E00D, PID: 003BB002 DWT
ROMTbl[0][2]: E0002000, CID: B105E00D, PID: 002BB003 FPB
ROMTbl[0][3]: E0000000, CID: B105E00D, PID: 003BB001 ITM
ROMTbl[0][4]: E0040000, CID: B105900D, PID: 000BB9A1 TPIU
ROMTbl[0][5]: E0041000, CID: B105900D, PID: 000BB925 ETM
Cortex-M4 identified.

```

5. Then start the RTT Client or connect via a Telnet Client to localhost:19021
(run `JLinkRTTClient`). The output will be visible in this window.

Usefull commands to use with JLinkexe:

```
?          Help
usb        Connect to J-Link via USB.  Syntax: usb <port>, where port is 0..3
           This basically resets the connection. You need to use this when
           flashing a new firmware using nrfjprog and then restart JLinkRTTClient
q          Quit
qc         Close JLink connection and quit
r          Reset target         (RESET)
g          go
loadfile   Load data file into target memory.
             Syntax: loadfile <filename>, [<addr>]
             Supported extensions: *.bin, *.mot, *.hex, *.srec
             <addr> is needed for bin files only.
             
ShowEmuList Shows a list of all emulators which are connected to the host.
            The interfaces to search on, can be specified.
             Syntax: ShowEmuList [<Interface0> <Interface1> ...]

```