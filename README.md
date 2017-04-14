Power supply with WiFi, USB and UART interface using STM32F103, ESP8266 and MCP41010
----

### Description

This project is about a remote controlled PSU using various interfaces like network (WiFi) and USB. An ESP8266 (ESP-05) module is used to provide a web interface and a simple TCP protocol to control the PSU. Also the STM32's USB interface provides control through a CDC interface. I've used a generic LM2596 PSU like the following which is very cheap to buy in ebay. This PSU has a variable 10K pot that sets the output voltage.

![LM2596 module](http://www.stupid-projects.com/wp-content/uploads/2017/04/LM2596-300x300.jpg)

You need to unsolder and replace the 10K pot and use a digital pot in its place. For that purpose I've used the MCP41010 digital pot from microchip. The original pot is logarithmic but the digital pot is linear; therefore the PSU has better resolution in lower voltages especially under 6V and worst for over 6V. Finally, an opto-isolated relay is used in the output to control the ON/OFF of the PSU.

This is a post about this stupid project.

http://www.stupid-projects.com/wifi-digital-control-dc-power-supply-with-web-interface-and-usb/

### How to compile and flash
You need cmake to build this project either on Windows or Linux. To setup the cmake properly follow the instructions from [here](https://github.com/dimtass/cmake_toolchains/blob/master/README.md). Then edit the cmake/TOOLCHAIN_arm_none_eabi_cortex_m3.cmake file and point TOOLCHAIN_DIR to the correct GCC path.
> e.g. on Windows
> set(TOOLCHAIN_DIR C:/opt/gcc-arm-none-eabi-4_9-2015q3-20150921-win32)
> or on Linux
> set(TOOLCHAIN_DIR /opt/gcc-arm-none-eabi-4_9-2015q3)

Then on Windows run ```build.cmd``` or on Linux run ```./build.sh``` and the .bin and .hex files should be created in the ```build-stm32/src``` folder. Also, a .cproject and .project files are created if you want to edit the source code.

To flash the HEX file in windows use st-link utility like this:
```"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -c SWD -p build-stm32\src\stm32f103_wifi_usb_psu.hex -Rst```

To flash the bin in Linux:
```st-flash --reset write build-stm32/src/stm32f103_wifi_usb_psu.bin 0x8000000```

### Web interface
The web interface is embedded in the STM32; therefore you need to set the IP of the module in the inc/http_server.h file. The ESP8266 is set to station mode, connects to a WiFi router and gets an IP with DCHP. For this reason you need to set your router to assign always a specific IP to the ESP8266 MAC address, otherwise you need to change the IP in the inc/http_server.h every time. There's a way to do that automatically but you need to write the code your selves :D

In case you need to edit the html file in the http_server.c it's best to keep it simple and small, so the ESP8266 doesn't have to transfer big chunks of data every time and the loading time is faster. In the begin of the http_server.c file there are some definitions the each one corresponds to a button. For example the ```BTN_VOLT_1``` is the DOM id of the button that is sent with an ajax POST to the ESP8266 and the ```BTN_VOLT_1_STR``` is the string for the button, which by default is 2V5. If you need to change the voltage string e.g. to 3V you just need to change ```BTN_VOLT_1_STR``` to "3V". The handler code for this button is the same file in http_handler() function.

By default the ON and OFF switch turns on and off the output relay. The +/- buttons do a micro-trimming of the output voltage by changing the MCP41010 value only by a +/-1 step. The --/++ buttons change the MCP41010 pot value by +/-10 steps. The min and max values for MCP41010 are 0 and 255. Also 5 more buttons for 2V5, 3V3, 5V, 6V and 12V are in there by default. You can change these values to the ones that you prefer in the http string in http_server.c and then use the calibration procedure to store the correct values.

### TCP. CDC-USB and UART interface
You can also control the PSU output by ending simple ASCII text commands on a TCP socket, the CDC-USB or the UART1 port.
The pinout for the UART1 is the following:
UART1_TX : PA9
UART1_RX : PA10
The baudrate for the UART1 is 115200 and the rest settings are 8/N/1. You can use the same settings for CDC-USB interface.

The TCP IP and port are the same with the web interface (e.g. 192.168.0.80 and 80) and you can use any small TCP desktop or smartphone client to send the commands.

The supported commands are the following (you may use a newline at the end if you like):

```VERSION```
    Gets the version string of the firmware (includind date and time)

```POT=<VALUE>```
    sets the MCP41010 value, where <VALUE> is any value from 0 to 255

```SETPOT=<ID>,<VALUE>```
    sets the MCP41010 configuration value, where <ID> is one of the predefined configuration ids (1-5) and <VALUE> is any value from 0 to 255
	
```GETPOT=<ID>```
    gets the MCP41010 configuration value of the <ID>

```SAVE```
    saves the current pot value in the flash

```SSID=<ssid name>```
    saves the SSID name that will be used to connect

```PASS=<ssid password>```
    saves the SSID password that will be used

```RECONNECT```
    reconnects to the WiFi router

```AT=<at command>```
    sends an AT command to ESP8266 (this is used for debugging)

### Calibration
You need to re-calibrate the MCP41010 values for any change is made in to the module's input voltage. So, if you use a 12V power supply for a first time, you need to do the calibration procedure. If for some reason you change the input voltage or you find out at some point the input voltage changed, then you need to run the calibration procedure again. To do the calibration you need to connect a multimeter to the Relay output and then from the web interface select the voltage you need to calibrate by pressing the corresponding button. If the volt-meter shows a different value from the expected then press the +/- (or --/++) until you get the wanted value and then press the (S) button to save the values to STM32's flash.

You can also do the calibration from the debug UART or the CDC-USB interface using the ```SETPOT=<ID>,<VALUE>``` and ```SAVE``` commands.

### LED patterns
The LED uses some patterns to visualize the code state when the debug uart is not connected. If the LED toggles every 250ms (is a quite fast blinking) then it means that it tries to connect to the WiFi router. If the LED toggles every 500ms (slow blinking) then it means that is connected to the router. If it's always on, then there is a hard-fault. You can change these patterns in the led_pattern.c file.
