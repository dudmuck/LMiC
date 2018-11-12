## LMIC update to add support for LoRaWAN-1.1
The primary example project is localed in ``examples/transmit``

### provisioning
    moved to loraconfig.h
#### provision OTA
Enabled by defining JOINEUI: new session created upon join-accept if ROOT_APPKEY is defined, then device is lorawan-1.1 if only ROOT_NWKKEY is defined, then device is lorawan-1.0 
    
When ``DEVEUI`` is undefined, the DevEUI is dirived from CPU unique serial number; recommended to use this method.  DevEUI is known at runtime.  Alternately, the DevEUI can be forced by defining ``DEVEUI``.
#### provision ABP
Enabled by defining DEVADDR: session is permanent if NWK_S_KEY is defined, then device is lorawan-1.0 if SNWKSINTKEY and NWKSENCKEY and FNWKSINTKEY are defined, then device is lorawan-1.1 

In any case APP_S_KEY defines the application payload encryption key
### implementation

Purpose is for lorawan end-devices which do not have eeprom for non-volatile storage of sequence numbers (i.e. flash only), or using toolchain which is incapable of building mbed-os.  Generally, you should be using [LoRaWAN from mbed](https://os.mbed.com/cookbook/LoRa), but if the aformentioned restrictions prevent use of mbed, then this LMIC remains an option.

Originally LMiC only supported an STM32L151 using high-speed timer;  This adds portable low-power timer support for deep sleep.
oslmic is removed, by lmic remains intact.  oslmic is replaced with low-power ticker from mbed; only a small C portion of mbed: no C++.   In LMIC, variable names changed to match that in LoRaWAN specification, along with code updates to comply with the current specification.

Primary goal is portability: keeping implementation only to the lora-alliance specification.

### non-volatile memory
Only stores LoRaWAN sequence numbers. Nordic platform uses fstorage flash page.  STM32L1/L0 uses eeprom. TODO STM32L4xx with flash.
#### nvm OTA
if device is lorawan-1.1: JoinNonce and RJCount1 in NVM. RJCount0 and frame counters in RAM because it resets upon joining.
#### nvm ABP
All frame counters in NVM because session is permanent.

### platforms
    * NUCLEO-L073RZ
    * NUCLEO-L152Re
    * NRF52-DK
    * DISCO_L072CZ-LRWAN1

### STM32 platforms
Use [openstm32 toolchain](http://openstm32.org/HomePage) workspace is called ``ac6_workspace``

### Nordic platforms
Use [segger embedded studio](https://www.segger.com/products/development-tools/embedded-studio/) Project located at ``examples/transmit/ses``.  Provided is BLE-uart example.   Tested on NRF52-DK.

### fixed:
    * remove region-specific code from lmic.c to lmic_<region.[ch]: support any number of regions
    * add support for RX1DRoffset in join accept and RX1DRoffset in RXParamSetupReq
    * add support for receive window delay in join accept and RXTimingSetupReq
    * lorabase.h: change mac command names to those matching lorawan document
    * PingSlotChannelReq: add datarate field
    * pass to appliction layer, result of LinkCheckAns and DeviceTimeAns, via onEvent()
    * mac commands to be (re)sent until downlink received: RXTimingSetupAns, RXParamSetupAns.  See end of processDnData()

### TODO:
    * add regions
    * enforce max frame length
    * BeaconFreqReq, BeaconFreqAns
    * target MOTE-L152RC
    * target NUCLEO-L476RG
