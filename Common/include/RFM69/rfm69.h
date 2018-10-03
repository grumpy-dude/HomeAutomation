// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright LowPowerLab LLC 2018, https://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#ifndef RFM69_h
#define RFM69_h

#include <stdint.h>

#ifndef RASPBERRY
  #include <Arduino.h>            // assumes Arduino IDE v1.0 or greater

  #include "SPI/SPI.h"
#endif


/* ********** NOTE: RF69_SPI_SELECT must be defined. **********

This identifies the GPIO pin (for example on 8-bit Arduino boards or stand-alone microcontrollers) or
SPI device (for example when using "wiringPi" on a Raspberry Pi board) to be used for communication
with the RFM69 module.

For maximum versatility, this is best done on the command-line when invoking the C++ compiler
(e.g. "avr-g++ rfm69.cpp -DRF69_SPI_SELECT=SS ...", "g++ rfm69.cpp -lwiringPi -DRF69_SPI_SELECT=0", etc.).
*/


/* ********** NOTE: RF69_IRQ must be defined. **********

This identifies the GPIO pin or interrupt number to be used by the DIO0 signal coming from RFM69 module.

For maximum versatility, this is best done on the command-line when invoking the C++ compiler
(e.g. "avr-g++ rfm69.cpp -DRF69_IRQ=PCINT4 ...", "g++ rfm69.cpp -lwiringPi -DRF69_IRQ=0", etc.).
*/


#define RF69_FIFO_SIZE          66
#define RF69_MAX_DATA_LEN       (RF69_FIFO_SIZE - 5) // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)

#define RF69_ENCRYPTION_KEY_SIZE 16

#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

//#define RF69_LISTENMODE_ENABLE  //comment this line out to compile sketches without the ListenMode (saves ~2k)

#if defined(RF69_LISTENMODE_ENABLE)
  // By default, receive for 256uS in listen mode and idle for ~1s
  #define  DEFAULT_LISTEN_RX_US 256
  #define  DEFAULT_LISTEN_IDLE_US 1000000
#endif

class RFM69 {
  public:
    static volatile uint8_t DATA[RF69_MAX_DATA_LEN]; // recv/xmit buf, including header & crc bytes
    static volatile uint8_t DATALEN;
    static volatile uint8_t SENDERID;
    static volatile uint8_t TARGETID; // should match _address
    static volatile uint8_t PAYLOADLEN;
    static volatile uint8_t ACK_REQUESTED;
    static volatile uint8_t ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
    static volatile int16_t RSSI; // most accurate RSSI during reception (closest to the reception). RSSI of last packet.
    static volatile uint8_t _mode; // should be protected?

    RFM69(uint8_t slaveSelect, uint8_t interrupt, bool isRFM69HW, uint8_t interruptNum) //interruptNum is now deprecated
                : RFM69(slaveSelect, interrupt, isRFM69HW){};

    RFM69(uint8_t slaveSelect=RF69_SPI_SELECT, uint8_t interrupt=RF69_IRQ, bool isRFM69HW=false);

    bool initialize(uint8_t freqBand, uint8_t ID, uint8_t networkID=1);
    void setAddress(uint8_t addr);
    void setNetwork(uint8_t networkID);
    bool canSend();
    virtual void send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK=false);
    virtual bool sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries=2, uint8_t retryWaitTime=40); // 40ms roundtrip req for 61byte packets
    virtual bool receiveDone();
    bool ACKReceived(uint8_t fromNodeID);
    bool ACKRequested();
    virtual void sendACK(const void* buffer = "", uint8_t bufferSize=0);
    uint32_t getFrequency();
    void setFrequency(uint32_t freqHz);
    void encrypt(const char* key);
    void setCS(uint8_t newSPISlaveSelect);
    int16_t readRSSI(bool forceTrigger=false); // *current* signal strength indicator; e.g. < -90dBm says the frequency channel is free + ready to transmit
    void promiscuous(bool onOff=true);
    virtual void setHighPower(bool onOFF=true); // has to be called after initialize() for RFM69HW
    virtual void setPowerLevel(uint8_t level); // reduce/increase transmit power level
    void sleep();
    uint8_t readTemperature(uint8_t calFactor=0); // get CMOS temperature (8bit)
    void rcCalibration(); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t val);
    void readAllRegs();
    void readAllRegsCompact();

  protected:
    static void isr0();
    void interruptHandler();
    virtual void interruptHook(uint8_t CTLbyte) {};
    static volatile bool _haveData;
    virtual void sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, bool requestACK=false, bool sendACK=false);

    static RFM69* selfPointer;
    uint8_t _slaveSelect;
    uint8_t _interrupt;
    uint8_t _interruptNum;
    uint8_t _address;
    bool _promiscuousMode;
    uint8_t _powerLevel;
    bool _isRFM69HW;
#if defined (SPCR) && defined (SPSR)
    uint8_t _SPCR;
    uint8_t _SPSR;
#endif

    virtual void receiveBegin();
    virtual void setMode(uint8_t mode);
    virtual void setHighPowerRegs(bool onOff);
    virtual void select();
    virtual void unselect();

#if defined(RF69_LISTENMODE_ENABLE)
  //=============================================================================
  //                     ListenMode specific declarations
  //=============================================================================
  public:
    // When we receive a packet in listen mode, this is the time left in the sender's burst.
    // You need to wait at least this long before trying to reply.
    static volatile uint16_t RF69_LISTEN_BURST_REMAINING_MS;

    void listenModeStart(void);
    void listenModeEnd(void);
    void listenModeHighSpeed(bool highSpeed) { _isHighSpeed = highSpeed; }

    // rx and idle duration in microseconds
    bool listenModeSetDurations(uint32_t& rxDuration, uint32_t& idleDuration);

    // The values passed to listenModeSetDurations() may be slightly different to accomodate
    // what is allowed by the radio. This function returns the actual values used.
    void listenModeGetDurations(uint32_t& rxDuration, uint32_t& idleDuration);

    // This repeatedly sends the message to the target node for the duration
    // of an entire listen cycle. The amount of time remaining in the burst
    // is transmitted to the receiver, and it is expected that the receiver
    // wait for the burst to end before attempting a reply.
    // See RF69_LISTEN_BURST_REMAINING_MS above.
    void listenModeSendBurst(uint8_t targetNode, void* buffer, uint8_t size);

  protected:
    void listenModeInterruptHandler(void);
    void listenModeApplyHighSpeedSettings();
    void listenModeReset(); //resets variables used on the receiving end
    bool reinitRadio(void);
    static void listenModeIrq();

    bool _isHighSpeed;
    bool _haveEncryptKey;
    char _encryptKey[RF69_ENCRYPTION_KEY_SIZE];

    // Save these so we can reinitialize the radio after sending a burst
    // or exiting listen mode.
    uint8_t _freqBand;
    uint8_t _networkID;
    uint8_t _rxListenCoef;
    uint8_t _rxListenResolution;
    uint8_t _idleListenCoef;
    uint8_t _idleListenResolution;
    uint32_t _listenCycleDurationUs;
#endif
};

#endif