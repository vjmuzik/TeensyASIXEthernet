/* USB ASIXEthernet driver for Teensy 3.6/4.0
 * Copyright 2019 vjmuzik (vjmuzik1@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef ASIXEthernet_h
#define ASIXEthernet_h

#include "USBHost_t36.h"

//--------------------------------------------------------------------------
class ASIXEthernet : public USBDriver {
public:
    ASIXEthernet(USBHost &host) { init(); }
    ASIXEthernet(USBHost *host) { init(); }
    bool read();
    void sendPacket(const uint8_t* data, uint32_t length);
    void setHandleRecieve(void (*fptr)(const uint8_t* data, uint32_t length)) {
        handleRecieve = fptr;
    }
    void setPacketTypePromiscuous() {
        PACKET_TYPE_PROMISCUOUS = true;
    }
    void setHandleWait(void (*fptr)()) {
        handleWait = fptr;
    }
    void readPHY(uint32_t address, uint16_t *data);
    void writePHY(uint32_t address, uint16_t data);
    void setMulticast(uint8_t *hashTable);
    uint8_t nodeID[6]; //Also known as MAC address
    uint8_t txQueued() {return tx_packet_queued;}
    volatile bool initialized;
    volatile bool connected;
    volatile bool PHYSpeed;
protected:
    virtual bool claim(Device_t *device, int type, const uint8_t *descriptors, uint32_t len);
    virtual void control(const Transfer_t *transfer);
    virtual void disconnect();
    static void rx_callback(const Transfer_t *transfer);
    static void tx_callback(const Transfer_t *transfer);
    static void interrupt_callback(const Transfer_t *transfer);
    void rx_data(const Transfer_t *transfer);
    void tx_data(const Transfer_t *transfer);
    void interrupt_data(const Transfer_t *transfer);
    void init();
private:
    
    bool PACKET_TYPE_PROMISCUOUS = false;
    
    uint32_t rx_size;
    uint32_t tx_size;
    uint32_t interrupt_size;
    Pipe_t *rxpipe;
    Pipe_t *txpipe;
    Pipe_t *interruptpipe;
    uint32_t rx_ep = 0;
    uint32_t tx_ep = 0;
    uint32_t interrupt_ep = 0;
    uint16_t rx_interval = 0;
    uint16_t tx_interval = 0;
    uint16_t interrupt_interval = 0;
    uint8_t rx_packet_queued;
    uint8_t tx_packet_queued;
    bool interrupt_packet_queued;
    bool control_queued;
    uint8_t pending_control;
    
    uint8_t owner[2];
    uint8_t verify[8];
    uint8_t interface;
    uint8_t PHYAddressReg[2] = {0xE0, 0x10};
    
    setup_t setup;
    uint8_t setupdata[16];
    static const uint32_t transferSize = 1024 * 16; //Change recieve buffer size
    static const uint32_t transmitSize = 1518;  //Change transmit buffer size
                                       //1518 is the max size message that can be sent
    
    volatile uint8_t current_rx_buffer = 0;
    static const uint8_t num_rx_buffers = 1; //Number of recieve buffers
    uint8_t* rx_buffer;
    volatile uint8_t rx_buffer0[transferSize * num_rx_buffers];
    
    volatile uint8_t current_tx_buffer = 0;
    static const uint8_t num_tx_buffers = 32; //Number of transmit buffers
    uint8_t* tx_buffer;
    volatile uint8_t tx_buffer0[transmitSize * num_tx_buffers];
    
    uint8_t interrupt_buffer[8];
    
    Pipe_t mypipes[4] __attribute__ ((aligned(32)));
    Transfer_t mytransfers[134] __attribute__ ((aligned(32)));
    strbuf_t mystring_bufs[1];
    void (*handleRecieve)(const uint8_t *data, uint32_t length);
    void (*handleWait)();
};

#endif /* ASIXEthernet_h */
