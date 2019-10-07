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

#include <Arduino.h>
#include "ASIXEthernet.h"

#define print   USBHost::print_
#define println USBHost::println_

void ASIXEthernet::init() {
    contribute_Pipes(mypipes, sizeof(mypipes)/sizeof(Pipe_t));
    contribute_Transfers(mytransfers, sizeof(mytransfers)/sizeof(Transfer_t));
    contribute_String_Buffers(mystring_bufs, sizeof(mystring_bufs)/sizeof(strbuf_t));
    handleRecieve = NULL;
    initialized = false;
    connected = false;
    driver_ready_for_device(this);
}

bool ASIXEthernet::claim(Device_t *dev, int type, const uint8_t *descriptors, uint32_t len) {
    
    const uint8_t *p = descriptors;
    const uint8_t *end = p + len;
    
    if(type != 1) return false;
    if(dev->idVendor != 0x0B95) return false;
    println("ASIXEthernet claim this=", (uint32_t)this, HEX);
    println("type=", type);
    print("vid=", dev->idVendor, HEX);
    print(", pid=", dev->idProduct, HEX);
    print(", bDeviceClass = ", dev->bDeviceClass);
    print(", bDeviceSubClass = ", dev->bDeviceSubClass);
    println(", bDeviceProtocol = ", dev->bDeviceProtocol);
    
    
    uint8_t length = p[0];
    uint8_t descriptorType = p[1];
    uint8_t descriptorClass = p[5];
    uint8_t descriptorSubclass = p[6];
    uint8_t epType;
    uint8_t epAddress;
    uint16_t epSize;
    uint8_t epInterval;
    
    if(length != 9 || descriptorType != 4) return false; //Interface desciptor
    uint8_t numEndpoints = p[4];
    println("numEndpoints=", numEndpoints, DEC);
    if(descriptorClass != 255 || descriptorSubclass != 255) return false; //bInterfaceClass/Subclass
    p += length;
    interrupt_ep = 0;
    rx_ep = 0;
    tx_ep = 0;
    interrupt_size = 0;
    rx_size = 0;
    tx_size = 0;
    while (p < end) {
        length = p[0];
        if (p + length > end) return false; // reject if beyond end of data
        descriptorType = p[1];
        if(descriptorType == 5) {  //Endpoint
            epType = p[3];
            epAddress = p[2];
            epSize = p[4] | (p[5] << 8);
            epInterval = p[6];
            switch (epType) {
                case 2: //Bulk Endpoint
                    switch (epAddress & 0xF0) {
                        case 0x80: //IN Endpoint
                            rx_ep = epAddress;// & 0x0F;
                            rx_size = epSize;
                            rx_interval = epInterval;
                            println("      rx_size = ", rx_size);
                            println("      rx_ep = ", rx_ep);
                            println("      rx_interval = ", rx_interval);
                            break;
                        case 0x00: //OutEndpoint
                            tx_ep = epAddress;
                            tx_size = epSize;
                            tx_interval = epInterval;
                            println("      tx_size = ", tx_size);
                            println("      tx_ep = ", tx_ep);
                            println("      tx_interval = ", tx_interval);
                            break;
                        default:
                            break;
                    }
                    break;
                case 3: //Interrupt Endpoint
                    switch (epAddress & 0xF0) {
                        case 0x80: //IN Endpoint
                            interrupt_ep = epAddress;// & 0x0F;
                            interrupt_size = epSize;
                            interrupt_interval = epInterval;
                            println("      interrupt_size = ", interrupt_size);
                            println("      interrupt_ep = ", interrupt_ep);
                            println("      interrupt_interval = ", interrupt_interval);
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
        }
        p += length;
    }
    
    // if an IN endpoint was found, create its pipe
    if (rx_ep && rx_size <= 512) {
        rxpipe = new_Pipe(dev, 2, rx_ep, 1, rx_size, rx_interval);
        if (rxpipe) {
            rxpipe->callback_function = rx_callback;
            
            rx_buffer = (uint8_t*)rx_buffer0 + (current_rx_buffer * transferSize);
            if(current_rx_buffer == (num_rx_buffers - 1)) current_rx_buffer = 0;
            else current_rx_buffer++;
            
            queue_Data_Transfer(rxpipe, rx_buffer, transferSize, this);
            rx_packet_queued++;
        }
    } else {
        rxpipe = NULL;
    }
    // if an OUT endpoint was found, create its pipe
    if (tx_ep && tx_size <= 512) {
        txpipe = new_Pipe(dev, 2, tx_ep, 0, tx_size, tx_interval);
        if (txpipe) {
            txpipe->callback_function = tx_callback;
        }
    } else {
        txpipe = NULL;
    }
    // if an IN endpoint was found, create its pipe
    if (interrupt_ep && interrupt_size <= 512) {
        interruptpipe = new_Pipe(dev, 3, interrupt_ep, 1, interrupt_size, interrupt_interval);
        if (interruptpipe) {
            interruptpipe->callback_function = interrupt_callback;
            queue_Data_Transfer(interruptpipe, interrupt_buffer, interrupt_size, this);
            interrupt_packet_queued = true;
        }
    } else {
        interruptpipe = NULL;
    }
    
    println("Control - ASIX...");
    mk_setup(setup, 0xc0, 11, 0x0004, 0, 2);
    queue_Control_Transfer(dev, &setup, nodeID, this);
    control_queued = true;
    pending_control = 1;
    return (rxpipe || txpipe || interruptpipe);
}

void ASIXEthernet::control(const Transfer_t *transfer) {
    println("control callback (asix) ", pending_control, DEC);
    control_queued = false;
pending:
    switch (pending_control) { //This order was derived from how MacOS sets this up
        case 1:                                                         //Get Mac Bytes 2-3
            mk_setup(setup, 0xc0, 11, 0x0005, 0, 2);
            queue_Control_Transfer(device, &setup, nodeID + 2, this);
            control_queued = true;
            pending_control = 2;
            break;
        case 2:                                                         //Get Mac Bytes 4-5
            mk_setup(setup, 0xc0, 11, 0x0006, 0, 2);
            queue_Control_Transfer(device, &setup, nodeID + 4, this);
            control_queued = true;
            pending_control = 3;
            break;
        case 3:        //Probably not needed                            //Get Unknown Bytes 0xFFFF
            print("nodeID: ");
            print_hexbytes(nodeID, 6);
            
            mk_setup(setup, 0xc0, 11, 0x0017, 0, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 4;
            return;
        case 4:                                                         //Read Station Management Status Register    Determine Owner 0 010(chip code) 0 0 0 0
            mk_setup(setup, 0xc0, 9, 0x0000, 0, 1);
            queue_Control_Transfer(device, &setup, owner, this);
            control_queued = true;
            pending_control = 5;
            break;
        case 5:                                                         //Write to IPG/IPG1/IPG2 Register
            mk_setup(setup, 0x40, 18, 0x0C15, 14, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 6;
            break;
        case 6:                                                         //Read IPG/IPG1/IPG2 Register to verify write
            mk_setup(setup, 0xC0, 17, 0x0000, 0, 3);
            queue_Control_Transfer(device, &setup, verify, this);
            control_queued = true;
            pending_control = 7;
            break;
        case 7:                                                         //Write to COE RX Control Register    Setup receive checks and packet drops
            if(verify[0] != 0x15 && verify[1] != 0x0C && verify[2] != 0x0E) {
                print("verify: ");
                print_hexbytes(verify, 3);
                pending_control = 5;
                goto pending;
            }
            mk_setup(setup, 0x40, 44, 0x007B, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 8;
            break;
        case 8:                                                         //Write to COE TX Control Register    Setup transmit checksum insertion
            mk_setup(setup, 0x40, 46, 0x003F, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 9;
            break;
        case 9:                                                         //Read Software Interface Selection Status Register    Get current setup
            mk_setup(setup, 0xC0, 33, 0x0000, 0, 1);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 10;
            break;
        case 10:                                                        //Write Software Interface Selection Register    Setup since default    0x01 Ethernet PHY
            mk_setup(setup, 0x40, 34, 0x0001, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 11;
            break;
        case 11:                                                        //Read Software Interface Selection Status Register    Verify setup
            mk_setup(setup, 0xC0, 33, 0x0000, 0, 1);
            queue_Control_Transfer(device, &setup, verify, this);
            control_queued = true;
            pending_control = 12;
            break;
        case 12:                                                        //Ethernet/HomePNA PHY Address Register    PHY address from ROM 11h    0xE010 (default)
            //The value read here should technically be used later on when
            //reading/writing the PHY, but is currently not being used
            if(verify[0] != 0x01) {
                pending_control = 10;
                goto pending;
            }
            mk_setup(setup, 0xC0, 25, 0x0000, 0, 2);
            queue_Control_Transfer(device, &setup, PHYAddressReg, this);
            control_queued = true;
            pending_control = 13;
            break;
        case 13:                                                        //Write GPIOs Register
            mk_setup(setup, 0x40, 31, 0x00B0, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 14;
            break;
        case 14:                                                        //Write Power And Reset Register        20 00 Internal PHY Reset Control
            mk_setup(setup, 0x40, 32, 0x0020, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 15;
            break;
        case 15:                                                        //Write Power And Reset Register        60 00 Internal PHY Reset Control & Power Down Control
            mk_setup(setup, 0x40, 32, 0x0060, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 16;
            break;
        case 16:                                                        //Write Power And Reset Register        20 00 Internal PHY Reset Control
            mk_setup(setup, 0x40, 32, 0x0020, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 17;
            break;
        case 17:                                                        //Write Power And Reset Register        00 00
            mk_setup(setup, 0x40, 32, 0x0000, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 18;
            break;
        case 18:                                                        //Write Power And Reset Register        20 00 Internal PHY Reset Control
            mk_setup(setup, 0x40, 32, 0x0020, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 19;
            break;
        case 19:                                                        //Write Rx Control Register        00 00 All disabled
            mk_setup(setup, 0x40, 16, 0x0000, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 20;
            break;
        case 20:                                                        //Read Node ID Register            6 bytes 00 50 b6 be 8b b4 MAC address, also read earlier
            mk_setup(setup, 0xC0, 19, 0x0000, 0, 6);
            queue_Control_Transfer(device, &setup, nodeID, this);
            control_queued = true;
            pending_control = 21;
            break;
        case 21:                                                        //Write Jam Limit Count Register        3Fh (default)
            mk_setup(setup, 0x40, 38, 0x003F, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 22;
            break;
        case 22:                                                        //Read Monitor Mode Status Register        72
            mk_setup(setup, 0xC0, 28, 0x0000, 0, 1);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 23;
            break;
        case 23:                                                        //Write Software Station Management Control Register    Request Ownership
            mk_setup(setup, 0x40, 6, 0x0000, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 24;
            break;
        case 24:                                                        //Read Station Management Status Register        Check Ownership     0 010(chip) 0 0 0 1(Owner)
            mk_setup(setup, 0xC0, 9, 0x0000, 0, 1);
            queue_Control_Transfer(device, &setup, verify, this);
            control_queued = true;
            pending_control = 25;
            break;
        case 25:                                                        //Read PHY Register  02h            PHY Id Reg 1 003Bh (default)  OUI MSB
            if((verify[0] & 1) != 1){
                pending_control = 23;
                goto pending;
            }
            mk_setup(setup, 0xC0, 7, 0x0010, 2, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 26;
            break;
        case 26:                                                        //Read Station Management Status Register        Check Ownership     0 010(chip) 0 0 0 1(Owner)
            mk_setup(setup, 0xC0, 9, 0x0000, 0, 1);
            queue_Control_Transfer(device, &setup, verify, this);
            control_queued = true;
            pending_control = 27;
            break;
        case 27:                                                        //Read PHY Register  01h        Basic Mode Ctr Reg 3100h (default) AutoNeg Full Duplex
            mk_setup(setup, 0xC0, 7, 0x0010, 0, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 28;
            break;
        case 28:                                                        //Read PHY Register  01h        Basic Mode Ctr Reg 3100h (default) AutoNeg Full Duplex
            mk_setup(setup, 0xC0, 7, 0x0010, 0, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 29;
            break;
        case 29:                                                        //Read PHY Register  04h    Auto Neg Ad Reg 05E1h (01E1h) 0 0 0 00 1 0 1 1 1 1 00001 Duplex Pause
            mk_setup(setup, 0xC0, 7, 0x0010, 4, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 30;
            break;
        case 30:{                                                        //Write PHY Register 04h    Auto Neg Ad Reg 05E1h (01E1h) 0 0 0 00 1 0 1 1 1 1 00001 Duplex Pause
            mk_setup(setup, 0x40, 8, 0x0010, 4, 2);
            uint8_t xfr[2] = {0xE1, 0x05};
            queue_Control_Transfer(device, &setup, xfr, this);
            control_queued = true;
            pending_control = 31;
            break;}
        case 31:{                                                        //Write PHY Register 01h        Basic Mode Ctr Reg 3300h (3100h) Reset AutoNeg Full Duplex
            mk_setup(setup, 0x40, 8, 0x0010, 0, 2);
            uint8_t xfr[2] = {0x00, 0x33};
            queue_Control_Transfer(device, &setup, xfr, this);
            control_queued = true;
            pending_control = 32;
            break;}
        case 32:                                                        //Read PHY Register 01h        Basic Mode Ctr Reg 3100h  (3100h) AutoNeg Full Duplex
            mk_setup(setup, 0xC0, 7, 0x0010, 0, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 33;
            break;
        case 33:                                                        //Read PHY Register        Unknown 22 86
            mk_setup(setup, 0xC0, 7, 0x0010, 18, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 34;
            break;
        case 34:{                                                        //Write PHY Register        Unknown 2f 86    Reset?
            mk_setup(setup, 0x40, 8, 0x0010, 18, 2);
            uint8_t xfr[2] = {0x2F, 0x86};
            queue_Control_Transfer(device, &setup, xfr, this);
            control_queued = true;
            pending_control = 35;
            break;}
        case 35:                                                        //Read PHY Register        Unknown 22 86
            mk_setup(setup, 0xC0, 7, 0x0010, 18, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 36;
            break;
        case 36:                                                        //Write Medium Mode Register    36 03  0011 0110  0000 0011 Enable F Duplex & Flow Control
            mk_setup(setup, 0x40, 27, 0x0336, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 37;
            break;
        case 37:                                                        //Write Hdwr Stn Mngmnt Ctr Reg    Release Ownership
            mk_setup(setup, 0x40, 10, 0x0000, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 38;
            break;
        case 38:                                                        //Write to IPG/IPG1/IPG2 Register    15 16 1a
            mk_setup(setup, 0x40, 18, 0x1615, 26, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 39;
            break;
        case 39:                                                        //Read SROM Register        18h PHY Power Saving Config & checksum c0 09
            mk_setup(setup, 0xC0, 11, 0x0018, 0, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 40;
            break;
        case 40:                                                        //Write Power And Reset Register        20 09 Intrnl PHY Rst Ctr & Cbl pwr sav Hdwr, sav lvl 1
            mk_setup(setup, 0x40, 32, 0x0920, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 41;
            break;
        case 41:                                                        //Write transfer size
            //0x8000, 0x8001 2k buffers
            //0x8300, 0x83D7 8k buffers
            //0x8400, 0x851E 16k buffers
            //0x8600, 0x87AE 24k buffers
            //0x8700, 0x8A3D 32k buffers
            mk_setup(setup, 0x40, 42, 0x8000, 0x8001, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 42;
            break;
        case 42:                                                        //Write Rx Control Register    88 03  Strt Op, BCast, RX Hdr Mode
            mk_setup(setup, 0x40, 16, 0x0338, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 43;
            break;
        case 43:                                                        //Write Rx Control Register    98 03  Strt Op, MCast, BCast, RX Hdr Mode
            mk_setup(setup, 0x40, 16, 0x0398, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 44;
            break;
        case 44:{                                                        //Write Multicast Filter Array Register 00 00 00 00  00 00 00 00
            mk_setup(setup, 0x40, 22, 0x0000, 0, 8);
            uint8_t xfr[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
            queue_Control_Transfer(device, &setup, xfr, this);
            control_queued = true;
            pending_control = 45;
            break;}
        case 45:                                                        //Write Rx Control Register    88 03  Strt Op, BCast, RX Hdr Mode
            mk_setup(setup, 0x40, 16, 0x0388, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 46;
            break;
        case 46:                                                        //Write Rx Control Register    98 03  Strt Op, MCast, BCast, RX Hdr Mode
            mk_setup(setup, 0x40, 16, 0x03D8, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 47;
            break;
        case 47:{                                                        //Write Multicast Filter Array Register 00 40 00 00  00 00 00 00
            mk_setup(setup, 0x40, 22, 0x0000, 0, 8);
            uint8_t xfr[8] = {0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00};
            queue_Control_Transfer(device, &setup, xfr, this);
            control_queued = true;
            pending_control = 255;                      //Starts searching for network
            break;}
        case 48:                                                        //Write Software Station Management Control Register    Request Ownership
            mk_setup(setup, 0x40, 6, 0x0000, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 49;
            return;
        case 49:                                                        //Read Station Management Status Register        Check Ownership     0 010(chip) 0 0 0 1(Owner)
            mk_setup(setup, 0xC0, 9, 0x0000, 0, 1);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 50;
            break;
        case 50:                                                        //Read PHY Register 00h        Basic Mode Ctr Reg 3100h (default) AutoNeg Full Duplex
            mk_setup(setup, 0xC0, 7, 0x0010, 0, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 51;
            break;
        case 51:                                                        //Read PHY Register 01h    Bsc mode stat reg 2D 78 Duplex Capable, auto neg done, Linked, extended
            mk_setup(setup, 0xC0, 7, 0x0010, 1, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 52;
            break;
        case 52:                                                        //Write Hdwr Stn Mngmnt Ctr Reg    Release Ownership
            mk_setup(setup, 0x40, 10, 0x0000, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 53;
            break;
        case 53:                                                        //Write Software Station Management Control Register    Request Ownership
            mk_setup(setup, 0x40, 6, 0x0000, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 54;
            break;
        case 54:                                                        //Read Station Management Status Register        Check Ownership     0 010(chip) 0 0 0 1(Owner)
            mk_setup(setup, 0xC0, 9, 0x0000, 0, 1);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 55;
            break;
        case 55:                                                        //Read PHY Register 00h        Basic Mode Ctr Reg 3100h (default) AutoNeg Full Duplex
            mk_setup(setup, 0xC0, 7, 0x0010, 0, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 56;
            break;
        case 56:                                                        //Read PHY Register 01h    Bsc mode stat reg 2D 78 Duplex Capable, auto neg done, Linked, extended
            mk_setup(setup, 0xC0, 7, 0x0010, 1, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 57;
            break;
        case 57:                                                        //Read PHY Register 02h        PHY Id Reg 1 003Bh (default)  OUI MSB
            mk_setup(setup, 0xC0, 7, 0x0010, 2, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 58;
            break;
        case 58:                                                        //Read PHY Register 03h        PHY Id Reg 2 1881h (default)  OUI LSB
            mk_setup(setup, 0xC0, 7, 0x0010, 3, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 59;
            break;
        case 59:                                                        //Read PHY Register 04h    Auto Neg Ad Reg 05E1h (01E1h) 0 0 0 00 1 0 1 1 1 1 00001 Duplex Pause
            mk_setup(setup, 0xC0, 7, 0x0010, 4, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 60;
            break;
        case 60:                                                        //Read PHY Register 05h    Auto neg link prtnr abl reg C101h (0000h) Duplex modes, ptcl sel bits
            mk_setup(setup, 0xC0, 7, 0x0010, 5, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 61;
            break;
        case 61:                                                        //Read PHY Register 06h    Auto neg expnsn reg 000Bh (0000h) page en, new page, auto neg acpt
            mk_setup(setup, 0xC0, 7, 0x0010, 6, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 62;
            break;
        case 62:                                                        //Write Hdwr Stn Mngmnt Ctr Reg    Release Ownership
            mk_setup(setup, 0x40, 10, 0x0000, 0, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 63;
            break;
        case 63:                                                        //Read Medium Status Register    36 03 0011 0110  0000 0011 F Duplex & Flow Control Enabled
            mk_setup(setup, 0xC0, 26, 0x0000, 0, 2);
            queue_Control_Transfer(device, &setup, setupdata, this);
            control_queued = true;
            pending_control = 64;
            break;
        case 64:                                                        //Write transfer size
            //0x8000, 0x8001 2k buffers
            //0x8300, 0x83D7 8k buffers
            //0x8400, 0x851E 16k buffers
            //0x8600, 0x87AE 24k buffers
            //0x8700, 0x8A3D 32k buffers
            mk_setup(setup, 0x40, 42, 0x8000, 0x8001, 0);
            queue_Control_Transfer(device, &setup, NULL, this);
            control_queued = true;
            pending_control = 65;
            break;
            
        case 65:{                                                       //This sets the multicast address filter array bitmap, I haven't been able to figure out how this is supposed to be calculated. Their manual doesn't specify too well and all my efforts have been deadends. Setting all these to 0xFF lets all multicast messages through lowering bandwidth for other messages.
            mk_setup(setup, 0x40, 22, 0x0000, 0, 8);
//            uint8_t xfr[8] = {B01000000,B10000000,0x20,0x40,B10000000,0x00,0x80,B00010000};
            uint8_t xfr[8] = {0,0,0,0,0,0,0,0};
            queue_Control_Transfer(device, &setup, xfr, this);
            control_queued = true;
            pending_control = 254;
            initialized = true;
            connected = true;
            break;}
        default:
            return;
            //    }
    }
        println("Done");
}

void ASIXEthernet::rx_callback(const Transfer_t *transfer) {
//    println("rx_callback(asix)");
    if (transfer->driver) {
//        print("transfer->qtd.token = ");
//        println(transfer->qtd.token & 255);
        ((ASIXEthernet *)(transfer->driver))->rx_data(transfer);
    }
}

void ASIXEthernet::tx_callback(const Transfer_t *transfer) {
//    println("tx_callback(asix)");
    if (transfer->driver) {
//        print("transfer->qtd.token = ");
//        println(transfer->qtd.token & 255);
        ((ASIXEthernet *)(transfer->driver))->tx_data(transfer);
    }
}

void ASIXEthernet::interrupt_callback(const Transfer_t *transfer) {
//    println("interrupt_callback(asix)");
    if (transfer->driver) {
//        print("transfer->qtd.token = ");
//        println(transfer->qtd.token & 255);
        ((ASIXEthernet *)(transfer->driver))->interrupt_data(transfer);
    }
}

void ASIXEthernet::disconnect() {
    rxpipe = NULL;
    txpipe = NULL;
    interruptpipe = NULL;
    connected = 0;
    println("Device Disconnected...");
}

void ASIXEthernet::rx_data(const Transfer_t *transfer) {
    //Current header format is: bytes 0-1 = Packet Length LSB-MSB
    //Current header format is: bytes 2-3 = One's Complement Packet Length LSB-MSB
    //Current header format is: bytes 4-5 = Packet Type information and checksum error detected
    //Current header format is: bytes 6-(length + 6) is ethernet packet
    //Current header format is: bytes (length + 7)-end ie last 3 bytes is unknown possible crc
    uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);
    if(len > 1000) println("rx_data(asix): ", len, DEC);
//    print_hexbytes((uint8_t*)transfer->buffer, len);
//    println("queue another receive packet");
    rx_buffer = (uint8_t*)rx_buffer0 + (current_rx_buffer * transferSize);
    if(current_rx_buffer == (num_rx_buffers - 1)) current_rx_buffer = 0;
    else current_rx_buffer++;
    
    rx_packet_queued--;
    queue_Data_Transfer(rxpipe, rx_buffer, transferSize, this);
    rx_packet_queued++;
    
    (*handleRecieve)((uint8_t*)transfer->buffer, len);
}

void ASIXEthernet::tx_data(const Transfer_t *transfer) {
    uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);
    if(len > 1000) println("tx_data(asix): ", len, DEC);
//    print_hexbytes((uint8_t*)transfer->buffer, len);
    tx_packet_queued--;
}

void ASIXEthernet::interrupt_data(const Transfer_t *transfer) {
//    uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);
    const uint8_t *p = (const uint8_t *)transfer->buffer;
    PHYSpeed = (p[2] & 0x10) ? 1 : 0;
    if(((p[2] & 0x1) ? 1 : 0) == 1 && pending_control == 255) {
        pending_control = 48;
        mk_setup(setup, 0x40, 6, 0x0000, 0, 0);
        queue_Control_Transfer(device, &setup, NULL, this);
        control_queued = true;
        pending_control = 49;
    }
    else if(((p[2] & 0x1) ? 1 : 0) == 0 && pending_control == 254) {
        pending_control = 255;
        connected = false;
    }
//    println("interrupt_data(asix): ", len, DEC);
//    print_hexbytes((uint8_t*)transfer->buffer, (len < 32)? len : 32 );
    queue_Data_Transfer(interruptpipe, interrupt_buffer, interrupt_size, this);
}

bool ASIXEthernet::read() {
    if(!rxpipe) return false;
    if(pending_control != 254) return false;
    if (!rx_packet_queued && rxpipe) {
        NVIC_DISABLE_IRQ(IRQ_USBHS);
        
        rx_buffer = (uint8_t*)rx_buffer0 + (current_rx_buffer * transferSize);
        if(current_rx_buffer == (num_rx_buffers - 1)) current_rx_buffer = 0;
        else current_rx_buffer++;
        
        queue_Data_Transfer(rxpipe, rx_buffer, transferSize, this);
        NVIC_ENABLE_IRQ(IRQ_USBHS);
        rx_packet_queued++;
    }
    return true;
}

void ASIXEthernet::sendPacket(const uint8_t *data, uint32_t length) {
    if (!txpipe) return;
    if(pending_control != 254) return;
    
    tx_buffer = (uint8_t*)tx_buffer0 + (current_tx_buffer * transmitSize);
    if(current_tx_buffer == (num_tx_buffers - 1)) current_tx_buffer = 0;
    else current_tx_buffer++;

    //Insert USB Header to data message
    //This is the default format and the most basic
    //it can be changed to an alternate format
    //but this is the simplest one that works fine
    tx_buffer[0] = length & 0x00FF;     //Length of packet LSB
    tx_buffer[1] = (length >> 8) & 0x7; //Length of packet MSB
    tx_buffer[2] = ~length & 0x00FF;                //One's complement Length of packet LSB
    tx_buffer[3] = 0xF0 | ((~length >> 8) & 0x7);   //One's complement Length of packet MSB
    
    for(uint16_t i = 0; i < length; i++) {
        tx_buffer[i + 4] = *data++;
    }
    if(length < 64) {   //Add padding bytes for small messages
        for(uint16_t i = length + 4; i < 64 + 4; i++) {
            tx_buffer[i] = 0;
        }
        length = 64;
    }
    length += 4; //Add header size
    uint16_t _index = 0;
    while(length > transmitSize) { //Send chunks if large message
        queue_Data_Transfer(txpipe, tx_buffer + _index, transmitSize, this);
        tx_packet_queued++;
        length -= transmitSize;
        _index += transmitSize;
    }
    if(length){
        queue_Data_Transfer(txpipe, tx_buffer + _index, length, this);
        tx_packet_queued++;
    }
}

void ASIXEthernet::readPHY(uint32_t address, uint16_t *data) {
    mk_setup(setup, 0xc0, 7, 0x0010, address, 2);
    queue_Control_Transfer(device, &setup, (uint8_t*)data, this);
    control_queued = true;
}

void ASIXEthernet::writePHY(uint32_t address, uint16_t data) {
    mk_setup(setup, 0x40, 8, 0x0010, address, 2);
    uint8_t xfr[2] = {(uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF)};
    queue_Control_Transfer(device, &setup, xfr, this);
    control_queued = true;
}

void ASIXEthernet::setMulticast(uint8_t *hashTable) {
    mk_setup(setup, 0x40, 22, 0x0000, 0, 8);
    queue_Control_Transfer(device, &setup, hashTable, this);
    control_queued = true;
}
