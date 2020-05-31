#include "rx_bayang.h"

#include <stdio.h>

#include "binary.h"
#include "control.h"
#include "drv_spi.h"
#include "drv_spi_xn297.h"
#include "drv_time.h"
#include "project.h"
#include "util.h"

#ifdef RX_BAYANG_PROTOCOL

int rx_ready = 1;
int bind_safety = 0;

extern char lastaux[AUX_CHANNEL_MAX];
extern char auxchange[AUX_CHANNEL_MAX];

void writeregs(uint8_t data[], uint8_t size) {
  spi_cson();
  for (uint8_t i = 0; i < size; i++) {
    spi_sendbyte(data[i]);
  }
  spi_csoff();
  delay(1000);
}

void rx_init() {

  /*
uint8_t bbcal[6] = { 0x3f , 0x4c , 0x84 , 0x6F , 0x9c , 0x20  };
writeregs( bbcal , sizeof(bbcal) );


uint8_t rfcal[8] = { 0x3e , 0xc9 , 0x9a , 0x80 , 0x61 , 0xbb , 0xab , 0x9c  };
writeregs( rfcal , sizeof(rfcal) );


uint8_t demodcal[6] = { 0x39 , 0x0b , 0xdf , 0xc4 , 0xa7 , 0x03};
writeregs( demodcal , sizeof(demodcal) );
*/

  int rxaddress[5] = {0, 0, 0, 0, 0};
  xn_writerxaddress(rxaddress);

  xn_writereg(EN_AA, 0);            // aa disabled
  xn_writereg(EN_RXADDR, 1);        // pipe 0 only
  xn_writereg(RF_SETUP, B00000001); // lna high current on ( better performance )
  xn_writereg(RX_PW_P0, 15);        // payload size
  xn_writereg(SETUP_RETR, 0);       // no retransmissions ( redundant?)
  xn_writereg(SETUP_AW, 3);         // address size (5 bits)
  xn_writereg(29, 32);              // feture reg , CE mode (software controlled)

  spi_cson();
  spi_sendbyte(0xFD); // internal CE high command
  spi_sendbyte(0);    // required for above
  spi_csoff();

  xn_command(FLUSH_RX);
  xn_writereg(RF_CH, 0);     // bind on channel 0
  xn_writereg(0, B10001111); // power up, crc enabled
}

static char checkpacket() {
  int status = xn_readreg(7);
  if (status & (1 << MASK_RX_DR)) { // rx clear bit
                                    // this is not working well
                                    // xn_writereg( STATUS , (1<<MASK_RX_DR) );
                                    //RX packet received
                                    //return 1;
  }
  if ((status & B00001110) != B00001110) {
    // rx fifo not empty
    return 2;
  }

  return 0;
}

int rxdata[15];

#define CHANOFFSET 512

float packettodata(int *data) {
  return (((data[0] & 0x0003) * 256 + data[1]) - CHANOFFSET) * 0.001953125;
}

static int decodepacket(void) {
  if (rxdata[0] == 165) {
    int sum = 0;
    for (int i = 0; i < 14; i++) {
      sum += rxdata[i];
    }
    if ((sum & 0xFF) == rxdata[14]) {
      state.rx.axis[0] = packettodata(&rxdata[4]);
      state.rx.axis[1] = packettodata(&rxdata[6]);
      state.rx.axis[2] = packettodata(&rxdata[10]);
      // throttle
      state.rx.axis[3] = ((rxdata[8] & 0x0003) * 256 + rxdata[9]) * 0.000976562;

      // trims are 50% of controls at max
      // trims are not used because they interfere with dynamic trims feature of devo firmware

      //			state.rx.axis[0] = state.rx.axis[0] + 0.03225 * 0.5 * (float)(((rxdata[4])>>2) - 31);
      //			state.rx.axis[1] = state.rx.axis[1] + 0.03225 * 0.5 * (float)(((rxdata[6])>>2) - 31);
      //			state.rx.axis[2] = state.rx.axis[2] + 0.03225 * 0.5 * (float)(((rxdata[10])>>2) - 31);

      state.aux[CH_INV] = (rxdata[3] & 0x80) ? 1 : 0; // inverted flag

      state.aux[CH_VID] = (rxdata[2] & 0x10) ? 1 : 0;

      state.aux[CH_PIC] = (rxdata[2] & 0x20) ? 1 : 0;

      state.aux[CH_FLIP] = (rxdata[2] & 0x08) ? 1 : 0;

      //state.aux[CH_EXPERT] = (rxdata[1] == 0xfa) ? 1 : 0;

      state.aux[CH_HEADFREE] = (rxdata[2] & 0x02) ? 1 : 0;

      state.aux[CH_RTH] = (rxdata[2] & 0x01) ? 1 : 0; // rth channel

      rx_apply_expo();

      for (int i = 0; i < AUX_CHANNEL_MAX - 2; i++) {
        auxchange[i] = 0;
        if (lastaux[i] != state.aux[i])
          auxchange[i] = 1;
        lastaux[i] = state.aux[i];
      }

      return 1; // valid packet
    }
    return 0; // sum fail
  }
  return 0; // first byte different
}

char rfchannel[4];
int rxaddress[5];
int chan = 0;

void nextchannel() {
  chan++;
  if (chan > 3)
    chan = 0;
  xn_writereg(0x25, rfchannel[chan]);
}

unsigned long lastrxtime;
unsigned long failsafetime;
unsigned long secondtimer;

//#define RXDEBUG

#ifdef RXDEBUG
unsigned long packettime;
int channelcount[4];
int failcount;
int packetrx;
int packetpersecond;
int skipstats[12];
int afterskip[12];
//#warning "RX debug enabled"
#endif

unsigned int skipchannel = 0;
int lastrxchan;
int timingfail = 0;

// packet period in uS
#define PACKET_PERIOD 3300

// was 250 ( uS )
// sign changed
#define PACKET_OFFSET 500

// how many times to hop ahead if no reception
#define HOPPING_NUMBER 4

void rx_check(void) {
  int packetreceived = checkpacket();
  int pass = 0;
  if (packetreceived) {
    if (flags.rxmode == RXMODE_BIND) { // rx startup , bind mode
      xn_readpayload(rxdata, 15);

      if (rxdata[0] == 164) { // bind packet
        rfchannel[0] = rxdata[6];
        rfchannel[1] = rxdata[7];
        rfchannel[2] = rxdata[8];
        rfchannel[3] = rxdata[9];

        int rxaddress[5];
        rxaddress[0] = rxdata[1];
        rxaddress[1] = rxdata[2];
        rxaddress[2] = rxdata[3];
        rxaddress[3] = rxdata[4];
        rxaddress[4] = rxdata[5];

        xn_writerxaddress(rxaddress);
        xn_writereg(0x25, rfchannel[chan]); // Set channel frequency
        flags.rxmode = RXMODE_NORMAL;

#ifdef SERIAL
        printf(" BIND \n");
#endif
      }
    } else { // normal mode
#ifdef RXDEBUG
      channelcount[chan]++;
      packettime = gettime() - lastrxtime;

      if (skipchannel && !timingfail)
        afterskip[skipchannel]++;
      if (timingfail)
        afterskip[0]++;

#endif

      unsigned long temptime = gettime();

      nextchannel();

      xn_readpayload(rxdata, 15);
      pass = decodepacket();

      if (pass) {
#ifdef RXDEBUG
        packetrx++;
#endif
        skipchannel = 0;
        timingfail = 0;
        lastrxchan = chan;
        lastrxtime = temptime;
        failsafetime = temptime;
        flags.failsafe = 0;
      } else {
#ifdef RXDEBUG
        failcount++;
#endif
      }
      bind_safety++;
      if (bind_safety > 9) { //requires 10 good frames to come in before rx_ready safety can be toggled to 1
        rx_ready = 1;        // because aux channels initialize low and clear the binding while armed flag before aux updates high
        bind_safety = 10;
      }
    } // end normal rx mode

  } // end packet received

  unsigned long time = gettime();

  // sequence period 12000
  if (time - lastrxtime > (HOPPING_NUMBER * PACKET_PERIOD + 1000) && flags.rxmode != RXMODE_BIND) {
    //  channel with no reception
    lastrxtime = time;
    // set channel to last with reception
    if (!timingfail)
      chan = lastrxchan;
    // advance to next channel
    nextchannel();
    // set flag to discard packet timing
    timingfail = 1;
  }

  if (!timingfail && skipchannel < HOPPING_NUMBER + 1 && flags.rxmode != RXMODE_BIND) {
    unsigned int temp = time - lastrxtime;

    if (temp > 1000 && (temp + (PACKET_OFFSET)) / ((int)PACKET_PERIOD) >= (skipchannel + 1)) {
      nextchannel();
#ifdef RXDEBUG
      skipstats[skipchannel]++;
#endif
      skipchannel++;
    }
  }

  if (time - failsafetime > FAILSAFETIME) { //  failsafe
    flags.failsafe = 1;
    state.rx.axis[0] = 0;
    state.rx.axis[1] = 0;
    state.rx.axis[2] = 0;
    state.rx.axis[3] = 0;
  }
#ifdef RXDEBUG
  if (gettime() - secondtimer > 1000000) {
    packetpersecond = packetrx;
    packetrx = 0;
    secondtimer = gettime();
  }
#endif
}

#endif
