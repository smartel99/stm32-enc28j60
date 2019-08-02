#include "enc28j60.h"

extern void Error_Handler(void);
static uint8_t Enc28j60Bank;
static uint16_t gNextPacketPtr;
static uint8_t erxfcon;
static SPI_HandleTypeDef *hspi = NULL;


void enc28j60_set_spi(SPI_HandleTypeDef *hspi_new){
	hspi = hspi_new;
}


void error(float error_num, char infinite);
uint8_t enc28j60_SendByte(uint8_t tx){
	if(hspi == NULL)
		return 0;
	
	uint8_t rx = 0;
	int r;
	
	r = HAL_SPI_TransmitReceive(hspi, &tx, &rx, 1, 0xFFFFFFFF);
	
	if(r != HAL_OK)
		Error_Handler();
	
	return rx;
}


uint8_t enc28j60ReadOp(uint8_t op, uint8_t address){
	uint8_t temp;
	enableChip;
	enc28j60_SendByte(op | (address & ADDR_MASK));
	enc28j60_SendByte(data);
	disableChip;
}


void enc28j60PowerDown() {
	enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXEN);
	while(enc28j60Read(ESTAT)&ESTAT_RXBUSY);
	while(enc28j60Read(ECON1)&ECON1_TXRTS);
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PWRSV);
}


void enc28j60PowerUp(){
	enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON2, ECON2_PWRSV);
	while(!enc28j60Read(ESTAT)&ESTAT_CLKRDY);
}


void enc28j60ReadBuffer(uint16_t len, uint8_t* data){
	enableChip;
	enc28j60_SendByte(ENC28J60_READ_BUF_MEM);
	while(len--){
		*data++ = enc28j60_SendByte(0x00);
	}
	disableChip;
}


static uint16_t enc28j60ReadBufferWord(){
	uint16_t result;
	enc28j60ReadBuffer(2, (uint8_t*)&result);
	return result;
}


void enc28j60WriteBuffer(uint16_t len, uint8_t* data){
	enableChip;
	enc28j60_SendByte(ENC28J60_WRITE_BUF_MEM);
	while(len--)
		enc28j60_SendByte(*data++);
	
	disableChip;
}


void enc28j60SetBank(uint8_t address){
	if((address & BANK_MASK) != Enc28j60Bank){
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL1|ECON1_BSEL0);
		Enc28j60Bank = address&BANK_MASK;
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, Enc28j60Bank>>5);
	}
}


uint8_t enc28j60Read(uint8_t address){
	// Set the bank.
	enc28j60SetBank(address);
	// Do the read.
	return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
}


void enc28j60WriteWord(uint8_t address, uint16_t data){
	enc28j60Write(address, data&0xFF);
	enc28j60Write(address+1, data>>8);
}


// Read upper 8bits.
uint16_t enc28j60PhyReadH(uint8_t address){
	// Set the right address and start the register read operation.
	enc28j60Write(MIREGADR, address);
	enc28j60Write(MICMD, MICMD_MIIRD);
	Delay(15);
	
	// Wait until the PHY read completes.
	while(enc28j60Read(MISTAT)&MISTAT_BUSY);
	
	// Reset the reading bit.
	enc28j60Write(MICMD, 0x00);
	
	return (enc28j60Read(MIRDH));
}


void enc28j60Write(uint8_t address, uint8_t data){
	// Set the bank.
	enc28j60SetBank(address);
	// Do the write.
	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}


void enc28j60PhyWrite(uint8_t address, uint16_t data){
	// Set the PHY register address.
	enc28j60Write(MIREGADR, address);
	// Write the PHY data.
	enc28j60Write(MIWRL, data & 0xFF);
	enc28j60Write(MIWRH, data >> 8);
	
	// Wait until the PHY write completes.
	while(enc28j60Read(MISTAT)&MISTAT_BUSY){
		Delay(15);
	}
}


void enc28j60clkout(uint8_t clk){
	// Setup clkout: 2 is 12.5MHz:
	enc28j60Write(ECOCON, clk&0x7);
}


void enc28j60Init(uint8_t* macaddr){
	enableChip;	// Chip Select = 0.
	
	// Perform system reset.
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
	Delay(50);
	// Check CLKRDY bit to see if reset is complete.
	// The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
	//while(!(enc28j60Read(ESTAT)&ESTAT_CLKRDY));
	
	// Do bank 0 stuff:
	// - Initialise RX buffer
	// - 16-bit transfers, must write low byte first
	// - Set RX buffer start and end address 
	// - Set TX buffer start and end address
	gNextPacketPtr = RXSTART_INIT;
	// Set RX buffer Start.
	enc28j60WriteWord(ERXSTL,  RXSTART_INIT);
	// Set RX pointer address.
	enc28j60WriteWord(ERXRDPTL, RXSTART_INIT);
	// Set RX buffer End.
	enc28j60WriteWord(ERXNDL, RXSTOP_INIT);
	// Set TX buffer Start.
	enc28j60WriteWord(ETXSTL, TXSTART_INIT);
	// Set TX buffer End.
	enc28j60WriteWord(ETXNDL, TXSTOP_INIT);
	
	// Do bank 1 stuff, packet filer:
	//	- In regards to broadcast packets, only ARP packets are allowed
	//	- All other packets should be unicast for our MAC only (MAADR)
	//
	// The pattern to match on is therefore:
	//	Type	ETH.DST
	//	ARP		BROADCAST
	//	06 08 -- ff ff ff ff ff ff -> ip checksum for these bytes = f7f9
	// In binary these positions are: 11 0000 0011 11111
	// This is hex 303F->EPMM0=0x3F, EPMM1=0x30.
	
	erxfcon = ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN|ERXFCON_BCEN;
	enc28j60Write(ERXFCON, erxfcon);
	enc28j60WriteWord(EPMM0, 0x303F);
	enc28j60WriteWord(EPMCSL, 0xF7F9);
	
	// Do bank 2 stuff:
	// Enable MACK receive.
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// Bring MAC out of reset.
	enc28j60Write(MACON2, 0x00);
	// Enable automatic padding to 60bytes and CRC operations.
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, 
			MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// Set inter-frame gap (non-back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept.
	// Do not send packets longer than MAX_FRAMELEN.
	enc28j60WriteWord(MAMXFLL, MAX_FRAMELEN);
	
	// Do bank 3 studd:
	// Write MAC address.
	// !!!NOTE: The MAC address in the ENC28J60 controller is byte-backward!!!
	enc28j60Write(MAADR5, macaddr[0]);
	enc28j60Write(MAADR4, macaddr[1]);
	enc28j60Write(MAADR3, macaddr[2]);
	enc28j60Write(MAADR2, macaddr[3]);
	enc28j60Write(MAADR1, macaddr[4]);
	enc28j60Write(MAADR0, macaddr[5]);
	// No loopback of transmitted frames.
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);
	
	// Switch to bank 0.
	enc28j60SetBank(ECON1);
	// Enable interrupts.
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
	// Enable packet reception.
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}


// Read the revision of the chip:
uint8_t enc28j60getrev(void){
	uint8_t rev;
	rev=enc28j60Read(EREVID);
	// Microchip forgot to step the number on the silicon when they
	// released the B7 revision. 6 is now rev. B7. 
	// We still have to see what they will do when they release rev. B8.
	// There is currently no revision B8 at the moment.
	if(rev>5) rev++;
	return rev;
}


// A number of utility functions to enable/disable broadcast and multicast bits.
void enc28j60EnableBroadcast(void){
	erxfcon |= ERCFCON_BCEN;
	enc28j60Write(ERXFCON, erxfcon);
}


void enc28j60DisableBroadcast(void){
	erxfcon &= ~ERXFCON_BCEN;
	enc28j60Write(ERXFCON, erxfcon);
}


void enc28j60EnableMulticast(void){
	erxfcon |= ERXFCON_MCEN;
	enc28j60Write(ERXFCON, erxfcon);
}


void enc28j60DisableMulticast(void){
	erxfcon &= ~ERXFCON_MCEN;
	enc28j60Write(ERXFCON, erxfcon);
}


// Link Status.
uint8_t enc28j60linkup(void){
	// Bit 10 (bit 3 of upper reg)
	return(enc28j60PhyReadH(PHSTAT2)&4);
}


void enc28j60PacketSend(uint16_t len, uint8_t* packet){
	// Verify that no transmission is in progress.
	while(enc28j60ReadOp(ENC28J60_READ_CTRL_REG, ECON1)&ECON1_TXRTS){
		// Reset the transmit logic problem. See Rev B4 Silicon Errata point 12.
		if(enc28j60Read(EIR)&EIR_TXERIF){
			enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
			enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
		}
	}
	
	// Set the write pointer to the beginning of TX buffer area.
	enc28j60WriteWord(EWRPTL, TXSTART_INIT);
	// Set the TXND pointer to correspond to the packet size given.
	enc28j60WriteWord(ETXNDL, (TXSTART_INIT+len));
	//Write per-packet control byte (0x00 means Use MACON3 settings).
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
	// Copy the packet into the TX buffer.
	enc28j60WriteBuffer(len, packet);
	// Send the content of the transmit buffer onto the network.
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}


// Gets a packet from the network RX buffer, if one is available.
// The packet will be headed by an ethernet header.
//		maxlen:		The maximum acceptable length of a retrieved packet.
//		packet:		The pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet){
	uint16_t rxstat;
	uint16_t len;
	
	// Check if a packet has been received and buffered.
	//if(!(enc28j60Read(EIR)&EIR_PKTIF)){
	// The above does not work. See Rev. B4 Silicon Errata point 6.
	// If a packet has been received, the EPKTCNT pointer will have moved from
	// it's original position, AKA ERXSTL, or 0x0000.
	if(enc28j60Read(EPKTCNT) == 0){
		return 0;
	}
	
	// Set the read pointer to the start of the received packet.
	enc28j60WriteWord(ERDPTL, gNextPacketPtr);
	// Read the next packet pointer.
	gNextPacketPtr = enc28j60ReadBufferWord();
	// Read the packet length (see datasheet page 43) and remove the CRC count.
	len = enc28j60ReadBufferWord() - 4;
	// Read the receive status (See datasheet page 43).
	rxstat = enc28j60ReadBufferWord();
	// Limit retrieve length.
	if(len>maxlen-1){
		len=maxlen-1;
	}
	// Check CRC and symbol errors (see datasheet page 44, table 7-3):
	// The ERXFCON.CRCEN is set by default, we should not need to check this.
	if((rxstat&0x80)==0){
		// Invalid.
		len=0;
	}
	else{
		// Copy the packet from the receive buffer.
		enc28j60ReadBuffer(len, packet);
	}
	// Move the RX read pointer to the start of the next received packet.
	// This frees the memory we just read out.
	enc28j60WriteWord(ERXRDPTL, gNextPacketPtr);
	// However, compensate for the errata point 13, rev B4: 
	//    never write an even address!
	if((gNextPacketPtr-1<RXSTART_INIT) || (gNextPacketPtr-1>RXSTOP_INIT)){
		enc28j60WriteWord(ERXRDPTL, RXSTOP_INIT);
	}
	else{
		enc28j60WriteWord(ERXRDPTL, (gNextPacketPtr-1));
	}
	// Decrement the packet counter indicate we are done with this packet.
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
	return len;
}

