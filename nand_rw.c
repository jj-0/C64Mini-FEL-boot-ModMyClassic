/*
 * gpio_lib.c
 *
 * Copyright 2013 Stefan Mavrodiev <support@olimex.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */


#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>

#include "gpio_lib.h"

#define NULL ((void*)0)
#define MAX_WAIT_READ_BUSY	1000000

//#define DEBUG 1
//#define DEBUG2 1

/*
Pin/Signal	Description			Type			
NAND FLASH					
NDQ[7:0]	NAND Flash Data Bit[7:0]	I/O			
NCE[7:0]	NAND Flash Chip Select[7:0]	O			
NWE#		NAND Flash Write Enable		O			
NALE		NAND Flash Address Latch Enable	O			
NCLE		NAND Flash Command Latch Enable	O			
NRE#		NAND Flash Read Enable		O			
NRB[1:0]	NAND Flash Ready/Busy Bit	I			
NWP		NAND Flash Write Protection	O			
NDQS		NAND Flash Data Strobe		I/O			
					

	Default					[nand_para]
Port	Pull-up/down	Definition		nand_used = 1
PC0	Z		NWE#			nand_we = port:PC00<2><default><default><default>
PC1	Z		NALE			nand_ale = port:PC01<2><default><default><default>
PC2	Z		NCLE			nand_cle = port:PC02<2><default><default><default>
PC3	Pull-Up		NCE1			nand_ce1 = port:PC03<2><default><default><default>
PC4	Pull-Up		NCE0			nand_ce0 = port:PC04<2><default><default><default>
PC5	Z		NRE#			nand_nre = port:PC05<2><default><default><default>
PC6	Pull-Up		NRB0			nand_rb0 = port:PC06<2><default><default><default>
PC7	Pull-Up		NRB1			nand_rb1 = port:PC07<2><default><default><default>
PC8	Z		NDQ0			nand_d0 = port:PC08<2><default><default><default>
PC9	Z		NDQ1			nand_d1 = port:PC09<2><default><default><default>
PC10	Z		NDQ2			nand_d2 = port:PC10<2><default><default><default>
PC11	Z		NDQ3			nand_d3 = port:PC11<2><default><default><default>
PC12	Z		NDQ4			nand_d4 = port:PC12<2><default><default><default>
PC13	Z		NDQ5			nand_d5 = port:PC13<2><default><default><default>
PC14	Z		NDQ6			nand_d6 = port:PC14<2><default><default><default>
PC15	Z		NDQ7			nand_d7 = port:PC15<2><default><default><default>
PC16	Pull-Down	NWP			nand_wp = port:PC16<2><default><default><default>
PC17	Pull-Up		NCE2			nand_ce2 = port:PC17<2><default><default><default>
PC18	Pull-Up		NCE3			nand_ce3 = port:PC18<2><default><default><default>
PC19	Z		NCE4			nand_ce4 =
PC20	Z		NCE5			nand_ce5 =
PC21	Z		NCE6			nand_ce6 =
PC22	Z		NCE7			nand_ce7 =
PC23	Pull-Up		nand_spi		nand_spi = port:PC23<3><default><default><default>
PC24	Z		NDQS			nand_ndqs = port:PC24<2><default><default><default>

All default input					

*/


/* Pins changed to A20 nand controller */
#define N_WRITE_PROTECT		SUNXI_GPC(16) // PC16 pulled down ** pulled up by RPi, this is useful
#define N_READ_BUSY		SUNXI_GPC(6) // PC6 NRB0 Pull-Up PC7 = NRB1 **pulled up by RPi, this is also useful
#define N_READ_BUSY2		SUNXI_GPC(7) // PC6 NRB0 Pull-Up PC7 = NRB1 **pulled up by RPi, this is also useful

#define N_WRITE_ENABLE 		SUNXI_GPC(0) // PC0 NWE#
#define ADDRESS_LATCH_ENABLE	SUNXI_GPC(1)  // PC1 NALE
#define COMMAND_LATCH_ENABLE	SUNXI_GPC(2) // PC2 NCLE
#define N_READ_ENABLE		SUNXI_GPC(5) // PC5 NRE#
#define N_CHIP_ENABLE		SUNXI_GPC(4) // PC4 NCE0 Pull-Up PC3 = NCE1
#define N_CHIP_ENABLE2		SUNXI_GPC(3) // PC4 NCE0 Pull-Up PC3 = NCE1

static const unsigned data_to_gpio_map[8] = { SUNXI_GPC(8), SUNXI_GPC(9), SUNXI_GPC(10), SUNXI_GPC(11), SUNXI_GPC(12), SUNXI_GPC(13), SUNXI_GPC(14), SUNXI_GPC(15) }; // 8 is NDQ0, etc.

//static const char num2hex_map[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
static const unsigned nand_gpio_nums[17] = { N_WRITE_PROTECT, N_READ_BUSY, N_WRITE_ENABLE, ADDRESS_LATCH_ENABLE, COMMAND_LATCH_ENABLE, N_READ_ENABLE, N_CHIP_ENABLE, SUNXI_GPC(8), SUNXI_GPC(9), SUNXI_GPC(10), SUNXI_GPC(11), SUNXI_GPC(12), SUNXI_GPC(13), SUNXI_GPC(14), SUNXI_GPC(15), N_READ_BUSY2, N_CHIP_ENABLE2 };
static const char nand_gpio_names[18][30] = { "N_WRITE_PROTECT", "N_READ_BUSY", "N_WRITE_ENABLE", "ADDRESS_LATCH_ENABLE", "COMMAND_LATCH_ENABLE", "N_READ_ENABLE", "N_CHIP_ENABLE", "IO0", "IO1", "IO2", "IO3", "IO4", "IO5", "IO6", "IO7", "N_READ_BUSY2", "N_CHIP_ENABLE2", "UNKNOWN" };

unsigned int SUNXI_PIO_BASE = 0;

static volatile long int *gpio_map = NULL;
int read_id(unsigned char id[5]);
int read_pages(int first_page_number, int number_of_pages, char *outfile, int write_spare);
int write_pages(int first_page_number, int number_of_pages, char *infile);
int erase_blocks(int first_block_number, int number_of_blocks);

int sunxi_gpio_init(void) {
    int fd;
    unsigned int addr_start, addr_offset;
    unsigned int PageSize, PageMask;


    fd = open("/dev/mem", O_RDWR);
    if(fd < 0) {
        return SETUP_DEVMEM_FAIL;
    }

    PageSize = sysconf(_SC_PAGESIZE);
    PageMask = (~(PageSize-1));

    addr_start = SW_PORTC_IO_BASE & PageMask;
    addr_offset = SW_PORTC_IO_BASE & ~PageMask;

    gpio_map = (void *)mmap(0, PageSize*2, PROT_READ|PROT_WRITE, MAP_SHARED, fd, addr_start);
    if(gpio_map == MAP_FAILED) {
        return SETUP_MMAP_FAIL;
    }

    SUNXI_PIO_BASE = (unsigned int)gpio_map;
    SUNXI_PIO_BASE += addr_offset;

    close(fd);
    return SETUP_OK;
}

int sunxi_gpio_set_cfgpin(unsigned int pin, unsigned int val) {

    unsigned int cfg;
    unsigned int bank = GPIO_BANK(pin);
    unsigned int index = GPIO_CFG_INDEX(pin);
    unsigned int offset = GPIO_CFG_OFFSET(pin);

    if(SUNXI_PIO_BASE == 0) {
        return -1;
    }

    struct sunxi_gpio *pio =
        &((struct sunxi_gpio_reg *)SUNXI_PIO_BASE)->gpio_bank[bank];


    cfg = *(&pio->cfg[0] + index);
    cfg &= ~(0xf << offset);
    cfg |= val << offset;

    *(&pio->cfg[0] + index) = cfg;

    return 0;
}

int sunxi_gpio_get_cfgpin(unsigned int pin) {

    unsigned int cfg;
    unsigned int bank = GPIO_BANK(pin);
    unsigned int index = GPIO_CFG_INDEX(pin);
    unsigned int offset = GPIO_CFG_OFFSET(pin);
    if(SUNXI_PIO_BASE == 0)
    {
        return -1;
    }
    struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *)SUNXI_PIO_BASE)->gpio_bank[bank];
    cfg = *(&pio->cfg[0] + index);
    cfg >>= offset;
    return (cfg & 0xf);
}
int sunxi_gpio_output(unsigned int pin, unsigned int val) {

    unsigned int bank = GPIO_BANK(pin);
    unsigned int num = GPIO_NUM(pin);

    if(SUNXI_PIO_BASE == 0)
    {
        return -1;
    }
    struct sunxi_gpio *pio =&((struct sunxi_gpio_reg *)SUNXI_PIO_BASE)->gpio_bank[bank];

    if(val)
        *(&pio->dat) |= 1 << num;
    else
        *(&pio->dat) &= ~(1 << num);

    return 0;
}

int sunxi_gpio_input(unsigned int pin) {

    unsigned int dat;
    unsigned int bank = GPIO_BANK(pin);
    unsigned int num = GPIO_NUM(pin);

    if(SUNXI_PIO_BASE == 0)
    {
        return -1;
    }

    struct sunxi_gpio *pio =&((struct sunxi_gpio_reg *)SUNXI_PIO_BASE)->gpio_bank[bank];

    dat = *(&pio->dat);
    dat >>= num;

    return (dat & 0x1);
}
void sunxi_gpio_cleanup(void)
{
    unsigned int PageSize;
    if (gpio_map == NULL)
        return;

    PageSize = sysconf(_SC_PAGESIZE);
    munmap((void*)gpio_map, PageSize*2);
}

char *gpio_num_to_name(unsigned g)
{
	int i=8;
	int j=17;
	unsigned k;
	for (i=0; i<18; i++)
	{
		k = nand_gpio_nums[i];
		if (g==k)
		{
			j=i;
		}
	}
	return nand_gpio_names + j;
}

void INP_GPIO(int g)
{
#ifdef DEBUG2
	printf("setting direction of GPIO %s to input\n", gpio_num_to_name(g));
	printf("\tcurrent direction:\t%d\n",sunxi_gpio_get_cfgpin(g));
#endif
	sunxi_gpio_set_cfgpin(g,INPUT);
#ifdef DEBUG2
	printf("\tnew direction:\t%d\n",sunxi_gpio_get_cfgpin(g));
#endif
}

void OUT_GPIO(int g)
{
//	INP_GPIO(g);
#ifdef DEBUG2
	printf("setting direction of GPIO %s to output\n", gpio_num_to_name(g));
	printf("\tcurrent direction:\t%d\n",sunxi_gpio_get_cfgpin(g));
#endif
	sunxi_gpio_set_cfgpin(g,OUTPUT);
#ifdef DEBUG2
	printf("\tnew direction:\t%d\n",sunxi_gpio_get_cfgpin(g));
#endif
}

void GPIO_SET_1(int g)
{
	sunxi_gpio_output(g, 1);

#ifdef DEBUG
	printf("setting GPIO %s to 1\n", gpio_num_to_name(g));
//	GPIO_READ(g);
	
#endif
}

void GPIO_SET_0(int g)
{
	sunxi_gpio_output(g, 0);

#ifdef DEBUG
	printf("setting GPIO %s to 0\n", gpio_num_to_name(g));
//	GPIO_READ(g);
#endif
}

int GPIO_READ(int g)
{
	int x = sunxi_gpio_input(g);
#ifdef DEBUG
//	printf("GPIO %s reads as %d\n", gpio_num_to_name(g), x);
#endif
	return x;
}

void set_data_direction_in(void)
{
	int i;
#ifdef DEBUG
	printf("data direction => IN\n");
#endif
	for (i = 0; i < 8; i++)
		INP_GPIO(data_to_gpio_map[i]);
}

void set_data_direction_out(void)
{
	int i;
#ifdef DEBUG
	printf("data direction => OUT\n");
#endif
	for (i = 0; i < 8; i++)
		OUT_GPIO(data_to_gpio_map[i]);
}

int GPIO_DATA8_IN(void)
{
	int i, data;
	for (i = data = 0; i < 8; i++, data = data << 1) {
		data |= GPIO_READ(data_to_gpio_map[7 - i]);
	}
	data >>= 1;
#ifdef DEBUG
	printf("GPIO_DATA8_IN: data=%02x\n", data);
#endif
	return data;
}

void GPIO_DATA8_OUT(int data)
{
	int i;
#ifdef DEBUG
	printf("GPIO_DATA8_OUT: data=%02x\n", data);
#endif
	for (i = 0; i < 8; i++, data >>= 1) {
		if (data & 1)
			GPIO_SET_1(data_to_gpio_map[i]);
		else
			GPIO_SET_0(data_to_gpio_map[i]);
	}
}

int delay = 1;
int PAGE_SIZE = 2176; // (2K + 128)Byte
int BLOCK_SIZE = 139264; // 64 pages (128K + 8K)Byte

void shortpause()
{
	int i;
	volatile int dontcare = 0;
	for (i = 0; i < delay; i++) {
		dontcare++;
	}
}


void error_msg(char *msg)
{
	printf("%s\nBe sure to check wiring, and check that pressure is applied on clip (if used)\n", msg);
}

void print_id(unsigned char id[5])
{
	unsigned int i, bit, page_size, ras_size, orga, plane_number;
	unsigned long block_size, plane_size, nand_size, nandras_size;
	char maker[16], device[16], serial_access[20];
	unsigned *thirdbits = (unsigned*)malloc(sizeof(unsigned) * 8);
	unsigned *fourthbits = (unsigned*)malloc(sizeof(unsigned) * 8);
	unsigned *fifthbits = (unsigned*)malloc(sizeof(unsigned) * 8);

	printf("Raw ID data: ");
	for (i = 0; i < 5; i++)
		printf("0x%02X ", id[i]);
	printf("\n");

 	switch(id[0]) {
 		case 0xEC: {
 			strcpy(maker, "Samsung");
 			switch(id[1]) {
 				case 0xA1: strcpy(device, "K9F1G08R0A"); break;
 				case 0xD5: strcpy(device, "K9GAG08U0M"); break;
 				case 0xF1: strcpy(device, "K9F1G08U0A/B"); break;
 				default: strcpy(device, "unknown");
 			}
 			break;
 		}
 		case 0xAD: {
 			strcpy(maker, "Hynix");
 			switch(id[1]) {
 				case 0x73: strcpy(device, "HY27US08281A"); break;
 				case 0xD7: strcpy(device, "H27UBG8T2A"); break;
 				case 0xDA: strcpy(device, "HY27UF082G2B"); break;
 				case 0xDC: strcpy(device, "H27U4G8F2D"); break;
 				default: strcpy(device, "unknown");
 			}
 			break;
 		}
 		case 0x2C: {
 			strcpy(maker, "Micron");
 			switch(id[1]) {
 				default: strcpy(device, "unknown");
 			}
 			break;
 		}
 		default: strcpy(maker, "unknown"); strcpy(device, "unknown");
 	}

/* all sizes in bytes */
	for(bit = 0; bit < 8; ++bit)
		thirdbits[bit] = (id[2] >> bit) & 1;

	for(bit = 0; bit < 8; ++bit)
		fourthbits[bit] = (id[3] >> bit) & 1;
	switch(fourthbits[1] * 10 + fourthbits[0]) {
		case 00: page_size = 1024; break;
		case 01: page_size = 2048; break;
		case 10: page_size = 4096; break;
		case 11: page_size = 8192; break;
	}
	switch(fourthbits[5] * 10 + fourthbits[4]) {
		case 00: block_size = 64 * 1024; break;
		case 01: block_size = 128 * 1024; break;
		case 10: block_size = 256 * 1024; break;
		case 11: block_size = 512 * 1024; break;
	}
	switch(fourthbits[2]) {
		case 0: ras_size = 8; break; // for 512 bytes
		case 1: ras_size = 16; break; // for 512 bytes
	}
	switch(fourthbits[6]) {
		case 0: orga = 8; break; // bits
		case 1: orga = 16; break; // bits
	}
	switch(fourthbits[7] * 10 + fourthbits[3]) {
		case 00: strcpy(serial_access, "50ns/30ns minimum"); break;
		case 10: strcpy(serial_access, "25ns minimum"); break;
		case 01: strcpy(serial_access, "unknown (reserved)"); break;
		case 11: strcpy(serial_access, "unknown (reserved)"); break;
	}

	for(bit = 0; bit < 8; ++bit)
		fifthbits[bit] = (id[4] >> bit) & 1;
	switch(fifthbits[3] * 10 + fifthbits[2]) {
		case 00: plane_number = 1; break;
		case 01: plane_number = 2; break;
		case 10: plane_number = 4; break;
		case 11: plane_number = 8; break;
	}
	switch(fifthbits[6] * 100 + fifthbits[5] * 10 + fifthbits[4]) {
		case 000: plane_size = 64 / 8 * 1024 * 1024; break; // 64 megabits
		case 001: plane_size = 128 / 8 * 1024 * 1024; break; // 128 megabits
		case 010: plane_size = 256 / 8 * 1024 * 1024; break; // 256 megabits
		case 011: plane_size = 512 / 8 * 1024 * 1024; break; // 512 megabits
		case 100: plane_size = 1024 / 8 * 1024 * 1024; break; // 1 gigabit
		case 101: plane_size = 2048 / 8 * 1024 * 1024; break; // 2 gigabits
		case 110: plane_size = 4096 / 8 * 1024 * 1024; break; // 4 gigabits
		case 111: plane_size = 8192 / 8 * 1024 * 1024; break; // 8 gigabits
	}

	nand_size = plane_number * plane_size;
	nandras_size = nand_size + ras_size * nand_size / 512;

	printf("\n");
	printf("NAND manufacturer:  %s (0x%02X)\n", maker, id[0]);
	printf("NAND model:         %s (0x%02X)\n", device, id[1]);
	printf("\n");

	printf("              I/O|7|6|5|4|3|2|1|0|\n");
	printf("3rd ID data:     |");
	for(bit = 8; bit--;)
        printf("%u|", thirdbits[bit]);
    printf(" (0x%02X)\n", id[2]);
	printf("4th ID data:     |");
	for(bit = 8; bit--;)
        printf("%u|", fourthbits[bit]);
    printf(" (0x%02X)\n", id[3]);
	printf("5th ID data:     |");
	for(bit = 8; bit--;)
        printf("%u|", fifthbits[bit]);
    printf(" (0x%02X)\n", id[4]);

	printf("\n");
	printf("Page size:          %d bytes\n", page_size);
	printf("Block size:         %lu bytes\n", block_size);
	printf("RAS (/512 bytes):   %d bytes\n", ras_size);
	// printf("RAS (per page):  %d bytes\n", ras_size * page_size / 512);
	// printf("RAS (per block): %d bytes\n", ras_size * block_size / 512);
	printf("Organisation:       %d bit\n", orga);
	printf("Serial access:      %s\n", serial_access);
	printf("Number of planes:   %d\n", plane_number);
	printf("Plane size:         %lu bytes\n", plane_size);
	printf("\n");
	printf("NAND size:          %lu MB\n", nand_size / (1024 * 1024));
	printf("NAND size + RAS:    %lu MB\n", nandras_size / (1024 * 1024));
	printf("Number of blocks:   %lu\n", nand_size / block_size);
	printf("Number of pages:    %lu\n", nand_size / page_size);
}

int read_id(unsigned char id[5])
{
	int i;
	unsigned char buf[5];

	#ifdef DEBUG
	printf("READ ID");
	#endif

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	set_data_direction_out(); GPIO_DATA8_OUT(0x90); // Read ID byte 1
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause(); set_data_direction_in();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(ADDRESS_LATCH_ENABLE);
	GPIO_SET_0(N_WRITE_ENABLE);
	set_data_direction_out(); GPIO_DATA8_OUT(0x00); // Read ID byte 2
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause(); set_data_direction_in();
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);
	shortpause();

	for (i = 0; i < 5; i++) {
		GPIO_SET_0(N_READ_ENABLE);
		shortpause();
		buf[i] = GPIO_DATA8_IN();
		GPIO_SET_1(N_READ_ENABLE);
			shortpause();
	}
	if (id != NULL)
		memcpy(id, buf, 5);
	else
		print_id(buf);
	if (buf[0] == buf[1] && buf[1] == buf[2] && buf[2] == buf[3] && buf[3] == buf[4]) {
		error_msg("all five ID bytes are identical, this is not normal");
		return -1;
	}
	return 0;
}

int page_to_address(int page, int address_byte_index)
{
	switch(address_byte_index) {
	case 2:
		return page & 0xff;
	case 3:
		return (page >>  8) & 0xff;
	case 4:
		return (page >> 16) & 0xff;
	default:
		return 0;
	}
}

int send_read_command(int page)
{
	int i;

	set_data_direction_out();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause(); GPIO_SET_0(N_WRITE_ENABLE);
	GPIO_DATA8_OUT(0x00);
	shortpause(); GPIO_SET_1(N_WRITE_ENABLE);
	shortpause(); GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(ADDRESS_LATCH_ENABLE);
	for (i = 0; i < 5; i++) {
		GPIO_SET_0(N_WRITE_ENABLE);
		GPIO_DATA8_OUT(page_to_address(page, i));
		shortpause();
		GPIO_SET_1(N_WRITE_ENABLE);
		shortpause();
	}
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	shortpause();
	GPIO_DATA8_OUT(0x30);
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	return 0;
}

int send_write_command(int page, unsigned char data[PAGE_SIZE])
{
	int i;

	set_data_direction_out();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	shortpause();
	GPIO_DATA8_OUT(0x80);
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(ADDRESS_LATCH_ENABLE);
	for (i = 0; i < 5; i++) {
		GPIO_SET_0(N_WRITE_ENABLE);

		// if (i < 2) {
		// 	printf("Col Add%d = %d\n", i + 1, page_to_address(page, i));
		// }
		// else {
		// 	printf("Row Add%d = %d\n", i - 1, page_to_address(page, i));
		// }

		GPIO_DATA8_OUT(page_to_address(page, i));
		shortpause();
		GPIO_SET_1(N_WRITE_ENABLE);
		shortpause();
	}
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);
	shortpause();

	for (i = 0; i < PAGE_SIZE; i++) {
		GPIO_SET_0(N_WRITE_ENABLE);
		shortpause();
		GPIO_DATA8_OUT(data[i]); //
		shortpause();
		GPIO_SET_1(N_WRITE_ENABLE);
		shortpause();
//		printf("%X",data[i]);
	}

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause(); GPIO_SET_0(N_WRITE_ENABLE);
	GPIO_DATA8_OUT(0x10);
	shortpause(); GPIO_SET_1(N_WRITE_ENABLE);
	shortpause(); GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	return 0;
}

int send_eraseblock_command(int block)
{
	int i;

	set_data_direction_out();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	shortpause();
	GPIO_DATA8_OUT(0x60);
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(ADDRESS_LATCH_ENABLE);
	for (i = 2; i < 5; i++) {
		GPIO_SET_0(N_WRITE_ENABLE);
		shortpause();

		// printf("Row Add%d = %d\n", i - 1, page_to_address(block, i));

		GPIO_DATA8_OUT(page_to_address(block, i));
		shortpause();
		GPIO_SET_1(N_WRITE_ENABLE);
		shortpause();
	}
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	shortpause();
	GPIO_DATA8_OUT(0xD0);
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	return 0;
}

int read_status()
{
	int i, data;
	unsigned char buf[5];

	set_data_direction_out();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause(); GPIO_SET_0(N_WRITE_ENABLE);
	GPIO_DATA8_OUT(0x70);
	shortpause(); GPIO_SET_1(N_WRITE_ENABLE);
	shortpause(); GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	set_data_direction_in();

	GPIO_SET_0(N_READ_ENABLE);
	shortpause();
	data = GPIO_DATA8_IN(); //
	shortpause();
	GPIO_SET_1(N_READ_ENABLE);
	shortpause();

	// printf("Status data = %d\n", data);

	return data & 1; // I/O0=0 success , I/O0=1 error
}


int read_pages(int first_page_number, int number_of_pages, char *outfile, int write_spare)
{
	int page, page_no, block_no, page_nbr, percent, i, n, retry_count;
	unsigned char id[5], id2[5];
	unsigned char buf[PAGE_SIZE * 2];
	FILE *badlog, *f = fopen(outfile, "w+");
	if (f == NULL) {
		perror("fopen output file");
		return -1;
	}
	if ((badlog = fopen("bad.log", "w+")) == NULL) {
		perror("fopen bad.log");
		return -1;
	}
	if (GPIO_READ(N_READ_BUSY) == 0) {
		error_msg((char*)"N_READ_BUSY should be 1 (pulled up), but reads as 0. make sure the NAND is powered on");
		return -1;
	}

	if (read_id(id) < 0)
		return -1;
	print_id(id);
	printf("if this ID is incorrect, press Ctrl-C NOW to abort (3s timeout)\n");
	sleep(3);

	printf("\nStart reading...\n");
	clock_t start = clock();


	for (retry_count = 0, page = first_page_number*2; page < (first_page_number + number_of_pages)*2; page++) {

	  retry_all:
		page_no = page >> 1;

		// printf("page = %d, n = %d\n",page, n);

		if (page % 2 == 0 && retry_count == 0) {
			// page_no = page / 2;
			page_nbr = page_no - first_page_number + 1;
			percent = (100 * page_nbr) / number_of_pages;
			block_no = page_no / 64;
			printf("Reading page n° %d in block n° %d (page %d of %d), %d%%\r", page_no, block_no, page_nbr, number_of_pages, percent);
			fflush(stdout);
		}
		// else {
		// 	printf("Reading the page again to ensure correct operation\n");
		// }

	  retry:
		read_id(id2);
		if (memcmp(id, id2, 5) != 0) {
			printf("\nNAND ID has changed! retrying");
			goto retry;
		}
		send_read_command(page_no);
		//for (i = 0; i < MAX_WAIT_READ_BUSY; i++) {
		//	if (GPIO_READ(N_READ_BUSY) == 0)
		//		break;
		//}
		while (GPIO_READ(N_READ_BUSY) == 0) {
			// printf("Busy\n");
			shortpause();
		}
		// if (i == MAX_WAIT_READ_BUSY) {
		// 	// #ifdef DEBUG
		// 		printf("N_READ_BUSY was not brought to 0 by NAND in time, retrying\n");
		// 	// #endif
		// 	goto retry;
		// }
		set_data_direction_in();
		// for (i = 0; i < MAX_WAIT_READ_BUSY; i++) {
		// 	if (GPIO_READ(N_READ_BUSY) == 1)
		// 		break;
		// }
		// if (i == MAX_WAIT_READ_BUSY) {
		// 	// #ifdef DEBUG
		// 		printf("N_READ_BUSY was not brought to 1 by NAND in time, retrying\n");
		// 	// #endif
		// 	goto retry;
		// }
		n = PAGE_SIZE*(page & 1);
		for (i = 0; i < PAGE_SIZE; i++) {
			GPIO_SET_0(N_READ_ENABLE);
			shortpause();
			buf[i + n] = GPIO_DATA8_IN();  /* <--- read should be here (while N_READ GPIO is set to 0 )*/
			GPIO_SET_1(N_READ_ENABLE);
			shortpause();
		}
		if (!n) // read the page again to ensure correct operation, bit 0 in page used for this purpose
			// printf("RE LOOP    | page = %d, n = %d\n",page, n);
			// printf("Reading the page n° %d again to ensure correct operation\n", page_no);
			continue;

		if (memcmp(buf, buf + PAGE_SIZE, PAGE_SIZE) != 0) {
			if (retry_count == 0) printf("\n");
			if (retry_count < 5) {
				printf("Page failed to read correctly! retrying\n");
				retry_count++;
				page = page & ~1;
				goto retry_all;
			}
			printf("Too many retries. Perhaps bad block?\n");
			fprintf(badlog, "Page %d seems to be bad\n", page_no);
		}
		if (write_spare) {
			if (fwrite(buf, PAGE_SIZE, 1, f) != 1) {
				perror("fwrite");
				return -1;
			}
		}
		else {
			if (fwrite(buf, 512 * (PAGE_SIZE / 512), 1, f) != 1) {
				perror("fwrite");
				return -1;
			}
		}
		retry_count = 0;
	}
	fcloseall();
	clock_t end = clock();
	printf("\n\nReading done in %f seconds\n", (float)(end - start) / CLOCKS_PER_SEC);

	//show cursor
	// printf("\e[?25h");
	// fflush(stdout) ;
}

/*int read_pages(int first_page_number, int number_of_pages, char *outfile, int write_spare)
{
	int page, block_no, page_nbr, percent, i;
	unsigned char buf[PAGE_SIZE], id[5], id2[5];;
	FILE *f = fopen(outfile, "w+");
	if (f == NULL) {
		perror("fopen output file");
		return -1;
	}
	if (GPIO_READ(N_READ_BUSY) == 0) {
		error_msg((char*)"N_READ_BUSY should be 1 (pulled up), but reads as 0. make sure the NAND is powered on");
		return -1;
	}

	if (read_id(id) < 0)
		return -1;
	print_id(id);
	printf("if this ID is incorrect, press Ctrl-C NOW to abort (3s timeout)\n");
	sleep(3);

	printf("\nStart reading...\n\n");
	clock_t start = clock();


	for (page = first_page_number; page < first_page_number + number_of_pages; page++) {

		// printf("page = %d, n = %d\n",page, n);

		// page_nbr = page - first_page_number + 1;
		// percent = (100 * page_nbr) / number_of_pages;
		// block_no = page / 64;
		// printf("Reading page n° %d in block n° %d (page %d of %d), %d%%\n", page, block_no, page_nbr, number_of_pages, percent);
		printf("\nReading page n° %d\n", page);

		send_read_command(page);
		while (GPIO_READ(N_READ_BUSY) == 0) {
			// printf("Busy\n");
			shortpause();
		}
		set_data_direction_in();
		for (i = 0; i < PAGE_SIZE; i++) {
			GPIO_SET_0(N_READ_ENABLE);
			shortpause();
			buf[i] = GPIO_DATA8_IN(); //
			shortpause();
			GPIO_SET_1(N_READ_ENABLE);
			shortpause();
		}
		if (write_spare) {
			if (fwrite(buf, PAGE_SIZE, 1, f) != 1) {
				perror("fwrite");
				return -1;
			}
		}
		else {
			if (fwrite(buf, 512 * (PAGE_SIZE / 512), 1, f) != 1) {
				perror("fwrite");
				return -1;
			}
		}
	}
	fcloseall();
	clock_t end = clock();
	printf("\nReading done in %f seconds\n", (float)(end - start) / CLOCKS_PER_SEC);
}
*/
int write_pages(int first_page_number, int number_of_pages, char *infile)
{
	int page, block_no, page_nbr, percent, retry_count;
	unsigned char buf[PAGE_SIZE], id[5], id2[5];;

	if (read_id(id) < 0)
		return -1;
	print_id(id);
	printf("if this ID is incorrect, press Ctrl-C NOW to abort (3s timeout)\n");
	sleep(3);

	printf("\nStart writing...\n");
	clock_t start = clock();


	FILE *f = fopen(infile, "rb");
	if (f == NULL) {
		perror("fopen input file");
		return -1;
	}

	// printf("first_page_number = %d\n", first_page_number);
	// printf("number of pages = %d\n", number_of_pages);


	for (retry_count = 0, page = first_page_number; page < first_page_number + number_of_pages; page++) {

	  retry_all:

		if (retry_count == 0) {
			// page_no = page / 2;
			page_nbr = page - first_page_number + 1;
			percent = (100 * page_nbr) / number_of_pages;
			block_no = page / 64;
			printf("Writing page n° %d in block n° %d (page %d of %d), %d%%\r", page, block_no, page_nbr, number_of_pages, percent);
			fflush(stdout);
		}

		fseek(f, (page - first_page_number) * PAGE_SIZE, SEEK_SET);
		fread(buf, PAGE_SIZE, 1, f);

		// printf("\nwriting page n°%d\n", page);

	  retry:
		read_id(id2);
		if (memcmp(id, id2, 5) != 0) {
			printf("\nNAND ID has changed! retrying");
			goto retry;
		}

		send_write_command(page, buf);
		while (GPIO_READ(N_READ_BUSY) == 0) {
			// printf("Busy\n");
			shortpause();
		}
		// read_status();
		if (read_status()) {
			if (retry_count == 0) printf("\n");
			if (retry_count < 5) {
				printf("Failed to write page correctly! retrying\n");
				retry_count++;
				goto retry_all;
			}
			printf("Too many retries. Perhaps bad block?\n");
			// retry_count = 0;
		}
		retry_count = 0;
	}





	fcloseall();
	clock_t end = clock();
	printf("\nWrite done in %f seconds\n", (float)(end - start) / CLOCKS_PER_SEC);
}

int erase_blocks(int first_block_number, int number_of_blocks)
{
	int block, block_no, block_nbr, percent, i, n, retry_count;
	unsigned char id[5], id2[5];

	if (read_id(id) < 0)
		return -1;
	print_id(id);
	printf("if this ID is incorrect, press Ctrl-C NOW to abort (3s timeout)\n");
	sleep(3);

	printf("\nStart erasing...\n");
	clock_t start = clock();

	for (retry_count = 0, block = first_block_number; block < (first_block_number + number_of_blocks); block++) {

	  retry_all:
			
		block_nbr = block - first_block_number + 1;
		percent = (100 * block_nbr) / number_of_blocks;

		if (retry_count == 0) {
			printf("Erasing block n° %d at adress 0x%02X (block %d of %d), %d%%\r", block, block * BLOCK_SIZE, block_nbr, number_of_blocks, percent);
			fflush(stdout);
			// printf("Block address : %d (0x%02X)\n", block * BLOCK_SIZE, block * BLOCK_SIZE);
		}

	  retry:
		read_id(id2);
		if (memcmp(id, id2, 5) != 0) {
			printf("\nNAND ID has changed! retrying");
			goto retry;
		}

		send_eraseblock_command(block * 64); // 64 = pages per block
		while (GPIO_READ(N_READ_BUSY) == 0) {
			// printf("Busy\n");
			shortpause();
		}

		if (read_status()) {
			if (retry_count == 0) printf("\n");
			if (retry_count < 5) {
				printf("Failed to erase block correctly! retrying\n");
				retry_count++;
				goto retry_all;
			}
			printf("Too many retries. Perhaps bad block?\n");
			// retry_count = 0;
		}
		retry_count = 0;
	}

	clock_t end = clock();
	printf("\nErasing done in %f seconds\n", (float)(end - start) / CLOCKS_PER_SEC);

}

int main(int argc, char **argv)
{ 
	sunxi_gpio_init();
	
	for (int i=0; i < 25; i++)
		OUT_GPIO(SUNXI_GPC(i));

	INP_GPIO(N_READ_BUSY);

	OUT_GPIO(N_WRITE_PROTECT);
	GPIO_SET_1(N_WRITE_PROTECT);

	OUT_GPIO(N_READ_ENABLE);
	GPIO_SET_1(N_READ_ENABLE);

	OUT_GPIO(N_WRITE_ENABLE);
	GPIO_SET_1(N_WRITE_ENABLE);

	OUT_GPIO(COMMAND_LATCH_ENABLE);
	GPIO_SET_0(COMMAND_LATCH_ENABLE);

	OUT_GPIO(ADDRESS_LATCH_ENABLE);
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);

	OUT_GPIO(N_CHIP_ENABLE);
	GPIO_SET_0(N_CHIP_ENABLE);

	if (argc < 3) {
usage:
		
		printf("usage: %s <delay> <pagesize> <command> ...\n" \
			"\t<delay> is used to slow down operations (50 should work, increase in case of bad reads)\n" \
		    "Commands:\n" \
		    " read_id (no arguments)                        : read and decrypt chip ID\n" \
		    " read_full <page #> <# of pages> <output file> : read N pages including spare\n" \
		    " read_data <page #> <# of pages> <output file> : read N pages, discard spare\n" \
		    " write_full <page #> <# of pages> <input file> : write N pages, including spare\n" \
		    " write_data <page #> <# of pages> <input file> : write N pages, discard spare\n" \
		    " erase_blocks <block number> <# of blocks>     : erase N blocks\n\n" \
		    "Notes:\n" \
		    " This program assumes PAGE_SIZE == %d\n", \
			argv[0], atoi(argv[2]));
		sunxi_gpio_cleanup();
		return -1;
	}

	delay = atoi(argv[1]);
	if (delay < 20) {
		printf("delay must be >= 20\n");
		return -1;
	}

	PAGE_SIZE = atoi(argv[2]);
	if (PAGE_SIZE < 2048) {
		printf("pagesize looks weird: %i\n",PAGE_SIZE);
		return -1;
	}

	if (strcmp(argv[3], "read_id") == 0) {
		return read_id(NULL);
	}

	if (strcmp(argv[3], "read_full") == 0) {
		if (argc != 7) goto usage;
		if (atoi(argv[5]) <= 0) {
			printf("# of pages must be > 0\n");
			return -1;
		}
		return read_pages(atoi(argv[4]), atoi(argv[5]), argv[6], 1);
	}

	if (strcmp(argv[3], "read_data") == 0) {
		if (argc != 7) goto usage;
		if (atoi(argv[5]) <= 0) {
			printf("# of pages must be > 0\n");
			return -1;
		}
		return read_pages(atoi(argv[4]), atoi(argv[5]), argv[6], 0);
	}

	if (strcmp(argv[3], "write_full") == 0) {
		if (argc != 7) goto usage;
		if (atoi(argv[5]) <= 0) {
			printf("# of pages must be > 0\n");
			return -1;
		}
		return write_pages(atoi(argv[4]), atoi(argv[5]), argv[6]);
	}

	if (strcmp(argv[3], "erase_blocks") == 0) {
		if (argc != 6) goto usage;
		if (atoi(argv[5]) <= 0) {
			printf("# of blocks must be > 0\n");
			return -1;
		}
		return erase_blocks(atoi(argv[4]), atoi(argv[5]));
	}

	printf("unknown command '%s'\n", argv[3]);
	goto usage;
	sunxi_gpio_cleanup();
	return 0;
}


