module vga_driver_memory_double_buf	(
  // Clock Inputs
  input         CLOCK_50,    // 50MHz Input 1
  input         CLOCK2_50,   // 50MHz Input 2
  input         CLOCK3_50,   // 50MHz Input 3
  output        SMA_CLKOUT,  // External Clock Output
  input         SMA_CLKIN,   // External Clock Input

  // Push Button
  input  [3:0]  KEY,         // Pushbutton[3:0]

  // DPDT Switch
  input  [17:0] SW,          // Toggle Switch[17:0]

  // 7-SEG Display
  output [6:0]  HEX0,        // Seven Segment Digit 0
  output [6:0]  HEX1,        // Seven Segment Digit 1
  output [6:0]  HEX2,        // Seven Segment Digit 2
  output [6:0]  HEX3,        // Seven Segment Digit 3
  output [6:0]  HEX4,        // Seven Segment Digit 4
  output [6:0]  HEX5,        // Seven Segment Digit 5
  output [6:0]  HEX6,        // Seven Segment Digit 6
  output [6:0]  HEX7,        // Seven Segment Digit 7

  // LED
  output reg [8:0]  LEDG,        // LED Green[8:0]
  output [17:0] LEDR,        // LED Red[17:0]

  // UART
  output        UART_TXD,    // UART Transmitter
  input         UART_RXD,    // UART Receiver
  output        UART_CTS,    // UART Clear to Send
  input         UART_RTS,    // UART Reframe_buf_mem_quest to Send

  // IRDA
  input         IRDA_RXD,    // IRDA Receiver

  // SDRAM Interface
  inout  [31:0] DRAM_Dframe_buf_mem_q,     // SDRAM frame_buf_mem_data bus 32 Bits
  output [12:0] DRAM_ADDR,   // SDRAM frame_buf_mem_address bus 13 Bits
  output [1:0]  DRAM_BA,     // SDRAM Bank frame_buf_mem_address
  output [3:0]  DRAM_Dframe_buf_mem_qM,    // SDRAM Byte frame_buf_mem_data Mask 
  output        DRAM_RAS_N,  // SDRAM Row frame_buf_mem_address Strobe
  output        DRAM_CAS_N,  // SDRAM Column frame_buf_mem_address Strobe
  output        DRAM_CKE,    // SDRAM Clock Enable
  output        DRAM_CLK,    // SDRAM Clock
  output        DRAM_WE_N,   // SDRAM Write Enable
  output        DRAM_CS_N,   // SDRAM Chip Select

  // Flash Interface
  inout  [7:0]  FL_Dframe_buf_mem_q,       // FLASH frame_buf_mem_data bus 8 Bits
  output [22:0] FL_ADDR,     // FLASH frame_buf_mem_address bus 23 Bits
  output        FL_WE_N,     // FLASH Write Enable
  output        FL_WP_N,     // FLASH Write Protect / Programming Acceleration
  output        FL_RST_N,    // FLASH Reset
  output        FL_OE_N,     // FLASH Output Enable
  output        FL_CE_N,     // FLASH Chip Enable
  input         FL_RY,       // FLASH Ready/Busy output

  // SRAM Interface
  inout  [15:0] SRAM_Dframe_buf_mem_q,     // SRAM frame_buf_mem_data bus 16 Bits
  output [19:0] SRAM_ADDR,   // SRAM frame_buf_mem_address bus 20 Bits
  output        SRAM_OE_N,   // SRAM Output Enable
  output        SRAM_WE_N,   // SRAM Write Enable
  output        SRAM_CE_N,   // SRAM Chip Enable
  output        SRAM_UB_N,   // SRAM High-byte frame_buf_mem_data Mask 
  output        SRAM_LB_N,   // SRAM Low-byte frame_buf_mem_data Mask 

  // ISP1362 Interface
  inout  [15:0] OTG_frame_buf_mem_data,    // ISP1362 frame_buf_mem_data bus 16 Bits
  output [1:0]  OTG_ADDR,    // ISP1362 frame_buf_mem_address 2 Bits
  output        OTG_CS_N,    // ISP1362 Chip Select
  output        OTG_RD_N,    // ISP1362 Write
  output        OTG_WR_N,    // ISP1362 Read
  output        OTG_RST_N,   // ISP1362 Reset
  input  [1:0]  OTG_INT,     // ISP1362 Interrupts
  inout         OTG_FSPEED,  // USB Full Speed, 0 = Enable, Z = Disable
  inout         OTG_LSPEED,  // USB Low Speed,  0 = Enable, Z = Disable
  input  [1:0]  OTG_DREframe_buf_mem_q,    // ISP1362 DMA Reframe_buf_mem_quest
  output [1:0]  OTG_DACK_N,  // ISP1362 DMA Acknowledge

  // LCD Module 16X2
  inout  [7:0]  LCD_frame_buf_mem_data,    // LCD frame_buf_mem_data bus 8 bits
  output        LCD_ON,      // LCD Power ON/OFF
  output        LCD_BLON,    // LCD Back Light ON/OFF
  output        LCD_RW,      // LCD Read/Write Select, 0 = Write, 1 = Read
  output        LCD_EN,      // LCD Enable
  output        LCD_RS,      // LCD Command/frame_buf_mem_data Select, 0 = Command, 1 = frame_buf_mem_data

  // SD Card Interface
  inout  [3:0]  SD_DAT,      // SD Card frame_buf_mem_data
  inout         SD_CMD,      // SD Card Command Line
  output        SD_CLK,      // SD Card Clock
  input         SD_WP_N,     // SD Write Protect

  // EEPROM Interface
  output        EEP_I2C_SCLK, // EEPROM Clock
  inout         EEP_I2C_SDAT, // EEPROM frame_buf_mem_data

  // PS2
  inout         PS2_DAT,     // PS2 frame_buf_mem_data
  inout         PS2_CLK,     // PS2 Clock
  inout         PS2_DAT2,    // PS2 frame_buf_mem_data 2 (use for 2 devices and y-cable)
  inout         PS2_CLK2,    // PS2 Clock 2 (use for 2 devices and y-cable)

  // I2C  
  inout         I2C_SDAT,    // I2C frame_buf_mem_data
  output        I2C_SCLK,    // I2C Clock

  // Audio CODEC
  inout         AUD_ADCLRCK, // Audio CODEC ADC LR Clock
  input         AUD_ADCDAT,  // Audio CODEC ADC frame_buf_mem_data
  inout         AUD_DACLRCK, // Audio CODEC DAC LR Clock
  output        AUD_DACDAT,  // Audio CODEC DAC frame_buf_mem_data
  inout         AUD_BCLK,    // Audio CODEC Bit-Stream Clock
  output        AUD_XCK,     // Audio CODEC Chip Clock

  // Ethernet Interface (88E1111)
  input         ENETCLK_25,    // Ethernet clock source

  output        ENET0_GTX_CLK, // GMII Transmit Clock 1
  input         ENET0_INT_N,   // Interrupt open drain output 1
  input         ENET0_LINK100, // Parallel LED output of 100BASE-TX link 1
  output        ENET0_MDC,     // Management frame_buf_mem_data clock ref 1
  inout         ENET0_MDIO,    // Management frame_buf_mem_data 1
  output        ENET0_RST_N,   // Hardware Reset Signal 1
  input         ENET0_RX_CLK,  // GMII and MII receive clock 1
  input         ENET0_RX_COL,  // GMII and MII collision 1
  input         ENET0_RX_CRS,  // GMII and MII carrier sense 1
  input   [3:0] ENET0_RX_frame_buf_mem_data, // GMII and MII receive frame_buf_mem_data 1
  input         ENET0_RX_DV,   // GMII and MII receive frame_buf_mem_data valid 1
  input         ENET0_RX_ER,   // GMII and MII receive error 1
  input         ENET0_TX_CLK,  // MII Transmit clock 1
  output  [3:0] ENET0_TX_frame_buf_mem_data, // MII Transmit frame_buf_mem_data 1
  output        ENET0_TX_EN,   // GMII and MII transmit enable 1
  output        ENET0_TX_ER,   // GMII and MII transmit error 1

  output        ENET1_GTX_CLK, // GMII Transmit Clock 1
  input         ENET1_INT_N,   // Interrupt open drain output 1
  input         ENET1_LINK100, // Parallel LED output of 100BASE-TX link 1
  output        ENET1_MDC,     // Management frame_buf_mem_data clock ref 1
  inout         ENET1_MDIO,    // Management frame_buf_mem_data 1
  output        ENET1_RST_N,   // Hardware Reset Signal 1
  input         ENET1_RX_CLK,  // GMII and MII receive clock 1
  input         ENET1_RX_COL,  // GMII and MII collision 1
  input         ENET1_RX_CRS,  // GMII and MII carrier sense 1
  input   [3:0] ENET1_RX_frame_buf_mem_data, // GMII and MII receive frame_buf_mem_data 1
  input         ENET1_RX_DV,   // GMII and MII receive frame_buf_mem_data valid 1
  input         ENET1_RX_ER,   // GMII and MII receive error 1
  input         ENET1_TX_CLK,  // MII Transmit clock 1
  output  [3:0] ENET1_TX_frame_buf_mem_data, // MII Transmit frame_buf_mem_data 1
  output        ENET1_TX_EN,   // GMII and MII transmit enable 1
  output        ENET1_TX_ER,   // GMII and MII transmit error 1

  // Expansion Header
  inout   [6:0] EX_IO,       // 14-pin GPIO Header
  inout  [35:0] GPIO,        // 40-pin Expansion header

  // TV Decoder
  input  [8:0]  TD_frame_buf_mem_data,     // TV Decoder frame_buf_mem_data
  input         TD_CLK27,    // TV Decoder Clock Input
  input         TD_HS,       // TV Decoder H_SYNC
  input         TD_VS,       // TV Decoder V_SYNC
  output        TD_RESET_N,  // TV Decoder Reset

  // VGA
  output        VGA_CLK,     // VGA Clock
  output        VGA_HS,      // VGA H_SYNC
  output        VGA_VS,      // VGA V_SYNC
  output        VGA_BLANK_N, // VGA BLANK
  output        VGA_SYNC_N,  // VGA SYNC
  output reg [7:0]  VGA_R,       // VGA Red[9:0]
  output reg [7:0]  VGA_G,       // VGA Green[9:0]
  output reg [7:0]  VGA_B       // VGA Blue[9:0]
);

  // Turn off all displays.
  assign HEX0 = 7'h7F;
  assign HEX1 = 7'h7F;
  assign HEX2 = 7'h7F;
  assign HEX3 = 7'h7F;
  assign HEX4 = 7'h7F;
  assign HEX5 = 7'h7F;
  assign HEX6 = 7'h7F;
  assign HEX7 = 7'h7F;

  // Set all GPIO to tri-state.
  assign GPIO = 36'hzzzzzzzzz;
  
  // Disable audio codec.
  assign AUD_DACDAT = 1'b0;
  assign AUD_XCK    = 1'b0;

  // Disable DRAM
  assign DRAM_ADDR  = 13'h0;
  assign DRAM_BA  = 2'b0;
  assign DRAM_CAS_N = 1'b1;
  assign DRAM_CKE   = 1'b0;
  assign DRAM_CLK   = 1'b0;
  assign DRAM_CS_N  = 1'b1;
  assign DRAM_Dframe_buf_mem_q    = 32'hzzzz;
  assign DRAM_Dframe_buf_mem_qM   = 4'b0;
  assign DRAM_RAS_N = 1'b1;
  //assign DRAM_UDframe_buf_mem_qM  = 1'b0;
  assign DRAM_WE_N  = 1'b1;

  // Disable flash.
  assign FL_ADDR  = 23'h0;
  assign FL_CE_N  = 1'b1;
  assign FL_Dframe_buf_mem_q    = 8'hzz;
  assign FL_OE_N  = 1'b1;
  assign FL_RST_N = 1'b1;
  assign FL_WE_N  = 1'b1;
  assign FL_WP_N  = 1'b0;

  // Disable LCD.
  assign LCD_BLON = 1'b0;
  assign LCD_frame_buf_mem_data = 8'hzz;
  assign LCD_EN   = 1'b0;
  assign LCD_ON   = 1'b0;
  assign LCD_RS   = 1'b0;
  assign LCD_RW   = 1'b0;

  // Disable OTG.
  assign OTG_ADDR    = 2'h0;
  assign OTG_CS_N    = 1'b1;
  assign OTG_DACK_N  = 2'b11;
  assign OTG_FSPEED  = 1'b1;
  assign OTG_frame_buf_mem_data    = 16'hzzzz;
  assign OTG_LSPEED  = 1'b1;
  assign OTG_RD_N    = 1'b1;
  assign OTG_RST_N   = 1'b1;
  assign OTG_WR_N    = 1'b1;

  // Disable SD
  assign SD_DAT = 4'bzzzz;
  assign SD_CLK = 1'b0;
  assign SD_CMD = 1'b0;

  // Disable SRAM.
  assign SRAM_ADDR = 20'h0;
  assign SRAM_CE_N = 1'b1;
  assign SRAM_Dframe_buf_mem_q   = 16'hzzzz;
  assign SRAM_LB_N = 1'b1;
  assign SRAM_OE_N = 1'b1;
  assign SRAM_UB_N = 1'b1;
  assign SRAM_WE_N = 1'b1;

  // Disable all other peripherals.
  assign I2C_SCLK   = 1'b0;
  //assign TD_RESET_N = 1'b0;
  assign UART_TXD   = 1'b0;
  assign UART_CTS   = 1'b0;
// DONE STANDARD PORT DECLARATION ABOVE

/* HANDLE SIGNALS FOR CIRCUIT */
wire clk;
wire rst;

assign clk = CLOCK_50;
assign rst = KEY[0];

wire [17:0]SW_db;

debounce_switches db(
.clk(clk),
.rst(rst),
.SW(SW), 
.SW_db(SW_db)
);
/* -------------------------------- */

/* DEBUG SIGNALS */
//assign LEDR[0] = active_pixels;

/* -------------------------------- */
// VGA DRIVER
wire active_pixels; // is on when we're in the active draw space
wire frame_done;

wire [9:0]x; // current x
wire [9:0]y; // current y - 10 bits = 1024 ... a little bit more than we need

vga_driver the_vga(
.clk(clk),
.rst(rst),

.vga_clk(VGA_CLK),

.hsync(VGA_HS),
.vsync(VGA_VS),

.active_pixels(active_pixels),
.frame_done(frame_done),

.xPixel(x),
.yPixel(y),

.VGA_BLANK_N(VGA_BLANK_N),
.VGA_SYNC_N(VGA_SYNC_N)
);

always @(*)
begin
	/* This part is for taking the memory value read out from memory and sending to the VGA */
	if (S == RFM_INIT_WAIT || S == RFM_INIT_START || S == RFM_DRAWING)
	begin
		{VGA_R, VGA_G, VGA_B} = read_buf_mem_q;
	end
	else // BLACK OTHERWISE
		{VGA_R, VGA_G, VGA_B} = 24'hFFC0CB;
end

/* -------------------------------- */
/* 	FSM to control the writing and reading of the framebuffer. */
reg [15:0]i;
reg [7:0]S;
reg [7:0]NS;
parameter 
	START 			= 8'd0,
	// W2M is write to memory
	W2M_INIT 		= 8'd1,
	W2M_COND 		= 8'd2,
	W2M_INC 		= 8'd3,
	W2M_DONE 		= 8'd4,
	// The RFM = READ_FROM_MEMOERY reading cycles
	RFM_INIT_START 	= 8'd5,
	RFM_INIT_WAIT 	= 8'd6,
	RFM_DRAWING 	= 8'd7,
	ERROR 			= 8'hFF;

parameter MEMORY_SIZE = 16'd19200; // 160*120 // Number of memory spots ... highly reduced since memory is slow
parameter PIXEL_VIRTUAL_SIZE = 16'd4; // Pixels per spot - therefore 4x4 pixels per memory location

/* ACTUAL VGA RESOLUTION */
parameter VGA_WIDTH = 16'd640; 
parameter VGA_HEIGHT = 16'd480;

/* Our reduced RESOLUTION */
parameter VIRTUAL_PIXEL_WIDTH = VGA_WIDTH/PIXEL_VIRTUAL_SIZE; // 160
parameter VIRTUAL_PIXEL_HEIGHT = VGA_HEIGHT/PIXEL_VIRTUAL_SIZE; // 120

reg         pixel_start;
wire        pixel_done;
reg [31:0]  pixel_x;
reg [31:0]  pixel_y;
wire [23:0] pixel_color;
pixel_shader frag(SW, clk, pixel_start, stored_rand, pixel_x, pixel_y, pixel_color, pixel_done);

reg[255:0] stored_rand;
wire[255:0] randvals;

random randomerre(clk, randvals);

/* Calculate NS */
always @(*)
	case (S)
		START: 	
			if (KEY[1] == 1'b0 || 1) // Basically, if you hold down KEY[1] you will initialize the file with the FSM, otherwise you skip and have Rick as your mif initialization
				NS = W2M_INIT;
			else	
				NS = W2M_DONE;
		W2M_INIT: NS = W2M_COND;
		W2M_COND:
			if (i < MEMORY_SIZE)
        if (pixel_done)
				  NS = W2M_INC;
        else
          NS = W2M_COND;
			else
				NS = W2M_DONE;
		W2M_INC: NS = W2M_COND;
		W2M_DONE: 
			if (frame_done == 1'b1)
				NS = RFM_INIT_START;
			else
				NS = W2M_DONE;
	
		RFM_INIT_START: NS = RFM_INIT_WAIT;
		RFM_INIT_WAIT: 
			if (frame_done == 1'b0)
				NS = RFM_DRAWING;
			else	
				NS = RFM_INIT_WAIT;
		RFM_DRAWING:
			if (frame_done == 1'b1)
				NS = RFM_INIT_START;
			else
				NS = RFM_DRAWING;
		default:	NS = ERROR;
	endcase

always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
			S <= START;
	end
	else
	begin
			S <= NS;
	end
end

/* 
The code goes through a write phase (after reset) and an endless read phase once writing is done.

The W2M (write to memory) code is roughly:
for (i = 0; i < MEMORY_SIZE; i++)
	mem[i] = color // where color is a hard coded on off is on SW[16:14] for {R, G, B}

The RFM (read from memory) is synced with the VGA display (via vga_driver modules x and y) which goes row by row
for (y = 0; y < 480; y++) // height
	for (x = 0; x < 640; x++) // width
		color = mem[(x/4 * VP_HEIGHT) + j/4] reads from one of the buffers while you can write to the other buffer
*/
always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
		write_buf_mem_address <= 14'd0;
		write_buf_mem_data <= 24'd0;
		write_buf_mem_wren <= 1'd0;
		i <= 16'd0;
		wr_id <= MEM_INIT_WRITE;
	end
	else
	begin
		case (S)
			START:
			begin
				write_buf_mem_address <= 14'd0;
				write_buf_mem_data <= 24'd0;
				write_buf_mem_wren <= 1'd0;
				i <= 16'd0;
				wr_id <= MEM_INIT_WRITE;
			end
			W2M_INIT:
			begin
				write_buf_mem_address <= 14'd0;
				write_buf_mem_data <= 24'd0;
				write_buf_mem_wren <= 1'd1;
				i <= 16'd0;
        pixel_start <= 0;
        pixel_x <= 0;
        pixel_y <= 0;

        // initalize random number generator
        stored_rand <= randvals;
			end
			W2M_COND:
			begin
        pixel_start <= 1;
			end
			W2M_INC: 
			begin
				i <= i + 1'b1;
				write_buf_mem_address <= write_buf_mem_address + 1'b1;
				/* INITIALIZE to a solid color - IF SW[16-14] all off then = BLACK...all on = WHITE */
				// write_buf_mem_data <= {SW[16]*8'hFF, SW[15]*8'hFF, SW[14]*8'hFF}; // red, blue, and green done in the combinational part below	
        write_buf_mem_data <= pixel_color;
        pixel_start <= 0;
        pixel_x <= i % VIRTUAL_PIXEL_HEIGHT;
        pixel_y <= i / VIRTUAL_PIXEL_WIDTH;

        // Rendering LED for timing purpose
        LEDG[0] <= pixel_x[0];
        LEDG[1] <= pixel_x[1];
        LEDG[2] <= pixel_x[2];
        LEDG[3] <= pixel_x[3];
        LEDG[4] <= pixel_x[4];
        LEDG[5] <= pixel_x[5];
        LEDG[6] <= pixel_x[6];
        LEDG[7] <= pixel_x[7];

        // if (pixel_x != VIRTUAL_PIXEL_WIDTH) // normal scanline
        // begin
        //     pixel_x <= pixel_x + 1;
        // end
        // else // at the end of the scanline
        // begin
        //   pixel_x <= 0;
        //   pixel_y <= pixel_y + 1;
        // end
			end
			W2M_DONE: write_buf_mem_wren <= 1'd0; // turn off writing to memory
			RFM_INIT_START: 
			begin        
				write_buf_mem_wren <= 1'd0; // turn off writing to memory
				
				/* swap the buffers after each frame...the double buffer */
				if (wr_id == MEM_INIT_WRITE)
					wr_id <= MEM_M0_READ_M1_WRITE;
				else if (wr_id == MEM_M0_READ_M1_WRITE)
					wr_id <= MEM_M0_WRITE_M1_READ;
				else
					wr_id <= MEM_M0_READ_M1_WRITE;
								
				if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1) // or use the active_pixels signal
					read_buf_mem_address <= (x/PIXEL_VIRTUAL_SIZE) * VIRTUAL_PIXEL_HEIGHT + (y/PIXEL_VIRTUAL_SIZE) ;
			end
			RFM_INIT_WAIT:
			begin
				if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1) // or use the active_pixels signal
					read_buf_mem_address <= (x/PIXEL_VIRTUAL_SIZE) * VIRTUAL_PIXEL_HEIGHT + (y/PIXEL_VIRTUAL_SIZE) ;
			end
			RFM_DRAWING:
			begin		
				if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1)
					read_buf_mem_address <= (x/PIXEL_VIRTUAL_SIZE) * VIRTUAL_PIXEL_HEIGHT + (y/PIXEL_VIRTUAL_SIZE) ;

        // if (y < VGA_HEIGHT-1 && x < VGA_WIDTH-1 && SW[17] == 1'b1)
				// begin
				// 	write_buf_mem_address <= (y) * VIRTUAL_PIXEL_HEIGHT + (x);
				// 	// write_buf_mem_data <= {SW[16]*8'hFF, SW[15]*8'hFF, SW[14]*8'hFF};
				// 	write_buf_mem_data <= pixel_color;
				// 	write_buf_mem_wren <= 1'b1;
				// end
				
				// if (SW[17] == 1'b1) // When you turn on SWITCH 17, you will draw a RGB pixel (depending on SW[16-14] at location x = SW[13:7] and y = SW[6:0]
				// begin
				// 	write_buf_mem_address <= (SW[13:7]) * VIRTUAL_PIXEL_HEIGHT + (SW[6:0]);
				// 	write_buf_mem_data <= {SW[16]*8'hFF, SW[15]*8'hFF, SW[14]*8'hFF};
				// 	write_buf_mem_wren <= 1'b1;
				// end
				else
					write_buf_mem_wren <= 1'b0;
			end	
		endcase
	end
end

/* -------------------------------- */
/* MEMORY to STORE a MINI framebuffer.  Problem is the FPGA's on-chip memory can't hold an entire frame 640*480 , so some
form of compression is needed.  I show a simple compress the image to 16 pixels or a 4 by 4, but this memory
could handle more */
reg [14:0] frame_buf_mem_address0;
reg [23:0] frame_buf_mem_data0;
reg frame_buf_mem_wren0;
wire [23:0]frame_buf_mem_q0;

vga_frame vga_memory0(
	frame_buf_mem_address0,
	clk,
	frame_buf_mem_data0,
	frame_buf_mem_wren0,
	frame_buf_mem_q0);
	
reg [14:0] frame_buf_mem_address1;
reg [23:0] frame_buf_mem_data1;
reg frame_buf_mem_wren1;
wire [23:0]frame_buf_mem_q1;


vga_frame vga_memory1(
	frame_buf_mem_address1,
	clk,
	frame_buf_mem_data1,
	frame_buf_mem_wren1,
	frame_buf_mem_q1);

/* signals that will be combinationally swapped in each cycle */
reg [1:0]wr_id;	
reg [14:0] write_buf_mem_address;
reg [23:0] write_buf_mem_data;
reg write_buf_mem_wren;
reg [23:0]read_buf_mem_q;
reg [14:0] read_buf_mem_address;

parameter MEM_INIT_WRITE = 2'd0,
		  MEM_M0_READ_M1_WRITE = 2'd1,
		  MEM_M0_WRITE_M1_READ = 2'd2,
		  MEM_ERROR = 2'd3;

/* signals that will be combinationally swapped in each buffer output that swaps between wr_id where wr_id = 0 is for initialize */
always @(*)
begin
	if (wr_id == MEM_INIT_WRITE) // WRITING to BOTH
	begin
		frame_buf_mem_address0 = write_buf_mem_address;
		frame_buf_mem_data0 = write_buf_mem_data;
		frame_buf_mem_wren0 = write_buf_mem_wren;
		frame_buf_mem_address1 = write_buf_mem_address;
		frame_buf_mem_data1 = write_buf_mem_data;
		frame_buf_mem_wren1 = write_buf_mem_wren;
		
		read_buf_mem_q = frame_buf_mem_q1; // doesn't matter
	end
	else if (wr_id == MEM_M0_WRITE_M1_READ) // WRITING to MEM 0 READING FROM MEM 1
	begin
		// MEM 0 - WRITE
		frame_buf_mem_address0 = write_buf_mem_address;
		frame_buf_mem_data0 = write_buf_mem_data;
		frame_buf_mem_wren0 = write_buf_mem_wren;
		// MEM 1 - READ
		frame_buf_mem_address1 = read_buf_mem_address;
		frame_buf_mem_data1 = 24'd0;
		frame_buf_mem_wren1 = 1'b0;
		read_buf_mem_q = frame_buf_mem_q1;
	end
	else //if (wr_id == MEM_M0_READ_M1_WRITE) WRITING to MEM 1 READING FROM MEM 0
	begin
		// MEM 0 - READ
		frame_buf_mem_address0 = read_buf_mem_address;
		frame_buf_mem_data0 = 24'd0;
		frame_buf_mem_wren0 = 1'b0;
		read_buf_mem_q = frame_buf_mem_q0;
		// MEM 1 - WRITE
		frame_buf_mem_address1 = write_buf_mem_address;
		frame_buf_mem_data1 = write_buf_mem_data;
		frame_buf_mem_wren1 = write_buf_mem_wren;
	end
end

endmodule