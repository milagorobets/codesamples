`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 			University of Calgary
// Engineer: 			Mila Gorobets (www.milagorobets.com)
// 
// Create Date:    	14:13:13 11/10/2014 
// Design Name: 
// Module Name:    	SPI_INSTRUCTION_BLOCK_2 
// Project Name: 	 	INTERNAL BUS SNIFFER
// Target Devices: 	ZYNQ7020, ZEDBOARD
// Tool versions: 
// Description: 
//    This file provides description for the SPI-driven CPU. Included is the
//    instruction set.
// Dependencies: 
//    No dependencies.
// Revision: 2
// Revision 0.01 - File Created
// Additional Comments: 
// 						Nov 10: 	Rewriting the functions to remove odd latency in
//										implementation.
//
//////////////////////////////////////////////////////////////////////////////////

// IMPORTANT BITS
`define 		SAFE_IDLE_REG			0
`define 		SPI_TX_REG				5
`define 		SPI_RX_REG				6
`define 		TRUE 						1'b1
`define		FALSE 					1'b0

module SPI_INSTRUCTION_BLOCK_2(
		clk,
		reset,
		ABB,												// Address bus buffer
		inDBB,												// Data bus buffer
		outDBB,
		spi_done,
		spi_in,
		rw,
		cs,
		rABB,
		wABB
	 );
	 
// CONFIGURABLE PARAMETERS
parameter SPI_WIDTH 					= 16;
parameter ADDR_BUS_WIDTH			= 32;
parameter DATA_BUS_WIDTH 			= 32;
parameter SYNC_VALUE 				= 5;
parameter MAX_COUNT_LENGTH 		= 32;
parameter BANK_DATA_WIDTH 			= 32;
parameter BANK_SIZE 					= 6;												// Bits required to represent all addresses
parameter CTRL_REG_LENGTH 			= 16;
parameter OPCODE_LENGTH				= 8;
parameter CMD_FIELD					= 5;

// BLOCK INPUTS
input 										clk, reset;			// Regular signals
input 		[SPI_WIDTH-1:0]			spi_in;				// Data arriving via SPI
input			[DATA_BUS_WIDTH-1:0] 	inDBB;				// Data to take off Data Bus (from memory)
input											spi_done;			// Signals end of transmission

// BLOCK OUTPUTS
output										rw, cs;				// For controlling the memory (register bank)
output 		[ADDR_BUS_WIDTH-1:0]		ABB;					// Data to place on Address Bus
output		[DATA_BUS_WIDTH-1:0] 	outDBB;				// Data to place on Data Bus (going to memory)
output										rABB, wABB;			// Signals what kind of memory access is happening (for WU)

// INTERNAL SIGNAL
reg			[SPI_WIDTH-1:0]			r_spi_in;
reg			[BANK_DATA_WIDTH-1:0]	r_rx_data;			// Holds received data
reg			[BANK_DATA_WIDTH-1:0] 	r_tx_data, 			// Holds SPI_WIDTH bits of data to send
												r_tx_data_full;	// Holds entire fetched block of data
reg			[BANK_DATA_WIDTH-1:0]	r_wData_RX; 		// Data blocks to put out onto the outDBB					 		
reg			[SPI_WIDTH-1:0]			r_wData_TX;			// Only one is selected
reg			[BANK_SIZE-1:0]			r_wAddr_RX;
reg			[BANK_SIZE-1:0]			r_rAddr;
reg			[BANK_SIZE-1:0]			r_ABB;
reg			[DATA_BUS_WIDTH-1:0]		r_DBB;
reg 											rx_rw, 				// Receive r/w signal to memory
												rx_cs;				// Receive chip select signal to memory
reg 											tx_rw, 				// Transmit r/w signal to memory
												tx_cs;				// Transmit chip select signal to memory
reg 											whichABB;			// Selects which ABB register writes to Address Bus
reg											ABBr, ABBw;			// Signals whether this is a read or write access
reg											tx_data_go, rx_data_go;
reg											rx_data_done, tx_data_done;
reg			[2:0]							rx_data_state, rx_data_next;
reg			[2:0]							tx_data_state, tx_data_next;
reg [2:0]									rx_data_counter;	// Counts number of RX transmissions
reg [2:0]									tx_data_counter;	// Counts number of TX transmissions
wire 			[2:0]							write_now;
reg											write_now_tx, write_now_rx;
reg											read_now_tx, r_read_done;
reg 			[BANK_SIZE-1:0] 			r_rx_loc, r_tx_loc;// Addresses to read/write from/to
wire 											done_re_d, done_fe;
reg			[2:0]							r_done;
// State machines
reg			[2:0] 						main_state, main_next;
reg			[1:0]							write_state, read_state;
// State machine states
parameter 	[2:0] 						MAIN_SYNC 		= 3'b000,
												MAIN_CMD_WRITE	= 3'b001,
												MAIN_CMD_READ	= 3'b110,
												MAIN_DATA		= 3'b010,
												MAIN_WAIT		= 3'b011,
												MAIN_SYNC_2		= 3'b100,
												MAIN_CMD_WAIT	= 3'b101;
parameter 	[2:0]							RX_DATA_IDLE 	= 3'b000,
												RX_DATA_ACC		= 3'b001,
												RX_DATA_DUMP 	= 3'b010,
												RX_DATA_R		= 3'b011,
												RX_DATA_WRITE	= 3'b100,
												RX_DATA_START	= 3'b101,
												RX_DATA_WAIT	= 3'b110,
												RX_DATA_START_WAIT = 3'b111;
parameter   [2:0]							TX_DATA_IDLE 	= 3'b000,
												TX_DATA_FETCH  = 3'b001,
												TX_DATA_SEND   = 3'b010,
												TX_DATA_DONE	= 3'b011,
												TX_DATA_WAIT 	= 3'b100,
												TX_DATA_PAUSE	= 3'b101;
parameter	[1:0]							WRITE_ADDRESS	= 2'b00,
												WRITE_DATA_RX	= 2'b10,
												WRITE_DATA_TX	= 2'b11;
parameter 	[1:0]							READ_ADDRESS	= 2'b00,
												READ_DATA		= 2'b01;

// ----------------------------------------------------------------------------------------
// WIRE CONNECTIONS
assign 	rw = ~(rx_rw ^ tx_rw);								// XNOR: one or the other, not both 
assign 	cs = ~(rx_cs ^ tx_cs);
assign 	ABB = (whichABB)?{{ADDR_BUS_WIDTH-BANK_SIZE{1'b0}},r_ABB}:
					{{ADDR_BUS_WIDTH-BANK_SIZE{1'b0}},r_rAddr};					// Picks an ABB (read or write ABB)
assign 	outDBB = r_DBB;										// Connect outgoing data bus
assign 	rABB = ABBr;											// Access is a read
assign 	wABB = ABBw;											// Access is a write
assign 	write_now 	= {write_now_tx, write_now_rx};

// ----------------------------------------------------------------------------------------
// EDGE MONITORING
// (1)	Outputs only the rising edge of the zero pulse
//			Lasts one clock cycle
always @(negedge clk)
	r_done <= {r_done[1:0], spi_done};
assign done_re_d 	= (r_done == 3'b011);					// Delayed rising edge (1 clock cycle)
assign done_fe		= (r_done == 3'b100);					// Falling edge

// ----------------------------------------------------------------------------------------
// PROCESSES
// (1) 	Bring value in from SPI RX buffer on negative clock edge
always @(negedge clk) begin
	r_spi_in		<= 						spi_in; 				// Save input register
end

// (2)	State machine for general operation: actions and assignments. 
//			Watches the spi_done signal toggling
always @(posedge clk or negedge reset) begin
	if (!reset) begin
		main_state <= MAIN_SYNC;		
	end
	else begin
		case (main_state)
			MAIN_SYNC: 
						begin
							if ((r_spi_in == SYNC_VALUE) & (done_re_d)) begin		// Waits until SYNC arrives
								main_state <= MAIN_SYNC_2;
							end
							else begin 
								main_state <= MAIN_SYNC;
							end
						end
			MAIN_SYNC_2: 
						begin
							if (!done_re_d) main_state <= MAIN_CMD_WAIT;	
							else main_state <= MAIN_SYNC_2;
						end
			MAIN_CMD_WAIT:
						begin
							if (done_re_d & r_spi_in[SPI_WIDTH-1]) begin				// Check the R/W bit
								main_state <= MAIN_CMD_READ;
							end
							else if (done_re_d & (~r_spi_in[SPI_WIDTH-1])) begin	// Check the R/W bit
								main_state <= MAIN_CMD_WRITE;
							end
							else main_state <= MAIN_CMD_WAIT;
						end
			MAIN_CMD_READ:
						begin
							if (done_fe) main_state <= MAIN_WAIT;
							else main_state <= MAIN_CMD_READ;
						end
			MAIN_CMD_WRITE:
						begin
							if (done_fe) main_state <= MAIN_WAIT;
							else main_state <= MAIN_CMD_WRITE;
						end
			MAIN_WAIT:
						begin
							if (rx_data_done | tx_data_done) begin
								if (rx_data_done) begin 
									main_state 		<= MAIN_DATA; 							// Write received value back							
								end
								else if (tx_data_done) begin
									main_state 		<= MAIN_SYNC;
								end
								else main_state 	<= MAIN_WAIT;						// This block waits for the transmission to finish
							end
							else begin
								main_state <= MAIN_WAIT;
							end
						end
			MAIN_DATA:
						begin
							main_state 	<= MAIN_SYNC;	
						end
			default: begin
						end
		endcase
	end
end

// (3) 	Controls sending and receiving state machines
always @(main_state) begin										
	case (main_state)
		MAIN_SYNC:	begin
							rx_data_go 			= `FALSE;
							tx_data_go 			= `FALSE;
						end
		MAIN_SYNC_2: begin															// SM reacts to both edges, so have to skip
							rx_data_go 			= `FALSE;
							tx_data_go 			= `FALSE;															// the falling edge
						end
		MAIN_CMD_READ: begin
							tx_data_go = `TRUE;										// Sending data: R
							rx_data_go = `FALSE;		
						end
		MAIN_CMD_WRITE:begin	
							rx_data_go = `TRUE;										// Receiving data: !W
							tx_data_go = `FALSE;
							end
		MAIN_WAIT:	begin
							rx_data_go 			= `FALSE;
							tx_data_go 			= `FALSE;
						end
		MAIN_DATA:	begin
							rx_data_go 			= `FALSE;
							tx_data_go			= `FALSE;						
						end
		default: begin	
						rx_data_go 			= `FALSE;
						tx_data_go 			= `FALSE;    							// What happened here? Shouldn't quit the SM
					end	
	endcase
end

// (4)	Receiving state machine - writes data to a selected register
always @(posedge clk or negedge reset) begin
	if (!reset) begin
		rx_data_state 		<= RX_DATA_IDLE;
		r_rx_loc				<= 0;
		r_rx_data			<= 0;
		rx_data_counter	<= 0;
		r_wAddr_RX			<= 0;
		r_wData_RX			<= 0;
	end
	else begin
		case (rx_data_state) 
			RX_DATA_IDLE: 	begin
									write_now_rx 	<= 0;
									if (rx_data_go) begin
										rx_data_counter 	<= r_spi_in[1:0]+1'b1;			// Number of transmissions that follow
										r_rx_loc				<= r_spi_in[7:2];				// Extract register address
										rx_data_state 	<= RX_DATA_START;
									end
									else begin
										rx_data_state	<= RX_DATA_IDLE;
										
									end
								end
			RX_DATA_START: begin
									rx_data_state	<= RX_DATA_START_WAIT;
								end
			RX_DATA_START_WAIT: begin
									if (done_re_d) begin
										if (rx_data_counter == 0) begin
											rx_data_state	<= RX_DATA_DUMP;
										end
										else begin
											rx_data_state 	<= RX_DATA_ACC;
											rx_data_counter 	<= rx_data_counter - 1'b1;	// Decrement the counter
										end
									end
									else begin
										rx_data_state 	<= RX_DATA_START_WAIT;
									end
								end
			RX_DATA_ACC: 	begin
									r_rx_data			<= {r_rx_data[BANK_DATA_WIDTH-SPI_WIDTH-1:0],
															r_spi_in};							// For the full data value by shifting
									if (rx_data_counter > 0) 
										rx_data_state <= RX_DATA_WAIT;					// Continue if there is more to receive
									else rx_data_state <= RX_DATA_WRITE;				// Stop and write to register
								end
			RX_DATA_WAIT: 	begin
									if (done_re_d) begin
										rx_data_state <= RX_DATA_ACC;						// Return to forming the data
										rx_data_counter 	<= rx_data_counter - 1'b1;		// Decrement the counter
									end
									else rx_data_state <= RX_DATA_WAIT;					// Wait until end of transmission
								end
			RX_DATA_WRITE: begin
									r_wAddr_RX		<= r_rx_loc[BANK_SIZE-1:0];		// Location to write the data to
									r_wData_RX 		<= r_rx_data;
									write_now_rx	<= 0;
									rx_data_state 	<= RX_DATA_DUMP;
								end
			RX_DATA_DUMP: 	begin
									rx_data_state 	<= RX_DATA_IDLE;
									write_now_rx 	<= 1;										// Pulse the write signal
								end
			default: begin
			
						end		
		endcase	
	end
end

// (5)	Signals end of receiving operation
always @(rx_data_state) begin
	case (rx_data_state)
		RX_DATA_IDLE:	begin								
								rx_data_done = `FALSE;
							end
		RX_DATA_START: begin						
								rx_data_done = `FALSE;
							end
		RX_DATA_ACC:	begin
								rx_data_done = `FALSE;
							end
		RX_DATA_WRITE: begin
								rx_data_done = `TRUE;
							end
		RX_DATA_DUMP: 	begin
								rx_data_done = `TRUE;
							end
		default:			begin
								rx_data_done = `FALSE;
							end
		endcase
end

// (6) 	Handles writing to memory
always @(posedge clk or negedge reset) begin
	if (!reset) begin
		write_state <= WRITE_ADDRESS;
		r_ABB <= 0;
		r_DBB <= 0;
		rx_rw <= 1;
		rx_cs <= 1;
	end
	else begin
		case (write_state)
			WRITE_ADDRESS: begin
									case (write_now)
										2'b01: 	begin
														r_ABB 		<= r_wAddr_RX;
														rx_cs 		<= 0;						// Select memory
														rx_rw 		<= 0;						// Write operation
														ABBw 			<= 0;						// Do not signal WU yet
														r_DBB			<= r_wData_RX;
														write_state <= WRITE_DATA_RX; 
													end
										2'b10: 	begin
														r_ABB 		<= 5;
														rx_cs 		<= 0;						// Select memory
														rx_rw 		<= 0;						// Write operation
														ABBw 			<= 0;						// Do not signal WU yet
														r_DBB			<= {{(DATA_BUS_WIDTH-SPI_WIDTH){1'b0}},r_wData_TX};
														write_state <= WRITE_DATA_TX; 
													end
										default: begin
														rx_rw 		<= 1;
														rx_cs 		<= 1;
														r_ABB 		<= 0;						// Park at register 0
														r_DBB			<= 0;
														ABBw 			<= 0;
														write_state <= WRITE_ADDRESS;		// IDLE in this state
													end
									endcase
								end
			WRITE_DATA_RX: begin
									r_DBB			<= r_wData_RX;
									rx_cs 		<= 0;											// Select memory
									rx_rw 		<= 0;											// Write operation
									ABBw 			<= 1;											// Signal the WU to check
									write_state <= WRITE_ADDRESS;
								end
			WRITE_DATA_TX: begin
									r_DBB			<= {{(DATA_BUS_WIDTH-SPI_WIDTH){1'b0}},r_wData_TX};
									rx_cs 		<= 0;											// Select memory
									rx_rw 		<= 0;											// Write operation
									ABBw 			<= 1;											// Signal the WU to check
									write_state <= WRITE_ADDRESS;
								end	
			default: begin
							r_ABB 		<= 0;													// Park at register 0
							r_DBB			<= 0;
							ABBw 			<= 0;
							rx_rw 		<= 1;
							rx_cs 		<= 1;
						end
		endcase	
	end
end

// (7) 	Transmitting state machine - transmits selected register's contents
always @(posedge clk or negedge reset) begin
	if (!reset) begin
		r_tx_loc 			<= 0;
		tx_data_counter 	<= 0;
		tx_data_state 		<= TX_DATA_IDLE;
		read_now_tx			<= 0;
		tx_data_done		<= 0;
		write_now_tx		<= 0;
		r_tx_data			<= 0;
		r_wData_TX			<= 0;
		tx_rw 				<= 1;
		tx_cs 				<= 1;
	end
	else begin
		case (tx_data_state)
			TX_DATA_IDLE: 	begin
									tx_data_done <= 0;
									tx_rw <= 1;
									tx_cs <= 1;
									read_now_tx <= 0;
									if (tx_data_go) begin
										tx_data_counter 	<= r_spi_in[1:0]+1'b1;				// Number of transmission to send
										r_tx_loc				<= r_spi_in[7:2];					// Requested register's address
										ABBr 					<= 1;									// Signal to WU
										tx_data_state 		<= TX_DATA_FETCH;
									end
									else begin
										ABBr <= 0;
										tx_data_state	<= RX_DATA_IDLE;
									end
								end
			TX_DATA_FETCH: begin
									read_now_tx			<= 1;
									tx_rw 				<= 1;										// Read memory
									tx_cs 				<= 0;										// Select memory
									tx_data_state		<= TX_DATA_WAIT;
								end
			TX_DATA_WAIT:	begin
									read_now_tx			<= 0;
									tx_rw 				<= 1;										// Read memory
									tx_cs 				<= 1;										// Deselect memory
									if (r_read_done) begin
										tx_data_state  <= TX_DATA_SEND;
										r_tx_data		<= r_tx_data_full;
										ABBr				<= 0;
									end
									else tx_data_state <= TX_DATA_WAIT;
								end
			TX_DATA_SEND:	begin
									write_now_tx		<= 1;
									tx_rw					<= 0;										// Write to memory
									tx_cs					<= 0;										// Select memory
									r_wData_TX 			<= r_tx_data[DATA_BUS_WIDTH-1:DATA_BUS_WIDTH-SPI_WIDTH];
									if (tx_data_counter > 1) begin							// If still transmissions left
										tx_data_state		<= TX_DATA_PAUSE;
									end
									else begin														// If done sending
										tx_data_state 		<= TX_DATA_DONE;
									end
								end
			TX_DATA_PAUSE: begin
									write_now_tx 		<= 0;
									tx_rw 				<= 1;
									tx_cs 				<= 1;
									if (done_re_d) begin
										// Push data out
										r_tx_data		<= {r_tx_data[DATA_BUS_WIDTH-SPI_WIDTH-1:0],{SPI_WIDTH{1'b0}}};
										tx_data_state	<= TX_DATA_SEND;
										tx_data_counter<= tx_data_counter - 1'b1;
									end
									else begin
										tx_data_state	<= TX_DATA_PAUSE;
									end
								end			
			TX_DATA_DONE:	begin
									tx_rw					<= 1;
									tx_cs					<= 1;
									write_now_tx		<= 0;
									tx_data_done 		<= 1;
									tx_data_state 		<= TX_DATA_IDLE;
								end
			default:			begin
									tx_data_state 		<= TX_DATA_IDLE;  // Shouldn't happen
								end
		endcase
	end	
end

// (8) 	Handles reading from a set register
always @(posedge clk or negedge reset) begin
	if (!reset) begin
		r_rAddr 				<= 0;
		read_state 			<= READ_ADDRESS;
		r_tx_data_full 	<= 0;
		r_read_done			<= 0;
		whichABB 			<= 1;
	end
	else begin
		case (read_state)
			READ_ADDRESS: 	begin
									r_read_done		<= 0;
									case (read_now_tx) 
										1'b0: 	begin				// Do not read
														read_state 	<= READ_ADDRESS;
														r_rAddr 		<= 0;
														whichABB 	<= 1;
													end
										1'b1: 	begin				// Read!
														read_state 	<= READ_DATA;
														whichABB		<= 0;									// Output reading ABB to the bus
														r_rAddr		<= r_tx_loc[BANK_SIZE-1:0];
													end									
									endcase
								end
			READ_DATA	: 	begin
									whichABB 		<= 0;
									r_tx_data_full	<= inDBB;											// Get the value coming back
									read_state		<= READ_ADDRESS;
									r_read_done		<= 1;
								end		
		endcase;	
	end
end

endmodule