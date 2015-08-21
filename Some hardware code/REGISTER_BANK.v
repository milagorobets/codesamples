`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: SMILE Lab, University of Calgary
// Engineer: Mila Gorobets (milagorobets.com)
// 
// Create Date:    14:53:39 08/07/2014 
// Design Name: 	Internal Bus Sniffer
// Module Name:    REGISTER_BANK 
// Project Name: 	SNIFFER
// Target Devices: Zynq7020
// Tool versions: No idea
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 1.0 - Does the control actions that spi_instruction block used to do
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module REGISTER_BANK(
		clk,					// Common clock (make it something fast)
		reset,				// Reset - active low
		ABB,					// Address Bus
		inDBB,				// Input Data Bus 
		outDBB,				// Output Data Bus (used during reads)
		wDataWU,			
		cs,					// Activates the register bank, active low
		rw,					// READ, not WRITE
		wu_control,			// Control signals for the watch units
		NMI_signals,		// NMI signals from the watch units
		rxSPI,				// Connects directly to the SPI block's RX buffer
		txSPI					// Connects directly to the SPI block's TX buffer
    );
	 
// Signalling occurs during register writes	 
`ifndef CNTRL_LENGTH
`define CNTRL_SNIFF_ON 	0		// Turns sniffer on
`define CNTRL_WRITE_ON	1		// Watch ABB during writes
`define CNTRL_READ_ON	2		// Watch ABB during reads
`define CNTRL_RANGE_ON	3		// Enable range watch
`define CNTRL_N_LADDR	4		// Signal that watch address has been updated (or LADDR)
`define CNTRL_N_UADDR	5		// Signal that upper watch address was updated (UADDR)
`define CNTRL_N_PERIOD	6		// New period is submitted - counter automatically sets to this
`define CNTRL_N_WABB		7		// New value is on the ABB (check) - write
`define CNTRL_N_RABB		8		// New value is on the ABB (check) - read
`define CNTRL_N_COUNT 	9		// New count is submitted - replaces what is there right now, period stays the same
`define CNTRL_LENGTH		10		// How many registers are before this statement? 9 right now
`endif

// Definable values (can change from other blocks)
parameter 	NUM_WATCHUNITS 	= 1;
parameter 	NUM_TEMP_REG		= 8;
parameter 	WU_CONTROL_WD		= NUM_WATCHUNITS * `CNTRL_LENGTH; 
parameter 	DATA_WIDTH 			= 32;
parameter 	BANK_SIZE 			= 6;			// Bits required to represent all addresses
parameter	REG_COUNTER			= 3;
parameter	ADDR_BUS_WIDTH		= 32;
parameter	MAX_ADDR				= 32;

// Block inputs
input 													clk, reset;	// Usual stuff
input														cs;
input														rw;
input			[DATA_WIDTH*NUM_WATCHUNITS-1:0]	wDataWU;					// Watch Units' DATA bus for writing
input			[DATA_WIDTH-1:0] 						rxSPI;					// Connection to the SPI RX register
input			[ADDR_BUS_WIDTH-1:0]					ABB;
input			[DATA_WIDTH-1:0]						inDBB;
input 		[DATA_WIDTH-1:0]						NMI_signals; 			// Connection to NMI signals (shows which tripped)	
// Block outputs
output		[WU_CONTROL_WD-1:0] 					wu_control;				// Watch Units' CONTROL signals	
output 		[DATA_WIDTH-1:0] 						txSPI;					// Connection to the SPI TX register
output		[DATA_WIDTH-1:0]						outDBB;

// REGISTER BANK
reg			[DATA_WIDTH-1:0] register [0:31];
integer i,j;
reg [DATA_WIDTH-1:0] NMI_clear;
// Internal registers
reg			[NUM_WATCHUNITS-1:0]					signal_snon;
reg			[NUM_WATCHUNITS-1:0]					signal_n_count;
reg			[NUM_WATCHUNITS-1:0]					signal_n_laddr;
reg			[NUM_WATCHUNITS-1:0]					signal_n_period;
reg			[NUM_WATCHUNITS-1:0]					signal_n_uaddr;
reg			[NUM_WATCHUNITS-1:0]					signal_range;
reg			[NUM_WATCHUNITS-1:0]					signal_ron;
reg			[NUM_WATCHUNITS-1:0]					signal_won;
reg			[NUM_WATCHUNITS-1:0]					diff_counter;

// Signalling state machine
reg			[2:0]										signal_sm;
parameter	[2:0]										SIGNAL_IDLE = 0,
															SIGNAL_C_NCOUNT = 1,
															SIGNAL_C_NLADDR = 2,
															SIGNAL_C_NPERIOD = 3,
															SIGNAL_C_NUADDR = 4;

assign txSPI	  	= register[5];
assign outDBB		= register[ABB];
assign wu_control = {signal_n_count, 1'b0,1'b0,signal_n_period, signal_n_uaddr, signal_n_laddr,
							signal_range, signal_ron, signal_won, signal_snon};
							
wire [DATA_WIDTH-1:0] int_wDataWU [NUM_WATCHUNITS-1:0];

generate
	for (genvar k = 1; k <= NUM_WATCHUNITS; k = k+1) begin: LINKSTUFF
		assign int_wDataWU[k-1] = wDataWU[DATA_WIDTH*k-1:(k-1)*DATA_WIDTH];
	end
endgenerate

always @(posedge clk or negedge reset)
	begin
		if (!reset) 
			begin
				for (i = 0; i < 2**5; i = i+1)
				begin
					register[i] <= 0;
				end
			end
		else
			begin	
				register[6] <= rxSPI;
				register[2] <= ABB;
				register[3] <= inDBB;
				
				if (NMI_signals) begin
					register[7] <= NMI_signals;
				end
				else if (NMI_clear) begin
					register[7] <= register[7] & ~NMI_clear;
				end

				for (i = 1; i <= NUM_WATCHUNITS; i=i+1) begin
					if (diff_counter[i-1]) begin					
						register[REG_COUNTER+i*NUM_TEMP_REG] <= inDBB;
					end
					else begin
						register[REG_COUNTER+i*NUM_TEMP_REG] <= 
													int_wDataWU[i-1];
					end
				end
					if ((~rw)&(~cs)) begin
						case (ABB[2:0]) 
						// MODIFYING CONTROL REGISTER
						3'b000: 	begin			
										NMI_clear 			<= 0;
										register[ABB]		<= inDBB;
									end
						// WRITING NEW WATCH LADDR
						3'b001: 	begin			
										register[ABB]		<= inDBB;
										NMI_clear 			<= 0;
									end
						// WRITING NEW WATCH UADDR
						3'b010:	begin			
										register[ABB]		<= inDBB;
										NMI_clear 			<= 0;
									end
						// WRITING NEW COUNTER VALUE
						3'b011: 	begin
										NMI_clear <= 0;
										if ((ABB[5:3] == 0) | (ABB[5:3] > NUM_WATCHUNITS)) begin
											register[ABB]	<= inDBB;
										end
										// Need to signal a write to this dual-access register,
										// do it in a different process
									end
						3'b100: 	begin			// WRITING NEW PERIOD VALUE
						NMI_clear <= 0;
										if (inDBB == 0) begin
											register[ABB] <= 15;				// Don't let user set period to 0
										end
										else begin
											register[ABB]	<= inDBB;
										end
									end
						3'b101: 	begin
										NMI_clear <= 0;
										register[ABB]	<= inDBB;
									end
						3'b110: 	begin
						NMI_clear <= 0;
										register[ABB]	<= inDBB;
									end
						3'b111: 	begin
										if (ABB[5:3] == 0) begin
											NMI_clear <= inDBB;
										end
										else begin
											register[ABB]	<= inDBB;
										end
									end
						default: begin
										NMI_clear 			<= 0;
										register[0] 		<= 0;				// Shouldn't happen?
									end
					endcase	
				end
				else begin
					// Nothing happens
				end
			end
	end
	
always @(posedge clk or negedge reset) begin
	if (!reset) begin
		signal_sm <= SIGNAL_IDLE;
		signal_n_count <= 0;
		signal_n_laddr <= 0;
		signal_n_period <= 0;
		signal_n_uaddr <= 0;
		signal_range 	<= 0;
		signal_snon		<= 0;
		signal_won		<= 0;
		signal_ron		<= 0;
	end
	else begin
		case (signal_sm) 
			SIGNAL_IDLE: begin
				if ((!rw)&(!cs)) begin
					case (ABB[2:0]) 
							// MODIFYING CONTROL REGISTER
							3'b000: 	begin		
											// No signalling happens here, signals just hold their values
											signal_sm 		<= SIGNAL_IDLE;
											signal_snon[{ABB[5:3]-1}]	<= inDBB[`CNTRL_SNIFF_ON];
											signal_ron[{ABB[5:3]-1}]	<= inDBB[`CNTRL_READ_ON];
											signal_won[{ABB[5:3]-1}]	<= inDBB[`CNTRL_WRITE_ON];
											signal_range[{ABB[5:3]-1}] <= inDBB[`CNTRL_RANGE_ON];
										end
							// WRITING NEW WATCH LADDR
							3'b001: 	begin			
											signal_n_laddr[{ABB[5:3]-1}] <= 1;
											signal_sm		<= SIGNAL_C_NLADDR;
										end
							// WRITING NEW WATCH UADDR
							3'b010:	begin			
											signal_n_uaddr[{ABB[5:3]-1}]	<= 1;
											signal_sm		<= SIGNAL_C_NUADDR;
										end
							// WRITING NEW COUNTER VALUE
							3'b011: 	begin
											signal_n_count[{ABB[5:3]-1}] <= 1;
											signal_sm		<= SIGNAL_C_NCOUNT;
											diff_counter[{ABB[5:3]-1}] <= 1;
										end
							3'b100: 	begin			// WRITING NEW PERIOD VALUE
											signal_n_period[{ABB[5:3]-1}] <= 1;
											signal_sm		<= SIGNAL_C_NPERIOD;
										end
							3'b101: 	begin
											signal_sm 		<= SIGNAL_IDLE;
										end
							3'b110: 	begin
											signal_sm 		<= SIGNAL_IDLE;
										end
							3'b111: 	begin
											signal_sm 		<= SIGNAL_IDLE;
										end
							default: begin
											signal_sm 		<= SIGNAL_IDLE;
										end
						endcase
				end
			end
			SIGNAL_C_NLADDR: begin
				signal_n_laddr[{ABB[5:3]-1}] <= 0;
				if (cs) signal_sm		<= SIGNAL_IDLE;
				else signal_sm 		<= SIGNAL_C_NLADDR;
			end
			SIGNAL_C_NUADDR: begin
				signal_n_uaddr[{ABB[5:3]-1}] <= 0;
				if (cs) signal_sm		<= SIGNAL_IDLE;
				else signal_sm 		<= SIGNAL_C_NUADDR;
			end
			SIGNAL_C_NCOUNT: begin
				signal_n_count[{ABB[5:3]-1}] <= 0;
				if (cs) signal_sm		<= SIGNAL_IDLE;
				else signal_sm 		<= SIGNAL_C_NCOUNT;
				diff_counter[{ABB[5:3]-1}] <= 0;
			end
			SIGNAL_C_NPERIOD: begin
				signal_n_period[{ABB[5:3]-1}] <= 0;
				if (cs) signal_sm		<= SIGNAL_IDLE;
				else signal_sm 		<= SIGNAL_C_NPERIOD;
			end
		default: begin
			signal_sm		<= SIGNAL_IDLE;
		end	
		endcase
	end
end


endmodule
