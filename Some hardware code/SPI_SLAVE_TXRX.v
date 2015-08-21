`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    11:23:33 08/07/2014 
// Design Name: 
// Module Name:    SPI_SLAVE_TXRX 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module SPI_SLAVE_TXRX(
			reset,
			tx_en,
			tx_data,
			mlb,
			ss,
			sck,
			sdin,
			sdout,
			done,
			rx_data,
			clk
    );
	 
// CONFIGURABLE PARAMETERS
parameter SPI_DATA_WIDTH = 16;

// BLOCK INPUTS
input 											reset, ss, sck; 
input													sdin, tx_en, mlb,clk;
input [SPI_DATA_WIDTH-1:0] 				tx_data;

// BLOCK OUTPUTS
output 											sdout;
output reg 										done;
output reg [SPI_DATA_WIDTH-1:0] 			rx_data;

// INTERNAL CONNECTIONS
reg 	[SPI_DATA_WIDTH-1:0] 				tx_reg, rx_reg;
reg 	[3:0] 									nb;								// Max 16 bits (0 - 15)
wire 												sout;
reg 	[1:0] 									sckr;
wire 												sck_re, sck_fe;

// ----------------------------------------------------------------------------------------
// WIRE CONNECTIONS
assign sout = mlb?tx_reg[SPI_DATA_WIDTH-1]:tx_reg[0]; 
assign sdout = ( (!ss)&&tx_en ) ? sout : 1'b0; 							// Output the sout value

// ----------------------------------------------------------------------------------------
// EDGE MONITORING
// (1)	Monitors rising and falling edges of the sck (clock)
always @ (posedge clk)
	sckr <= {sckr[0], sck};
assign sck_re = (sckr==2'b01);
assign sck_fe = (sckr==2'b10);

// ----------------------------------------------------------------------------------------
// PROCESSES
// (1) 	Main process for downcounter, handles counting and resets
always @ (posedge clk or negedge reset)
begin
	if (!reset) begin
		rx_reg = 0; 												// Flush RX buffer
		rx_data = 0; 												// Set RX data to 0x00
		done = 0; 													// Nothing is done (no tx, nothing)
		nb = 0;														// Bits transmitted/received = 0
		tx_reg = (2 ** SPI_DATA_WIDTH) - 1'b1;
	end
	else if (sck_re) begin			// Rising edge
		if (!ss)
			begin
				if (mlb == 0) 	// LSB first, shift right
					begin 
						rx_reg = {sdin, rx_reg[SPI_DATA_WIDTH-1:1]}; // Shift data out from the end
																					// Pad register
					end
				else
					begin			// MSB first, shift left
						rx_reg = {rx_reg[SPI_DATA_WIDTH-2:0], sdin}; // Shift data out from head
																					// Pad register
					end
				
				if (nb != (SPI_DATA_WIDTH-1))	begin					// If still bits left to receive
					done = 0;
					nb = nb + 1'b1; 
end					// Update number of bits received
				else begin 
					rx_data = rx_reg;
					done = 1;
					nb = 0;
				end					
			end
	end
	else if (sck_fe) begin			// Falling edge
		if (!ss)
					begin
						if (nb == 0) 
							tx_reg = tx_data;
						else
							begin
								if (mlb == 0)		// LSB first
									begin 
										tx_reg = {1'b1, tx_reg[SPI_DATA_WIDTH-1:1]};
									end
								else
									begin
										tx_reg = {tx_reg[SPI_DATA_WIDTH-2:0], 1'b1};
									end
							end
					end	
	end
	else if (ss & (nb != 0)) begin									// In case transmission stopped for w/e reason
		nb = 0;
		done = 1;
	end
end	
endmodule
