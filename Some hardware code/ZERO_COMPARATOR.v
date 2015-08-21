`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: University of Calgary
// Engineer: Mila Gorobets
// 
// Create Date:    10:04:12 08/11/2014 
// Design Name: Internal bus sniffer
// Module Name:    COMPARATOR 
// Project Name: SNIFFER
// Target Devices: Zynq 7020
// Tool versions: 
// Description: Compares a value to 0.
//
// Dependencies: None
//
// Revision: 1.00
// Revision 0.01 - File Created
// 			1.00 - Topchip function confirmed (Jan 22, 2015). Cleaned up.
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module ZERO_COMPARATOR(
			A,
			result,
			clk,
			reset,
			enable
    );
	 
// CONFIGURABLE PARAMETERS	
parameter WIDTH = 32; 

// BLOCK INPUTS
input 							clk, reset, enable;			// Regular inputs
input 		[WIDTH-1:0] 	A;									// Value to compare to 0

// BLOCK OUTPUTS
output 							result;							// Pulses on match

// INTERNAL CONNECTIONS
reg 								result_reg;

// ----------------------------------------------------------------------------------------
// WIRE CONNECTIONS
assign result = result_reg;

// ----------------------------------------------------------------------------------------
// PROCESSES
// (1) 	Main process for zero comparator. Handles reset and comparison on enable
always @ (posedge clk or negedge reset)
begin
	if (!reset)
		begin
			result_reg 		<= 0;									// On reset set result to 0
		end
	else
		begin
			if ((A == 0) && enable) begin						// Match (enable is on)
				result_reg <= 1;
			end
			else begin												// Otherwise (no match, or not enabled)
				result_reg <= 0;
			end
		end	
end

endmodule
