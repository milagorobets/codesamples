`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: University of Calgary	
// Engineer: Mila Gorobets
// 
// Create Date:    10:04:12 08/11/2014 
// Design Name: Internal Bus Sniffer
// Module Name:    RANGE_COMPARATOR 
// Project Name: SNIFFER
// Target Devices: Zynq 7020
// Tool versions: 
// Description: Checks if a given value falls within a pre-defined range
//
// Dependencies: None
//
// Revision: 1.00
// Revision 0.01 - File Created
// 			1.00 - Topchip function confirmed (Jan 22, 2015). Cleaned up.
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module RANGE_COMPARATOR(
			lA,
			uA,
			B,
			result,
			clk,
			reset,
			enable,
			take_lA,
			take_uA
    );

// CONFIGURABLE PARAMETERS	
parameter WIDTH = 32; 

// BLOCK INPUTS
input 							clk, reset, enable;			// Regular inputs
input 		[WIDTH-1:0] 	lA, uA, B;						// Value inputs
input 							take_lA, take_uA;				// Latching signals

// BLOCK OUTPUTS
output 							result;							// Pulses on a range match

// INTERNAL CONNECTIONS
reg 								result_reg;						
reg 			[WIDTH-1:0] 	const_lA, const_uA;			// Hold latched values

// ----------------------------------------------------------------------------------------
// WIRE CONNECTIONS
assign result = result_reg;

// ----------------------------------------------------------------------------------------
// PROCESSES
// (1) 	Main process for comparator. Handles reset, latching and comparison on enable
always @ (posedge clk or negedge reset)
begin
	if (!reset)
		begin
			result_reg 	<= 0;										// Set to known values on reset
			const_lA		<= {WIDTH{1'b1}};						// Make range of 0 elements (just in case)
			const_uA		<= {WIDTH{1'b1}};
		end
	else
		begin
			if (take_lA) begin									// Latch a new lower bound value
				result_reg 	<= 0;									// Reset result
				const_lA		<= lA;
			end
			else if (take_uA) begin								// Latch a new upper bound value
				result_reg 	<= 0;									// Reset input
				const_uA 	<= uA;
			end
			else if (enable) begin								// Compare on enable
				if ((const_lA <= B) && (const_uA >= B)) begin
					result_reg <= 1;
				end
				else result_reg <= 0;							// No match
			end
			else begin
				result_reg <= 0;									// Not enabled
			end
		end	
end


endmodule
