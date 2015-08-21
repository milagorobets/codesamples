`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: University of Calgary
// Engineer: Mila Gorobets
// 
// Create Date:    10:04:12 08/11/2014 
// Design Name: Internal Bus Sniffer
// Module Name:    COMPARATOR 
// Project Name: SNIFFER
// Target Devices: Zynq 7020
// Tool versions: 
// Description: Compares two values. Outputs a pulse on a match.
//
// Dependencies: 
//
// Revision: 1.00
// Revision 0.01 - File Created
// 			1.00 - Topchip function confirmed (Jan 22, 2015). Cleaned up.
// Additional Comments:  
//
//////////////////////////////////////////////////////////////////////////////////
module COMPARATOR(
			A,
			B,
			result,
			clk,
			reset,
			enable,
			take
    );

// CONFIGURABLE PARAMETERS
parameter WIDTH = 32; 

// BLOCK INPUTS
input 							clk, reset, enable;
input 		[WIDTH-1:0] 	A, B;
input								take;

// BLOCK OUTPUTS 						
output 							result;

// INTERNAL CONNECTIONS
reg 								result_reg;
reg 			[WIDTH-1:0] 	const_A;							// Latch for value A

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
			result_reg 		<= 0;									// Set to known values on reset
			const_A			<= {WIDTH{1'b1}};					// All 1's, less likely to match accidentally
		end
	else
		begin
			if (take) begin										// Load signal to latch a constant A value
				result_reg 	<= 0;
				const_A		<= A;
			end
			else begin
				if ((const_A == B) && enable) begin			// Compare on enable
					result_reg <= 1;
				end
				else begin											// Not matched or not enabled
					result_reg <= 0;
				end
			end
		end	
end

endmodule
