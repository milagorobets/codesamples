`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: University of Calgary
// Engineer: Mila Gorobets
// 
// Create Date:    10:22:57 08/11/2014 
// Design Name: Internal bus sniffer
// Module Name:    DOWNCOUNTER 
// Project Name: SNIFFER
// Target Devices: Zynq 7020
// Tool versions: 
// Description: Counts down from max to 0. Can set counter to whatever value needed 
// 				 at any time
// Dependencies: Needs zero comparator
//
// Revision: 1.00
// Revision 0.01 - File Created
// 			1.00 - Topchip function confirmed (Jan 22, 2015). Cleaned up.
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module DOWNCOUNTER(
		clk, 
		reset,
		restart,
		load,
		load_count,
		currcount,
		enable,
		maxcount,
		current_value,
		zero
    );
	
// CONFIGURABLE PARAMETERS	
parameter WIDTH = 32;

// BLOCK INPUTS
input 							clk, reset, enable;			// Regular inputs
input								restart;
input								load;								// Loads a new period
input								load_count;						// Loads a different count from currcount bus
input 		[WIDTH-1:0] 	currcount;						// Used to directly change the count value
input 		[WIDTH-1:0] 	maxcount;						// Used to change the period (max count)

// BLOCK OUTPUTS
output 		[WIDTH-1:0] 	current_value;					// Current value of the downcounter, written to a register location
output 							zero;								// Pulses when count is at 0

// INTERNAL CONNECTIONS
reg 			[1:0] 			zero_reg;						// Zero rising edge detection
wire 								zero_w;
reg 			[WIDTH-1:0] 	counter_result;
reg 			[WIDTH-1:0] 	r_counter_period;
wire 			[WIDTH-1:0] 	i_counter_result;
reg 			[1:0]				dc_en_r;							// Downcounter rising edge detection
wire 								dc_en_re;

// ----------------------------------------------------------------------------------------
// COMPONENTS
// (1) 	Compares a given value to 0
// 		For monitoring when downcounter expires
ZERO_COMPARATOR #(WIDTH) zerocomparator(
			.A(i_counter_result),
			.result(zero_w),
			.clk(clk),
			.reset(reset),
			.enable(1'b1)
    );

// ----------------------------------------------------------------------------------------
// EDGE MONITORING
// (1)	Outputs only the rising edge of the zero pulse
//			Lasts one clock cycle
always @(posedge clk) begin
	zero_reg <=  {zero_reg[0], zero_w};
end
assign 		zero = (zero_reg == 2'b01);
// (2)	Detects rising edge of the enable signal
//			Otherwise counter would decrement twice for a single match
always @(posedge clk) begin
	dc_en_r <= {dc_en_r[0], enable};
end
assign dc_en_re = (dc_en_r == 2'b01);

// ----------------------------------------------------------------------------------------
// WIRE CONNECTIONS
assign 		current_value = counter_result;
assign 		i_counter_result = counter_result;

// ----------------------------------------------------------------------------------------
// PROCESSES
// (1) 	Main process for downcounter, handles counting and resets
always @ (posedge clk or negedge reset)
begin	
	if (!reset)
		begin
			counter_result 			<= 15;					// On reset, set everything to 15
			r_counter_period			<= 15;
		end
	else
		if (restart) begin
			counter_result 			<= r_counter_period;	// On restart (counter hit 0), reset to max value
		end
		else if (load_count) begin
			counter_result <= currcount;						// Load in a new count, overwrite current count
		end	
		else if (load) begin										// Load in a new period (used during restart), overwrites current count
			if (maxcount != 0) begin				
				r_counter_period			<= maxcount;		
				counter_result				<= maxcount;
			end
			else begin
				r_counter_period 			<= 15;				// In case user tried setting period to 0, set it to 15
				counter_result				<= 15;				
			end
		end
		else	begin
			if (dc_en_re == 1)									// If downcounter enabled, count down by 1
				begin					
					counter_result 		<= (counter_result - 1);
				end
		end
end

endmodule
