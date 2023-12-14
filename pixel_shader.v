module mul_1616(input signed [31:0] x, input signed [31:0] y, output reg signed[31:0] out);

reg signed[63:0] working_mul;

always @ (*)
begin
	working_mul = x * y;

	if (working_mul[63] == 1'b1) // is negative
	begin
		out[31] = 1;

		out[30:0] = working_mul[46:16];
	end
	else
		out = working_mul[47:16];
end

endmodule

module abs_1616(input signed [31:0] x, output reg signed[31:0] out);

always @ (*)
	if (x[31] == 1'b1) // is negative
	begin
		out = (~x + 1);
	end
	else // is positive
		out = x;

endmodule

module add_1616(input signed [31:0] l, input signed [31:0] r, output reg signed[31:0] out);

always @ (*)
	out = l + r;

endmodule

module sub_1616(input signed [31:0] l, input signed [31:0] r, output reg signed[31:0] out);

always @ (*)
	out = l + (~r + 1);

endmodule

// module get_rand_norm()

// module intersect_circle(
// 	input [31:0] cord_x,
// 	input [31:0] cord_y,
// 	input [31:0] circle_x,
// 	input [31:0] circle_y,
// 	input [31:0] radius_pixels,
// 	output reg hits
// );

// reg [31:0] intercalc_x;
// reg [31:0] intercalc_y;

// always @ (*)
// begin
// 	intercalc_x = (cord_x - circle_x);
// 	intercalc_y = (cord_y - circle_y);

// 	hits = (intercalc_x * intercalc_x + intercalc_y * intercalc_y) < (radius_pixels * radius_pixels);
// end

// endmodule

// 

module intersect_circle(
	input signed [31:0] x_cord_1616,
	input signed [31:0] y_cord_1616,
	input signed [31:0] x_circ_1616,
	input signed [31:0] y_circ_1616,
	input signed [31:0] circ_size_1616,
	output reg hits);

wire signed [31:0] x_cord_minus_circ;
wire signed [31:0] y_cord_minus_circ;
sub_1616 sub1(x_cord_1616, x_circ_1616, x_cord_minus_circ);
sub_1616 sub2(y_cord_1616, y_circ_1616, y_cord_minus_circ);

wire signed [31:0] x_circ_diff_square;
wire signed [31:0] y_circ_diff_square;
mul_1616 mul1(x_cord_minus_circ, x_cord_minus_circ, x_circ_diff_square);
mul_1616 mul2(y_cord_minus_circ, y_cord_minus_circ, y_circ_diff_square);

wire signed [31:0] size_squared;
mul_1616 mul3(circ_size_1616, circ_size_1616, size_squared);

always @ (*)
	hits = x_circ_diff_square + y_circ_diff_square < size_squared;

endmodule


module pixel_shader(
	input[17:0] SW,
	input clk,
	// 1 drawing | 0 reset
	input start,
	input[255:0] randvals,
    input[31:0] uncorrected_x,
    input[31:0] uncorrected_y,
	output reg [23:0] outRGB,
	output reg done);

parameter Height = 32'd120;
parameter Width  = 32'd160;

reg [7:0] S;
reg [7:0] NS;

reg [31:0] working_x_correction; // intermediate
reg [31:0] working_y_correction; // intermediate

reg [31:0] x; // 0 -> 255
reg [31:0] y; // 0 -> 255

reg [31:0] x1616; // 0.0 -> 255.0
reg [31:0] y1616; // 0.0 -> 255.0

wire signed [31:0] x_unorm_1616; // 0.0 -> 2.0
wire signed [31:0] y_unorm_1616; // 0.0 -> 2.0
mul_1616 x_unormer(x1616, 32'b0000_0000_0000_0000__0000_0010_0000_0000, x_unorm_1616); // 1 / 128
mul_1616 y_unormer(y1616, 32'b0000_0000_0000_0000__0000_0010_0000_0000, y_unorm_1616); // 1 / 128

wire signed [31:0] x_zt0_1616;
wire signed [31:0] y_zt0_1616;
mul_1616 x_unormerz(x1616, 32'b0000_0000_0000_0000__0000_0001_0000_0000, x_zt0_1616); // 1 / 256
mul_1616 y_unormerz(y1616, 32'b0000_0000_0000_0000__0000_0001_0000_0000, y_zt0_1616); // 1 / 256

wire signed [31:0] r_background_intercalc; // 0.0 -> 0.5
mul_1616 x_backinter(x1616, 32'b0000_0000_0000_0000__0000_0000_1000_0000, r_background_intercalc); // 1 / 512

wire signed [31:0] r_background; // 0.5 -> 1.0
add_1616 x_backadder(r_background_intercalc, 32'b0000_0000_0000_0000__1000_0000_0000_0000, r_background); // 0.5

wire signed [31:0] r_background_rgb; // 128.0 -> 255.0
mul_1616 x_rounderrr(r_background, 32'b0000_0001_0000_0000__0000_0000_0000_0000, r_background_rgb); // 256

wire signed [31:0] g_background_intercalc; // 0.0 -> 0.25
mul_1616 x_backinterg(x1616, 32'b0000_0000_0000_0000__0000_0000_0100_0000, g_background_intercalc); // 1 / 1024

wire signed [31:0] g_background; // 0.75 -> 1.0
add_1616 x_backadderg(g_background_intercalc, 32'b0000_0000_0000_0000__1100_0000_0000_0000, g_background); // 0.75

wire signed [31:0] g_background_rgb; // 128.0 -> 255.0
mul_1616 x_rounderggg(g_background, 32'b0000_0001_0000_0000__0000_0000_0000_0000, g_background_rgb); // 256



wire signed [31:0] x_norm_1616; // -1.0 -> 1.0
wire signed [31:0] y_norm_1616; // -1.0 -> 1.0
sub_1616 x_normer(x_unorm_1616, 32'b0000_0000_0000_0001__0000_0000_0000_0000, x_norm_1616);
sub_1616 y_normer(y_unorm_1616, 32'b0000_0000_0000_0001__0000_0000_0000_0000, y_norm_1616);

wire[3:0] hits;

intersect_circle circ1(
	x_norm_1616,
	y_norm_1616,
	32'b1111_1111_1111_1111__0000_0000_0000_0000 + randvals[30:15],
	32'b1111_1111_1111_1111__0000_0000_0000_0000 + randvals[43:28],
	32'b0000_0000_0000_0000__0000_0000_0000_0000 + randvals[181:168],
	hits[0]
);

intersect_circle circ2(
	x_norm_1616,
	y_norm_1616,
	32'b1111_1111_1111_1111__0000_0000_0000_0000 + randvals[72:57],
	32'b0000_0000_0000_0000__0000_0000_0000_0000 + randvals[138:123],
	32'b0000_0000_0000_0000__0000_0000_0000_0000 + randvals[71:58],
	hits[1]
);

intersect_circle circ3(
	x_norm_1616,
	y_norm_1616,
	32'b0000_0000_0000_0000__0000_0000_0000_0000 + randvals[231:216],
	32'b1111_1111_1111_1111__0000_0000_0000_0000 + randvals[83:68],
	32'b0000_0000_0000_0000__0000_0000_0000_0000 + randvals[40:27],
	hits[2]
);

intersect_circle circ4(
	x_norm_1616,
	y_norm_1616,
	32'b0000_0000_0000_0000__0000_0000_0000_0000 + randvals[156:141],
	32'b0000_0000_0000_0000__0000_0000_0000_0000 + randvals[222:207],
	32'b0000_0000_0000_0000__0000_0000_0000_0000 + randvals[93:80],
	hits[3]
);

// wire [31:0] x_norm_abs1616; 
// wire [31:0] y_norm_abs1616;
// mul_1616 x_normabser(x_norm_1616, x_norm_1616, x_norm_abs1616);
// mul_1616 y_normabser(y_norm_1616, y_norm_1616, y_norm_abs1616);

// wire [31:0] intercalc_xabs;
// wire [31:0] intercalc_yabs;
// abs_1616 xabser(intercalc_x, intercalc_xabs);
// abs_1616 yabser(intercalc_y, intercalc_yabs);

// reg [31:0]  pre_sqrt;
// reg         sqrt_start;
// wire        sqrt_done;
// wire [31:0] post_sqrt;

// sqrt rooter(
// 	.clk (clk),
// 	.start (sqrt_start),
// 	.valid (sqrt_done),
// 	.rad (pre_sqrt),
// 	.root (post_sqrt)
// );

reg       is_valid;
reg [7:0] out_r;
reg [7:0] out_g;
reg [7:0] out_b;


parameter START      = 0;
parameter SQRT_START = 1;
parameter SQRT_WAIT  = 2;
parameter DONE  = 3;

always @ (*)
	case (S)
	START: NS = SQRT_START;
	SQRT_START: NS = SQRT_WAIT;
	SQRT_WAIT:  
//		if (sqrt_done)
			NS = DONE;
		// else
		// 	NS = SQRT_WAIT;
	DONE:  NS = DONE;
	endcase

	
always @ (posedge clk or negedge start)
begin
	if (start == 1'b0)
	begin
		done <= 0;
	end
	else
	begin
		case (S)
		START: begin
			working_x_correction = ((uncorrected_x * 32'd255) / 120) + 9;
			x <= working_x_correction[7:0];
			x1616 <= {8'd0, x[7:0], 16'd0};

			working_y_correction <= ((uncorrected_y * 32'd343) / 160);
			y <= working_y_correction[7:0];
			y1616 <= {8'd0, y[7:0], 16'd0};
		end
		SQRT_WAIT:
		begin
			if (hits)
			begin
				out_r <= ({SW[1], SW[0]} == 2'b11) ? randvals[22:15] : 8'd255;
				out_g <= ({SW[1], SW[0]} == 2'b11) ? randvals[95:88] : 8'd0;
				out_b <= ({SW[1], SW[0]} == 2'b11) ? randvals[87:80] : 8'd0;
			end
			else
			begin
				case ({SW[1], SW[0]})
				2'b00: begin // good sky
					out_r <= r_background_rgb >> 16;
					out_g <= g_background_rgb >> 16;
					out_b <= 8'd255;
				end
				2'b01: begin // bad sky
					out_r <= r_background;
					out_g <= g_background;
					out_b <= 8'd255;
				end
				2'b10: begin // full screen color pallet
					out_r <= 8'd0;
					out_g <= (x_zt0_1616 * 255) >> 16;
					out_b <= (y_zt0_1616 * 255) >> 16;
				end
				2'b11: begin // random
					out_r <= randvals[241:234];
					out_g <= randvals[174:167];
					out_b <= randvals[152:145];
				end
				endcase
				
			end
		end
		DONE: begin
			outRGB = {
				out_r,
				out_g,
				out_b
			};
			done <= 1;
		end
		endcase
	end
end

always @ (posedge clk or negedge start)
	if (start == 1'b0)
		S <= START;
	else
		S <= NS;


endmodule


// get sqrt function working
// get division working   -----> X
// get rays
// normalize rays
// intersect???
		// 	is_valid <= 1;
		// 	pre_sqrt <= (x_offset_square + y_offset_square);
		// 	// out_r <= ((x_norm_abs_1616 * x_norm_abs_1616 + y_norm_abs_1616 * y_norm_abs_1616) 
		// 	// 		< 64'b0000_0000_0000_0000_0000_0000_0000_0000__0100_0000_0000_0000_0000_0000_0000_0000 ? 8'd255 : 8'b0);
		// 	out_g <= (x_norm_abs2_1616 * 255) >> 16;
		// 	out_b <= (y_norm_abs2_1616 * 255) >> 16;

		// 	sqrter_start <= 0;
		// end

		// WORK2: begin
		// 	out_r <= (post_sqrt < 32'b0000_0000_0000_0000__0100_0000_0000_0000) ? 8'd255 : 8'b0;
		// 	out_g <= (post_sqrt * 128) >> 16;
		// end