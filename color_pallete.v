module mul_1616(input[31:0] l, input[31:0] r, output reg[31:0] out);

reg[63:0] intermediate;

always @ (*)
begin
    intermediate = l * r;

    out = intermediate[31:16];
end

endmodule


// 00000100.00000000 4
// 00000000.01000000 0.25

module pallete(
    input[17:0] SW,
    input[31:0] uncorrected_x,
    input[31:0] uncorrected_y,
    output reg [23:0] color
);

parameter Width  = 32'd160;
parameter Height = 32'd120;

reg [31:0] working_x_correction; // intermediate
reg [31:0] working_y_correction; // intermediate

reg [31:0] x; // 0 -> 255
reg [31:0] y; // 0 -> 255

reg [31:0] x_fixed1616; // 0.0 -> 255.0
reg [31:0] y_fixed1616; // 0.0 -> 255.0

wire [31:0] x_unorm_fixed1616; // 0.0 -> 2.0
wire [31:0] y_unorm_fixed1616; // 0.0 -> 2.0

reg [7:0] out_r;
reg [7:0] out_g;
reg [7:0] out_b;

// correct variables to range 0->255
reg is_valid;
reg[15:0] distance_from_center_manhat;

mul_1616 unormerx(x_fixed1616, 32'b0000_0000_0000_0000__0000_0010_0000_0000, x_unorm_fixed1616);
mul_1616 unormery(y_fixed1616, 32'b0000_0000_0000_0000__0000_0010_0000_0000, y_unorm_fixed1616);

always @ (*)
begin
    is_valid = 1;

    // getting x and y to be normal values
    working_x_correction = ((uncorrected_x * 32'd255) / Width) + 8'd77;
    x = working_x_correction[7:0];
    x_fixed1616 = {x[15:0], 16'd0};
    

    working_y_correction = ((uncorrected_y * 32'd255) / Height) + (~8'd16 + 1);
    y = working_y_correction[7:0];
    y_fixed1616 = {y[15:0], 16'd0};

    // if (112 < x && x < 144 && 112 < y && y < 144)
    //     out_r = 8'd32;
    // else
    //     case ({x > 128, y > 128})
    //     2'b00: out_r = 8'd0;
    //     2'b01: out_r = 8'd128;
    //     2'b10: out_r = 8'd128;
    //     2'b11: out_r = 8'd255;
    //     endcase

    out_r = 32'b0;
    out_g = x_unorm_fixed1616[31:16] * 32;
    out_b = y_unorm_fixed1616[31:16] * 32;

    color = {
        is_valid ? out_r : 8'b0, // R
        is_valid ? out_g : 8'b0, // G
        is_valid ? out_b : 8'b0, // B
    };


end
endmodule

// module qmult #(
// 	//Parameterized values
// 	parameter Q = 12,
// 	parameter N = 16
// 	)
// 	(
// 	 input			[N-1:0]	a,
// 	 input			[N-1:0]	b,
// 	 output         [N-1:0] q_result,    //output quantized to same number of bits as the input
//      output			overflow             //signal to indicate output greater than the range of our format
// 	 );
	 
// 	 //	The underlying assumption, here, is that both fixed-point values are of the same length (N,Q)
// 	 //	Because of this, the results will be of length N+N = 2N bits
// 	 //	This also simplifies the hand-back of results, as the binimal point 
// 	 //	will always be in the same location
	
// 	wire [2*N-1:0]	f_result;		//	Multiplication by 2 values of N bits requires a 
// 									//	register that is N+N = 2N deep
// 	wire [N-1:0]   multiplicand;
// 	wire [N-1:0]	multiplier;
// 	wire [N-1:0]    a_2cmp, b_2cmp;
// 	wire [N-2:0]    quantized_result,quantized_result_2cmp;
	
// 	assign a_2cmp = {a[N-1],{(N-1){1'b1}} - a[N-2:0]+ 1'b1};  //2's complement of a
// 	assign b_2cmp = {b[N-1],{(N-1){1'b1}} - b[N-2:0]+ 1'b1};  //2's complement of b
	
//     assign multiplicand = (a[N-1]) ? a_2cmp : a;              
//     assign multiplier   = (b[N-1]) ? b_2cmp : b;
    
//     assign q_result[N-1] = a[N-1]^b[N-1];                      //Sign bit of output would be XOR or input sign bits
//     assign f_result = multiplicand[N-2:0] * multiplier[N-2:0]; //We remove the sign bit for multiplication
//     assign quantized_result = f_result[N-2+Q:Q];               //Quantization of output to required number of bits
//     assign quantized_result_2cmp = {(N-1){1'b1}} - quantized_result[N-2:0] + 1'b1;  //2's complement of quantized_result
//     assign q_result[N-2:0] = (q_result[N-1]) ? quantized_result_2cmp : quantized_result; //If the result is negative, we return a 2's complement representation 
//     																					 //of the output value
//     assign overflow = (f_result[2*N-2:N-1+Q] > 0) ? 1'b1 : 1'b0;

// endmodule

// module divu_int #(parameter WIDTH=5) ( // width of numbers in bits
//     input wire logic clk,              // clock
//     input wire logic rst,              // reset
//     input wire logic start,            // start calculation
//     output     logic busy,             // calculation in progress
//     output     logic done,             // calculation is complete (high for one tick)
//     output     logic valid,            // result is valid
//     output     logic dbz,              // divide by zero
//     input wire logic [WIDTH-1:0] a,    // dividend (numerator)
//     input wire logic [WIDTH-1:0] b,    // divisor (denominator)
//     output     logic [WIDTH-1:0] val,  // result value: quotient
//     output     logic [WIDTH-1:0] rem   // result: remainder
//     );

//     logic [WIDTH-1:0] b1;             // copy of divisor
//     logic [WIDTH-1:0] quo, quo_next;  // intermediate quotient
//     logic [WIDTH:0] acc, acc_next;    // accumulator (1 bit wider)
//     logic [$clog2(WIDTH)-1:0] i;      // iteration counter

//     // division algorithm iteration
//     always_comb begin
//         if (acc >= {1'b0, b1}) begin
//             acc_next = acc - b1;
//             {acc_next, quo_next} = {acc_next[WIDTH-1:0], quo, 1'b1};
//         end else begin
//             {acc_next, quo_next} = {acc, quo} << 1;
//         end
//     end

//     // calculation control
//     always_ff @(posedge clk) begin
//         done <= 0;
//         if (start) begin
//             valid <= 0;
//             i <= 0;
//             if (b == 0) begin  // catch divide by zero
//                 busy <= 0;
//                 done <= 1;
//                 dbz <= 1;
//             end else begin
//                 busy <= 1;
//                 dbz <= 0;
//                 b1 <= b;
//                 {acc, quo} <= {{WIDTH{1'b0}}, a, 1'b0};  // initialize calculation
//             end
//         end else if (busy) begin
//             if (i == WIDTH-1) begin  // we're done
//                 busy <= 0;
//                 done <= 1;
//                 valid <= 1;
//                 val <= quo_next;
//                 rem <= acc_next[WIDTH:1];  // undo final shift
//             end else begin  // next iteration
//                 i <= i + 1;
//                 acc <= acc_next;
//                 quo <= quo_next;
//             end
//         end
//         if (rst) begin
//             busy <= 0;
//             done <= 0;
//             valid <= 0;
//             dbz <= 0;
//             val <= 0;
//             rem <= 0;
//         end
//     end
// endmodule