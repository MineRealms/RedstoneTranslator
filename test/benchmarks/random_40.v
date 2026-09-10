module random_40(a, b, y);
  input [7:0] a, b;
  output [7:0] y;
  wire [7:0] t1, t2, t3, t4;
  assign t1 = a ^ b;
  assign t2 = a | b;
  assign t3 = t1 & t2;
  assign t4 = a & b;
  assign y = t3 ^ t4;
endmodule
