module random_10(a, b, c, d, y0, y1, y2);
  input a, b, c, d;
  output y0, y1, y2;
  wire n0, n1, n2, n3, n4, n5, n6;
  assign n0 = a ^ b;
  assign n1 = c & d;
  assign n2 = ~n0;
  assign n3 = n1 | n2;
  assign n4 = a & n3;
  assign n5 = n4 ^ c;
  assign n6 = n5 | n0;
  assign y0 = ~n6;
  assign y1 = n3 & n6;
  assign y2 = n0 ^ n4;
endmodule
