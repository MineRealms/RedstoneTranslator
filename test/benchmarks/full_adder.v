module full_adder(a, b, cin, sum, cout);
  input a, b, cin;
  output sum, cout;
  wire ab;
  assign ab = a ^ b;
  assign sum = ab ^ cin;
  assign cout = (a & b) | (sum & cin);
endmodule
