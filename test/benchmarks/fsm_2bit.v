module fsm_2bit(clk, go, state, done);
  input clk, go;
  output reg [1:0] state;
  output reg done;
  always @(posedge clk) begin
    case (state)
      0: begin if (go) begin state <= 1; end end
      1, 2: state <= 3;
      default: state <= 0;
    endcase
  end
  always @(*) begin
    case (state)
      3: done <= 1;
      default: done <= 0;
    endcase
  end
endmodule
