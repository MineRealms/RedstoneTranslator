module fsm_1bit(clk, go, state, done);
  input clk, go;
  output reg state;
  output reg done;
  always @(posedge clk) begin
    case (state)
      0: begin if (go) begin state <= 1; end end
      default: state <= 0;
    endcase
  end
  always @(*) begin
    case (state)
      1: done <= 1;
      default: done <= 0;
    endcase
  end
endmodule
