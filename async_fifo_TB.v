`timescale 1ns / 1ps

module async_fifo_TB;

  parameter DATA_WIDTH = 8;
  parameter SCOREBOARD_DEPTH = 128;

  // Output Wires
  wire [DATA_WIDTH-1:0] data_out;
  wire full;
  wire empty;
  
  // Input Regs
  reg [DATA_WIDTH-1:0] data_in;
  reg w_en, wclk, wrst_n;
  reg r_en, rclk, rrst_n;

  // Pure Verilog Scoreboard Array (Replaces SystemVerilog Queue)
  reg [DATA_WIDTH-1:0] scoreboard_mem [0:SCOREBOARD_DEPTH-1];
  reg [7:0] sb_wr_ptr;
  reg [7:0] sb_rd_ptr;
  reg [DATA_WIDTH-1:0] wdata;
  
  // Loop Iteration Variables (Declared globally for Verilog)
  integer i, j;

  // Instantiate Design Under Test (DUT)
  asynchronous_fifo as_fifo (
    .wclk(wclk), .wrst_n(wrst_n),
    .rclk(rclk), .rrst_n(rrst_n),
    .w_en(w_en), .r_en(r_en),
    .data_in(data_in), .data_out(data_out),
    .full(full), .empty(empty)
  );

  // Clock Generation
  always #5   wclk = ~wclk; // 10ns cycle period
  always #17  rclk = ~rclk; // 34ns cycle period
  
  //------------------------------------------------
  // WRITE STIMULUS DOMAIN
  //------------------------------------------------
  initial begin
    wclk = 1'b0; wrst_n = 1'b0;
    w_en = 1'b0;
    data_in = 0;
    sb_wr_ptr = 0;
    
    repeat(10) @(posedge wclk);
    wrst_n = 1'b1;

    for (j=0; j<2; j=j+1) begin
      for (i=0; i<30; i=i+1) begin
        @(posedge wclk);
        if (!full) begin
          w_en <= (i%2 == 0) ? 1'b1 : 1'b0;
          if (i%2 == 0) begin
            // Verilog random data generation
            data_in <= $random; 
            scoreboard_mem[sb_wr_ptr] = $random; // Store directly for matching
            sb_wr_ptr = sb_wr_ptr + 1;
          end
        end else begin
          w_en <= 1'b0;
        end
      end
      #50;
    end
  end

  //------------------------------------------------
  // READ STIMULUS & VERIFICATION DOMAIN
  //------------------------------------------------
  initial begin
    rclk = 1'b0; rrst_n = 1'b0;
    r_en = 1'b0;
    sb_rd_ptr = 0;

    repeat(20) @(posedge rclk);
    rrst_n = 1'b1;

    for (j=0; j<2; j=j+1) begin
      for (i=0; i<30; i=i+1) begin
        @(posedge rclk);
        if (!empty) begin
          r_en <= (i%2 == 0) ? 1'b1 : 1'b0;
          
          if (i%2 == 0) begin
            // Fetch validation target data from memory array
            wdata = scoreboard_mem[sb_rd_ptr];
            sb_rd_ptr = sb_rd_ptr + 1;
            
            // Wait 1 read clock cycle for RAM array lookup latency
            @(posedge rclk); 
            #1; // Delay past hold time window
            
            if (data_out !== wdata) begin
              $display("ERROR at %0t ns: Expected = %h, Got = %h", $time, wdata, data_out);
            end else begin
              $display("PASS at %0t ns: Read data successfully matches written data (%h)", $time, data_out);
            end
          end
        end else begin
          r_en <= 1'b0;
        end
      end
      #50;
    end

    $display("[TEST COMPLETE] Validation sequence ended.");
    $finish;
  end
  
  // Waveform Dump Configuration
  initial begin 
    $dumpfile("dump.vcd"); 
    $dumpvars(0, async_fifo_TB);
  end

endmodule
