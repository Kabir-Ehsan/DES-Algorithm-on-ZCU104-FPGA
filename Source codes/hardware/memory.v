`timescale 1ns / 1ps
  //  Xilinx Single Port Read First RAM
  //  This code implements a parameterizable single-port read-first memory where when data


module memory 
    ( 
        input clk,
        input we1,
        input ce0, ce1,
        input [9:0] addr0, addr1, 
        input reset,
        input [63:0] win,
        output reg [63:0] wout_all
    );
    
  (* ram_style = "block" *) reg [63:0] mem[1023:0];
  

  // The following code either initializes the memory values to a specified file or to all zeros to match hardware
  generate
      integer ram_index;
      initial
      begin
        for (ram_index = 0; ram_index < 256; ram_index = ram_index + 1)
          mem[ram_index] = {64{1'b0}};
      end
  endgenerate
      
    always @(posedge clk)
    begin: ramread
        if(reset==0)
        begin
             wout_all <= {64{1'b0}};
        end
        else
        begin
            if (ce0==1) 
            begin
                 wout_all <= mem[addr0];
            end
        end
    end

always @(posedge clk)  
begin    
    if (ce1) 
    begin
        if(reset == 0)
        begin
            mem[addr1] <= 0;
        end
        else if (we1) 
            mem[addr1] <= win; 
    end
end

endmodule				