`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/10/2022 02:15:29 PM
// Design Name: 
// Module Name: bramControl

// 
//////////////////////////////////////////////////////////////////////////////////


module bramControl(

        input clock,
        input reset,
        input enable,
        output [9:0] address,
        output valid,
        output finish

    );
    
    reg ce;
    reg [9:0] add;
    reg finished;
    
    always@(posedge clock)
    begin
        if(reset == 0)
        begin
            ce <= 0;
        end
        else
        begin
            if(enable==1 && add<1023)
            begin
                ce <= 1;
            end
            else
            begin
                ce <= 0;
            end
        end
    end
    
    always@(posedge clock)
    begin
        if(reset == 0)
        begin
            add <= 0;
            finished <= 0;
        end
        else
        begin
            if(ce == 1)
            begin
                if(add<1023)
                begin
                    add <= add+1;
                    finished <= finished;
                end
                else
                begin
                    add <= add;
                    finished <= 1;
                end
            end
            else
            begin
                finished <= finished;
                add <= add;
            end
        end
    end
    assign address = add;
    assign valid = ce;
    assign finish = finished;
    
endmodule
