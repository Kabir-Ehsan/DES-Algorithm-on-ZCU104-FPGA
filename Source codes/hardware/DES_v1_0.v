
`timescale 1 ns / 1 ps

	module DES_v1_0 #
	(
		// Users to add parameters here

		// User parameters ends
		// Do not modify the parameters beyond this line


		// Parameters of Axi Slave Bus Interface S00_AXIS
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 64,//32,//

		// Parameters of Axi Master Bus Interface M00_AXIS
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 64,//32,//
		parameter integer C_M00_AXIS_START_COUNT	= 32//64//
	)
	(
		// Users to add ports here
      
		// User ports ends
		// Do not modify the ports beyond this line


		// Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk,
		input wire  s00_axis_aresetn,
		output wire  s00_axis_tready,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata,
//		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1 : 0] s00_axis_tstrb,
		input wire  s00_axis_tlast,
		input wire  s00_axis_tvalid,

		// Ports of Axi Master Bus Interface M00_AXIS
		input wire  m00_axis_aclk,
		input wire  m00_axis_aresetn,
		output wire  m00_axis_tvalid,
		output wire [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata,
//		output wire [(C_M00_AXIS_TDATA_WIDTH/8)-1 : 0] m00_axis_tstrb,
		output wire  m00_axis_tlast,
		input wire  m00_axis_tready
	);
	
			   wire [63:0] out_0;
			   wire [63:0] in_0;
			   wire ov;			   
			   wire start2;
			   wire end2;
			   
	           wire [9:0] addra1;
	           reg [9:0] addra1_1;
	           wire [9:0] addrb2;
	           reg [9:0] addrb2_1;
	           wire [9:0] addra2;
	           reg [9:0] addra2_1;
	           wire [9:0] addrb1;
	           reg [9:0] addrb1_1;
	           
// Instantiation of Axi Bus Interface S00_AXIS
	DES_v1_0_S00_AXIS # ( 
		.C_S_AXIS_TDATA_WIDTH(C_S00_AXIS_TDATA_WIDTH)
	) DES_v1_0_S00_AXIS_inst (
		.S_AXIS_ACLK(s00_axis_aclk),
		.S_AXIS_ARESETN(s00_axis_aresetn),
		.S_AXIS_TREADY(s00_axis_tready),
		.S_AXIS_TDATA(s00_axis_tdata),
//		.S_AXIS_TSTRB(s00_axis_tstrb),
		.S_AXIS_TLAST(s00_axis_tlast),
		.S_AXIS_TVALID(s00_axis_tvalid),
		.axi_out(in_0),
		.start2(start2),
		.addr(addrb2),
		.end2(end2)
	);

           wire valid1;
           wire finish1;          	       
           wire valid2;
           reg valid2_1 = 0;
           wire finish2;
           
           reg started1 = 0, started2 = 0;
           reg ended1 = 0, ended2 = 0;
           
	       reg [63:0] key;// = 64'h133457799BBCDFF1;
	       reg [63:0] win;
	       reg [63:0] wout;
	       reg [63:0] din;
	       reg [63:0] dout;
	       wire [63:0] doutb1;         

	       wire enMaxis;

           wire [63:0] doutb2;
	
	always@(*)
    begin 
        started1 = valid1;
        started2 = start2;
        ended1 = finish2;
        ended2 = end2;
        valid2_1 = valid2;
    end
    
    always @(*)//posedge s00_axis_aclk )
	begin
	   if ( s00_axis_aresetn == 1'b0 )
	   begin
	       din = 0; 
	       win = 0;
	       wout = 0;
	       dout = 0;
	       addra1_1 = 0;
           addrb2_1 = 0;
           addra2_1 = 0;
           addrb1_1 = 0; //= 0,
           key = 64'h133457799BBCDFF1;
	   end
	   else 
	   begin
	   	   din = doutb1;
	       win = in_0;
	       wout = doutb2;
	       dout = out_0;
	       addra1_1 = addra1;
	       addrb2_1 = addrb2;
	       addra2_1 = addra2;
	       addrb1_1 = addrb1; 
	       key = 64'h133457799BBCDFF1;
	   end
	end

	bramControl control1
	(
	   .clock(s00_axis_aclk),
	   .reset(s00_axis_aresetn),
	   .enable(ended2),
	   .address(addrb1),
	   .valid(valid1),
	   .finish(finish1)
	);
    
    memory bram1
	(
        .clk(s00_axis_aclk),
        .reset(s00_axis_aresetn),
        .addr0(addrb1_1),
        .ce0(started1),
        .wout_all(doutb1),
        .addr1(addrb2_1),
        .ce1(started2),
        .we1(started2),
        .win(win)
    );
    
    des myDES
    (
        .clk(m00_axis_aclk), 
        .rst(~m00_axis_aresetn), 
        .message(din), 
        .key(key), 
        .iv(started1), 
        .ciphertext(out_0), 
        .ov(ov)
     );
     
     

	
// Instantiation of Axi Bus Interface M00_AXIS
	DES_v1_0_M00_AXIS # ( 
		.C_M_AXIS_TDATA_WIDTH(C_M00_AXIS_TDATA_WIDTH),
		.C_M_START_COUNT(C_M00_AXIS_START_COUNT)
	) DES_v1_0_M00_AXIS_inst (
		.M_AXIS_ACLK(m00_axis_aclk),
		.M_AXIS_ARESETN(m00_axis_aresetn),
		.M_AXIS_TVALID(m00_axis_tvalid),
		.M_AXIS_TDATA(m00_axis_tdata),
//		.M_AXIS_TSTRB(m00_axis_tstrb),
		.M_AXIS_TLAST(m00_axis_tlast),
		.M_AXIS_TREADY(m00_axis_tready),
		.addr(addra1),
		.readEn(enMaxis),
		.in(wout),
		.enable(ended1)
	);

	// Add user logic here

	
    bramControl control2
	(
	   .clock(m00_axis_aclk),
	   .reset(m00_axis_aresetn),
	   .enable(ov),
	   .address(addra2),
	   .valid(valid2),
	   .finish(finish2)
	);
	
    memory bram2
	(
        .clk(m00_axis_aclk),
        .reset(m00_axis_aresetn),
        .addr0(addra1_1),
        .ce0(enMaxis),
        .wout_all(doutb2),
        .addr1(addra2_1),
        .ce1(valid2_1),
        .we1(valid2_1),
        .win(dout)
    );	
    
    //assign m00_axis_tdata = doutb2;

	// User logic ends

	endmodule
