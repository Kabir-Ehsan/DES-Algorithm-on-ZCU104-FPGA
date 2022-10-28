//  Latency = 16 Clock Cycle


`timescale 1ns / 1ps

module des(input clk, input rst, input [64:1] message, input [64:1] key, input iv, output reg [64:1] ciphertext, output ov);

              reg [16:0] ii, oo;              
              wire [64:1] new_msg;
              wire [64:1] cipher;
			  reg [32:1] L[16:0], R[16:0];
			  reg [32:1] tempL, tempR;
			  wire [32:1] fout[16:1];
			  wire [48:1] k1, k2, k3, k4, k5, k6, k7, k8, k9, k10, k11, k12, k13, k14, k15, k16;

			  
			  always@(posedge clk)
			  begin 
			      if(rst)
			      begin
			         ii <= 16'b0;
			         oo <= 0;
			      end
			      else
			      begin
			       ii[0] <= iv;
			       ii[1] <= ii[0];
			       ii[2] <= ii[1];
			       ii[3] <= ii[2];
			       ii[4] <= ii[3];
			       ii[5] <= ii[4];
			       ii[6] <= ii[5];
			       ii[7] <= ii[6];
			       ii[8] <= ii[7];
			       ii[9] <= ii[8];
			       ii[10] <= ii[9];
			       ii[11] <= ii[10];
			       ii[12] <= ii[11];
			       ii[13] <= ii[12];
			       ii[14] <= ii[13];
			       ii[15] <= ii[14];
			       ii[16] <= ii[15];
			       oo <= ii[16];
			       //ov <= 
			      end
			  end
			  

			  assign ov = oo;
			  
			  perm_IP2 ip(.message(message), .temp_msg(new_msg));

              processKey2 pk(key, k1, k2, k3, k4, k5, k6, k7, k8, k9, k10, k11, k12, k13, k14, k15, k16);

              //always@(*)
              //always@(new_msg or fout[1] or fout[2] or fout[3] or fout[4] or fout[5] or fout[6] or fout[7] or fout[8] or fout[9] or fout[10] or fout[11] or fout[12] or fout[13] or fout[14] or fout[15] or fout[16])
              //begin
          always@(posedge clk)
          begin
			   {L[0],R[0]} <= new_msg;			    			     
			   L[1] <= R[0];			   
               L[2] <= R[1];              
               L[3] <= R[2];               
               L[4] <= R[3];               
               L[5] <= R[4];              
               L[6] <= R[5];              
               L[7] <= R[6];              
               L[8] <= R[7];               
               L[9] <= R[8];              
               L[10] <= R[9];              
               L[11] <= R[10];              
               L[12] <= R[11];              
               L[13] <= R[12];              
               L[14] <= R[13];              
               L[15] <= R[14];             
               L[16] <= R[15];
         end      
         always@(posedge clk)
         begin
               R[1] <= L[0]^fout[1];  
               R[2] <= L[1]^fout[2];                                                                        
               R[3] <= L[2]^fout[3];                              
               R[4] <= L[3]^fout[4];                              
               R[5] <= L[4]^fout[5];                 
               R[6] <= L[5]^fout[6];                               
               R[7] <= L[6]^fout[7];                                
               R[8] <= L[7]^fout[8];                             
               R[9] <= L[8]^fout[9];                            
               R[10]<= L[9]^fout[10];                  
               R[11]<= L[10]^fout[11];               
               R[12]<= L[11]^fout[12];                    
               R[13]<= L[12]^fout[13];                               
               R[14]<= L[13]^fout[14];                                
               R[15]<= L[14]^fout[15];                               
               R[16]<= L[15]^fout[16];                
         end
 
 
			  f_fun2 f1(.R(R[0]), .K(k1), .Rs(fout[1]));			  
			  
			  f_fun2 f2(.R(R[1]), .K(k2), .Rs(fout[2]));			  
			  			  
			  f_fun2 f3(.R(R[2]), .K(k3), .Rs(fout[3]));			  

			  f_fun2 f4(.R(R[3]), .K(k4), .Rs(fout[4]));			  
  
			  f_fun2 f5(.R(R[4]), .K(k5), .Rs(fout[5]));			  

			  f_fun2 f6(.R(R[5]), .K(k6), .Rs(fout[6]));

			  f_fun2 f7(.R(R[6]), .K(k7), .Rs(fout[7]));			  
			  
			  f_fun2 f8(.R(R[7]), .K(k8), .Rs(fout[8]));
			   
			  f_fun2 f9(.R(R[8]), .K(k9), .Rs(fout[9]));
			  
			  f_fun2 f10(.R(R[9]), .K(k10), .Rs(fout[10]));			  
			  
			  f_fun2 f11(.R(R[10]), .K(k11), .Rs(fout[11]));

			  f_fun2 f12(.R(R[11]), .K(k12), .Rs(fout[12]));

			  f_fun2 f13(.R(R[12]), .K(k13), .Rs(fout[13]));

			  f_fun2 f14(.R(R[13]), .K(k14), .Rs(fout[14]));
			  
			  f_fun2 f15(.R(R[14]), .K(k15), .Rs(fout[15]));
			  
			  f_fun2 f16(.R(R[15]), .K(k16), .Rs(fout[16]));
		  
			perm_IP_inv2 invIP(.message({R[16],L[16]}), .temp_msg(cipher));

			always @(posedge clk)
			begin
			     if(rst)
			     begin
			         ciphertext <= 0;
			     end
			     else
			         ciphertext <= cipher;
//			    $monitor("cipher=%b",cipher);
//			    $monitor("ciphertext=%b",ciphertext);
			end
			   		   
			   
endmodule

module perm_IP2(input [64:1] message, output [64:1] temp_msg);


				        assign temp_msg[1] = message[58];
						assign temp_msg[2] = message[50];
						assign temp_msg[3] = message[42];
						assign temp_msg[4] = message[34];
						assign temp_msg[5] = message[26];
						assign temp_msg[6] = message[18];
						assign temp_msg[7] = message[10];
						assign temp_msg[8] = message[2];
						assign temp_msg[9] = message[60];
						assign temp_msg[10] = message[52];
						assign temp_msg[11] = message[44];
						assign temp_msg[12] = message[36];
						assign temp_msg[13] = message[28];
						assign temp_msg[14] = message[20];
						assign temp_msg[15] = message[12];
						assign temp_msg[16] = message[4];
						assign temp_msg[17] = message[62];
						assign temp_msg[18] = message[54];
						assign temp_msg[19] = message[46];
						assign temp_msg[20] = message[38];
						assign temp_msg[21] = message[30];
						assign temp_msg[22] = message[22];
						assign temp_msg[23] = message[14];
						assign temp_msg[24] = message[6];
						assign temp_msg[25] = message[64];
						assign temp_msg[26] = message[56];
						assign temp_msg[27] = message[48];
						assign temp_msg[28] = message[40];
						assign temp_msg[29] = message[32];
						assign temp_msg[30] = message[24];
						assign temp_msg[31] = message[16];
						assign temp_msg[32] = message[8];
						assign temp_msg[33] = message[57];
						assign temp_msg[34] = message[49];
						assign temp_msg[35] = message[41];
						assign temp_msg[36] = message[33];
						assign temp_msg[37] = message[25];
						assign temp_msg[38] = message[17];
						assign temp_msg[39] = message[9];
						assign temp_msg[40] = message[1];
						assign temp_msg[41] = message[59];
						assign temp_msg[42] = message[51];
						assign temp_msg[43] = message[43];
						assign temp_msg[44] = message[35];
						assign temp_msg[45] = message[27];
						assign temp_msg[46] = message[19];
						assign temp_msg[47] = message[11];
						assign temp_msg[48] = message[3];
						assign temp_msg[49] = message[61];
						assign temp_msg[50] = message[53];
						assign temp_msg[51] = message[45];
						assign temp_msg[52] = message[37];
						assign temp_msg[53] = message[29];
						assign temp_msg[54] = message[21];
						assign temp_msg[55] = message[13];
						assign temp_msg[56] = message[5];
						assign temp_msg[57] = message[63];
						assign temp_msg[58] = message[55];
						assign temp_msg[59] = message[47];
						assign temp_msg[60] = message[39];
						assign temp_msg[61] = message[31];
						assign temp_msg[62] = message[23];
						assign temp_msg[63] = message[15];
						assign temp_msg[64] = message[7];
			      //perm_IP_inverse = temp_msg;
    //end
endmodule

module processKey2(input [64:1] key, output [48:1] key1, key2, key3, key4, key5, key6, key7, key8, key9, key10, key11, key12, key13, key14, key15, key16);
              
              wire [56:1] temp_pc1;
			  wire [28:1] C[16:1], D[16:1];
			  wire [48:1] K[16:1];
			  //integer i;

              pc1_perm2 pc1(.key(key), .temp_P(temp_pc1));
			  
			  ci_di3 CD1(.C_last(temp_pc1[56:29]), .D_last(temp_pc1[28:1]), .cidi({C[1],D[1]}));
			  pc2_perm2 pc21(.key({C[1],D[1]}), .temp_P(K[1]));
			  
			  ci_di3 CD2(.C_last(C[1]), .D_last(D[1]), .cidi({C[2],D[2]}));
			  pc2_perm2 pc22(.key({C[2],D[2]}), .temp_P(K[2]));
			  			  
			  ci_di2 CD3(.C_last(C[2]), .D_last(D[2]), .cidi({C[3],D[3]}));
			  pc2_perm2 pc23(.key({C[3],D[3]}), .temp_P(K[3]));
			  
			  ci_di2 CD4(.C_last(C[3]), .D_last(D[3]), .cidi({C[4],D[4]}));
			  pc2_perm2 pc24(.key({C[4],D[4]}), .temp_P(K[4]));
			  
			  ci_di2 CD5(.C_last(C[4]), .D_last(D[4]), .cidi({C[5],D[5]}));
			  pc2_perm2 pc25(.key({C[5],D[5]}), .temp_P(K[5]));
			  
			  ci_di2 CD6(.C_last(C[5]), .D_last(D[5]), .cidi({C[6],D[6]}));
			  pc2_perm2 pc26(.key({C[6],D[6]}), .temp_P(K[6]));
			  
			  ci_di2 CD7(.C_last(C[6]), .D_last(D[6]), .cidi({C[7],D[7]}));
			  pc2_perm2 pc27(.key({C[7],D[7]}), .temp_P(K[7]));
			  
			  ci_di2 CD8(.C_last(C[7]), .D_last(D[7]), .cidi({C[8],D[8]}));
			  pc2_perm2 pc28(.key({C[8],D[8]}), .temp_P(K[8]));
			  
			  ci_di3 CD9(.C_last(C[8]), .D_last(D[8]), .cidi({C[9],D[9]}));
			  pc2_perm2 pc29(.key({C[9],D[9]}), .temp_P(K[9]));
			  
			  ci_di2 CD10(.C_last(C[9]), .D_last(D[9]), .cidi({C[10],D[10]}));
			  pc2_perm2 pc210(.key({C[10],D[10]}), .temp_P(K[10]));
			  
			  ci_di2 CD11(.C_last(C[10]), .D_last(D[10]), .cidi({C[11],D[11]}));
			  pc2_perm2 pc211(.key({C[11],D[11]}), .temp_P(K[11]));
			  
			  ci_di2 CD12(.C_last(C[11]), .D_last(D[11]), .cidi({C[12],D[12]}));
			  pc2_perm2 pc212(.key({C[12],D[12]}), .temp_P(K[12]));
			  
			  ci_di2 CD13(.C_last(C[12]), .D_last(D[12]), .cidi({C[13],D[13]}));
			  pc2_perm2 pc213(.key({C[13],D[13]}), .temp_P(K[13]));
			  
			  ci_di2 CD14(.C_last(C[13]), .D_last(D[13]), .cidi({C[14],D[14]}));
			  pc2_perm2 pc214(.key({C[14],D[14]}), .temp_P(K[14]));
			  
			  ci_di2 CD15(.C_last(C[14]), .D_last(D[14]), .cidi({C[15],D[15]}));
			  pc2_perm2 pc215(.key({C[15],D[15]}), .temp_P(K[15]));
			  
			  ci_di3 CD16(.C_last(C[15]), .D_last(D[15]), .cidi({C[16],D[16]}));
			  pc2_perm2 pc216(.key({C[16],D[16]}), .temp_P(K[16]));			  

			     assign key1  = K[1];
			     assign key2  = K[2];
			     assign key3  = K[3];
			     assign key4  = K[4];
			     assign key5  = K[5];
			     assign key6  = K[6];
			     assign key7  = K[7];
			     assign key8  = K[8];
			     assign key9  = K[9];
			     assign key10 = K[10];
			     assign key11 = K[11];
			     assign key12 = K[12];
			     assign key13 = K[13];
			     assign key14 = K[14];
			     assign key15 = K[15];
			     assign key16 = K[16];

endmodule

module pc1_perm2(input [64:1] key, output [56:1] temp_P);

                        assign temp_P[56] = key[64-57+1];
						assign temp_P[55] = key[64-49+1];
						assign temp_P[54] = key[64-41+1];
						assign temp_P[53] = key[64-33+1];
						assign temp_P[52] = key[64-25+1];
						assign temp_P[51] = key[64-17+1];
						assign temp_P[50] = key[64-9+1];
						assign temp_P[49] = key[64-1+1];
						assign temp_P[48] = key[64-58+1];
						assign temp_P[47] = key[64-50+1];
						assign temp_P[46] = key[64-42+1];
						assign temp_P[45] = key[64-34+1];
						assign temp_P[44] = key[64-26+1];
						assign temp_P[43] = key[64-18+1];
						assign temp_P[42] = key[64-10+1];
						assign temp_P[41] = key[64-2+1];
						assign temp_P[40] = key[64-59+1];
						assign temp_P[39] = key[64-51+1];
						assign temp_P[38] = key[64-43+1];
						assign temp_P[37] = key[64-35+1];
						assign temp_P[36] = key[64-27+1];
						assign temp_P[35] = key[64-19+1];
						assign temp_P[34] = key[64-11+1];
						assign temp_P[33] = key[64-3+1];
						assign temp_P[32] = key[64-60+1];
						assign temp_P[31] = key[64-52+1];
						assign temp_P[30] = key[64-44+1];
						assign temp_P[29] = key[64-36+1];
						assign temp_P[28] = key[64-63+1];
						assign temp_P[27] = key[64-55+1];
						assign temp_P[26] = key[64-47+1];
						assign temp_P[25] = key[64-39+1];
						assign temp_P[24] = key[64-31+1];
						assign temp_P[23] = key[64-23+1];
						assign temp_P[22] = key[64-15+1];
						assign temp_P[21] = key[64-7+1];
						assign temp_P[20] = key[64-62+1];
						assign temp_P[19] = key[64-54+1];
						assign temp_P[18] = key[64-46+1];
						assign temp_P[17] = key[64-38+1];
						assign temp_P[16] = key[64-30+1];
						assign temp_P[15] = key[64-22+1];
						assign temp_P[14] = key[64-14+1];
						assign temp_P[13] = key[64-6+1];
						assign temp_P[12] = key[64-61+1];
						assign temp_P[11] = key[64-53+1];
						assign temp_P[10] = key[64-45+1];
						assign temp_P[9] = key[64-37+1];
						assign temp_P[8] = key[64-29+1];
						assign temp_P[7] = key[64-21+1];
						assign temp_P[6] = key[64-13+1];
						assign temp_P[5] = key[64-5+1];
						assign temp_P[4] = key[64-28+1];
						assign temp_P[3] = key[64-20+1];
						assign temp_P[2] = key[64-12+1];
						assign temp_P[1] = key[64-4+1];
endmodule

module ci_di2(input [28:1] C_last, D_last, output [56:1] cidi);

			assign cidi = {C_last[26:1], C_last[28:27], D_last[26:1], D_last[28:27]};

endmodule

module ci_di3(input [28:1] C_last, D_last, output [56:1] cidi);

			assign cidi = {C_last[27:1], C_last[28], D_last[27:1], D_last[28]};

endmodule



module pc2_perm2(input [56:1] key, output [48:1] temp_P);

                        assign temp_P[48] = key[56-14+1];
						assign temp_P[47] = key[56-17+1];
						assign temp_P[46] = key[56-11+1];
						assign temp_P[45] = key[56-24+1];
						assign temp_P[44] = key[56-1+1];
						assign temp_P[43] = key[56-5+1];
						assign temp_P[42] = key[56-3+1];
						assign temp_P[41] = key[56-28+1];
						assign temp_P[40] = key[56-15+1];
						assign temp_P[39] = key[56-6+1];
						assign temp_P[38] = key[56-21+1];
						assign temp_P[37] = key[56-10+1];
						assign temp_P[36] = key[56-23+1];
						assign temp_P[35] = key[56-19+1];
						assign temp_P[34] = key[56-12+1];
						assign temp_P[33] = key[56-4+1];
						assign temp_P[32] = key[56-26+1];
						assign temp_P[31] = key[56-8+1];
						assign temp_P[30] = key[56-16+1];
						assign temp_P[29] = key[56-7+1];
						assign temp_P[28] = key[56-27+1];
						assign temp_P[27] = key[56-20+1];
						assign temp_P[26] = key[56-13+1];
						assign temp_P[25] = key[56-2+1];
						assign temp_P[24] = key[56-41+1];
						assign temp_P[23] = key[56-52+1];
						assign temp_P[22] = key[56-31+1];
						assign temp_P[21] = key[56-37+1];
						assign temp_P[20] = key[56-47+1];
						assign temp_P[19] = key[56-55+1];
						assign temp_P[18] = key[56-30+1];
						assign temp_P[17] = key[56-40+1];
						assign temp_P[16] = key[56-51+1];
						assign temp_P[15] = key[56-45+1];
						assign temp_P[14] = key[56-33+1];
						assign temp_P[13] = key[56-48+1];
						assign temp_P[12] = key[56-44+1];
						assign temp_P[11] = key[56-49+1];
						assign temp_P[10] = key[56-39+1];
						assign temp_P[9]  = key[56-56+1];
						assign temp_P[8] = key[56-34+1];
						assign temp_P[7] = key[56-53+1];
						assign temp_P[6] = key[56-46+1];
						assign temp_P[5] = key[56-42+1];
						assign temp_P[4] = key[56-50+1];
						assign temp_P[3] = key[56-36+1];
						assign temp_P[2] = key[56-29+1];
						assign temp_P[1] = key[56-32+1];				
endmodule

module f_fun2(input [32:1] R, input [48:1] K, output [32:1] Rs);
			    wire [48:1] temp1;
			    wire [48:1] temp2;
			    //reg [32:1] temp_after_s_box;
			    wire [32:1] temp_after_perm;
			    wire [5:0] B[8:1];
			    //wire [5:0] id[8:1];
			    wire [3:0] s[8:1];
			    			    
			    perm_E2 permE(.R(R), .Ro(temp1));
			    perm_P2 permP(.s_res({s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8]}), .temp_P(temp_after_perm));
			    
			    assign temp2 = K ^ temp1;			    
			    
			    assign B[1] = temp2[48:43];
			    assign B[2] = temp2[42:37];
			    assign B[3] = temp2[36:31];
			    assign B[4] = temp2[30:25];
	            assign B[5] = temp2[24:19];
		        assign B[6] = temp2[18:13];
			    assign B[7] = temp2[12:7];
			    assign B[8] = temp2[6:1];			    			    
			    
			    sbox2 
			    s1(.B(B[1]), .s_table_id(4'd1), .S(s[1])),
			    s2(.B(B[2]), .s_table_id(4'd2), .S(s[2])),
			    s3(.B(B[3]), .s_table_id(4'd3), .S(s[3])),
			    s4(.B(B[4]), .s_table_id(4'd4), .S(s[4])),
			    s5(.B(B[5]), .s_table_id(4'd5), .S(s[5])),
			    s6(.B(B[6]), .s_table_id(4'd6), .S(s[6])),
			    s7(.B(B[7]), .s_table_id(4'd7), .S(s[7])),
			    s8(.B(B[8]), .s_table_id(4'd8), .S(s[8]));
			    			    
			    assign Rs = temp_after_perm;
			    
			    /*always@(temp_after_perm)
			    begin
			         if(rst)
			             Rs <= 0;
			         else 
			             Rs <= temp_after_perm;
			    end*/
			    //assign 			   
endmodule

module perm_E2(input [32:1] R, output [48:1] Ro);

                        assign Ro[48] = R[32-32+1];
						assign Ro[47] = R[32-1+1];
						assign Ro[46] = R[32-2+1];
						assign Ro[45] = R[32-3+1];
						assign Ro[44] = R[32-4+1];
						assign Ro[43] = R[32-5+1];
						assign Ro[42] = R[32-4+1];
						assign Ro[41] = R[32-5+1];
						assign Ro[40] = R[32-6+1];
						assign Ro[39] = R[32-7+1];
						assign Ro[38] = R[32-8+1];
						assign Ro[37] = R[32-9+1];
						assign Ro[36] = R[32-8+1];
						assign Ro[35] = R[32-9+1];
						assign Ro[34] = R[32-10+1];
						assign Ro[33] = R[32-11+1];
						assign Ro[32] = R[32-12+1];
						assign Ro[31] = R[32-13+1];
						assign Ro[30] = R[32-12+1];
						assign Ro[29] = R[32-13+1];
						assign Ro[28] = R[32-14+1];
						assign Ro[27] = R[32-15+1];
						assign Ro[26] = R[32-16+1];
						assign Ro[25] = R[32-17+1];
						assign Ro[24] = R[32-16+1];
						assign Ro[23] = R[32-17+1];
						assign Ro[22] = R[32-18+1];
						assign Ro[21] = R[32-19+1];
						assign Ro[20] = R[32-20+1];
						assign Ro[19] = R[32-21+1];
						assign Ro[18] = R[32-20+1];
						assign Ro[17] = R[32-21+1];
						assign Ro[16] = R[32-22+1];
						assign Ro[15] = R[32-23+1];
						assign Ro[14] = R[32-24+1];
						assign Ro[13] = R[32-25+1];
						assign Ro[12] = R[32-24+1];
						assign Ro[11] = R[32-25+1];
						assign Ro[10] = R[32-26+1];
						assign Ro[9] = R[32-27+1];
						assign Ro[8] = R[32-28+1];
						assign Ro[7] = R[32-29+1];
						assign Ro[6] = R[32-28+1];
						assign Ro[5] = R[32-29+1];
						assign Ro[4] = R[32-30+1];
						assign Ro[3] = R[32-31+1];
						assign Ro[2] = R[32-32+1];
						assign Ro[1] = R[32-1+1];
									                   
endmodule

module perm_P2(input [32:1] s_res, output [32:1] temp_P);

                        assign temp_P[32] = s_res[32-16+1];
						assign temp_P[31] = s_res[32-7+1];
						assign temp_P[30] = s_res[32-20+1];
			     		assign temp_P[29] = s_res[32-21+1];
						assign temp_P[28] = s_res[32-29+1];
						assign temp_P[27] = s_res[32-12+1];
						assign temp_P[26] = s_res[32-28+1];
						assign temp_P[25] = s_res[32-17+1];
						assign temp_P[24] = s_res[32-1+1];
						assign temp_P[23] = s_res[32-15+1];
						assign temp_P[22] = s_res[32-23+1];
						assign temp_P[21] = s_res[32-26+1];
						assign temp_P[20] = s_res[32-5+1];
						assign temp_P[19] = s_res[32-18+1];
						assign temp_P[18] = s_res[32-31+1];
						assign temp_P[17] = s_res[32-10+1];
						assign temp_P[16] = s_res[32-2+1];
						assign temp_P[15] = s_res[32-8+1];
						assign temp_P[14] = s_res[32-24+1];
						assign temp_P[13] = s_res[32-14+1];
						assign temp_P[12] = s_res[32-32+1];
						assign temp_P[11] = s_res[32-27+1];
						assign temp_P[10] = s_res[32-3+1];
						assign temp_P[9] = s_res[32-9+1];
						assign temp_P[8] = s_res[32-19+1];
						assign temp_P[7] = s_res[32-13+1];
						assign temp_P[6] = s_res[32-30+1];
						assign temp_P[5] = s_res[32-6+1];
						assign temp_P[4] = s_res[32-22+1];
						assign temp_P[3] = s_res[32-11+1];
						assign temp_P[2] = s_res[32-4+1];
						assign temp_P[1] = s_res[32-25+1];
endmodule

module perm_IP_inv2(input [64:1] message, output [64:1] temp_msg);
			
                        assign temp_msg[1] = message[40];
						assign temp_msg[2] = message[8];
						assign temp_msg[3] = message[48];
						assign temp_msg[4] = message[16];
						assign temp_msg[5] = message[56];
						assign temp_msg[6] = message[24];
						assign temp_msg[7] = message[64];
						assign temp_msg[8] = message[32];
						assign temp_msg[9] = message[39];
						assign temp_msg[10] = message[7];
						assign temp_msg[11] = message[47];
						assign temp_msg[12] = message[15];
						assign temp_msg[13] = message[55];
						assign temp_msg[14] = message[23];
						assign temp_msg[15] = message[63];
						assign temp_msg[16] = message[31];
						assign temp_msg[17] = message[38];
						assign temp_msg[18] = message[6];
						assign temp_msg[19] = message[46];
						assign temp_msg[20] = message[14];
						assign temp_msg[21] = message[54];
						assign temp_msg[22] = message[22];
						assign temp_msg[23] = message[62];
						assign temp_msg[24] = message[30];
						assign temp_msg[25] = message[37];
						assign temp_msg[26] = message[5];
						assign temp_msg[27] = message[45];
						assign temp_msg[28] = message[13];
						assign temp_msg[29] = message[53];
						assign temp_msg[30] = message[21];
						assign temp_msg[31] = message[61];
						assign temp_msg[32] = message[29];
						assign temp_msg[33] = message[36];
						assign temp_msg[34] = message[4];
						assign temp_msg[35] = message[44];
						assign temp_msg[36] = message[12];
						assign temp_msg[37] = message[52];
						assign temp_msg[38] = message[20];
						assign temp_msg[39] = message[60];
						assign temp_msg[40] = message[28];
						assign temp_msg[41] = message[35];
						assign temp_msg[42] = message[3];
						assign temp_msg[43] = message[43];
						assign temp_msg[44] = message[11];
						assign temp_msg[45] = message[51];
						assign temp_msg[46] = message[19];
						assign temp_msg[47] = message[59];
						assign temp_msg[48] = message[27];
						assign temp_msg[49] = message[34];
						assign temp_msg[50] = message[2];
						assign temp_msg[51] = message[42];
						assign temp_msg[52] = message[10];
						assign temp_msg[53] = message[50];
						assign temp_msg[54] = message[18];
						assign temp_msg[55] = message[58];
						assign temp_msg[56] = message[26];
						assign temp_msg[57] = message[33];
						assign temp_msg[58] = message[1];
						assign temp_msg[59] = message[41];
						assign temp_msg[60] = message[9];
						assign temp_msg[61] = message[49];
						assign temp_msg[62] = message[17];
						assign temp_msg[63] = message[57];
						assign temp_msg[64] = message[25];

endmodule

module sbox2(input [6:1] B, input [4:1] s_table_id, output reg[3:0] S);

                wire [2:1] i;
			    wire [4:1] j;
			    wire [3:0] S1[3:0][15:0];
			    wire [3:0] S2[3:0][15:0];
			    wire [3:0] S3[3:0][15:0];
			    wire [3:0] S4[3:0][15:0];
			    wire [3:0] S5[3:0][15:0];
			    wire [3:0] S6[3:0][15:0];
			    wire [3:0] S7[3:0][15:0];
			    wire [3:0] S8[3:0][15:0];
			    

                    assign S1[0][0] = 14;
					assign	S1[0][1] = 4;
					assign	S1[0][2] = 13;
					assign	S1[0][3] = 1;
					assign	S1[0][4] = 2;
					assign	S1[0][5] = 15;
					assign	S1[0][6] = 11;
					assign	S1[0][7] = 8;
					assign	S1[0][8] = 3;
					assign	S1[0][9] = 10;
					assign	S1[0][10] = 6;
					assign	S1[0][11] = 12;
					assign	S1[0][12] = 5;
					assign	S1[0][13] = 9;
					assign	S1[0][14] = 0;
					assign	S1[0][15] = 7;
					assign	S1[1][0] = 0;
					assign	S1[1][1] = 15;
					assign	S1[1][2] = 7;
					assign	S1[1][3] = 4;
					assign	S1[1][4] = 14;
					assign	S1[1][5] = 2;
					assign	S1[1][6] = 13;
					assign	S1[1][7] = 1;
					assign	S1[1][8] = 10;
					assign	S1[1][9] = 6;
					assign	S1[1][10] = 12;
					assign	S1[1][11] = 11;
					assign	S1[1][12] = 9;
					assign	S1[1][13] = 5;
					assign	S1[1][14] = 3;
					assign	S1[1][15] = 8;
					assign	S1[2][0] = 4;
					assign	S1[2][1] = 1;
					assign	S1[2][2] = 14;
					assign	S1[2][3] = 8;
					assign	S1[2][4] = 13;
					assign	S1[2][5] = 6;
					assign	S1[2][6] = 2;
					assign	S1[2][7] = 11;
					assign	S1[2][8] = 15;
					assign	S1[2][9] = 12;
					assign	S1[2][10] = 9;
					assign	S1[2][11] = 7;
					assign	S1[2][12] = 3;
					assign	S1[2][13] = 10;
					assign	S1[2][14] = 5;
					assign	S1[2][15] = 0;
					assign	S1[3][0] = 15;
					assign	S1[3][1] = 12;
					assign	S1[3][2] = 8;
					assign	S1[3][3] = 2;
					assign	S1[3][4] = 4;
					assign	S1[3][5] = 9;
					assign	S1[3][6] = 1;
					assign	S1[3][7] = 7;
					assign	S1[3][8] = 5;
					assign	S1[3][9] = 11;
					assign	S1[3][10] = 3;
					assign	S1[3][11] = 14;
					assign	S1[3][12] = 10;
					assign	S1[3][13] = 0;
					assign	S1[3][14] = 6;
					assign	S1[3][15] = 13;
					assign	S2[0][0] = 15;
					assign	S2[0][1] = 1;
					assign	S2[0][2] = 8;
					assign	S2[0][3] = 14;
					assign	S2[0][4] = 6;
					assign	S2[0][5] = 11;
					assign	S2[0][6] = 3;
					assign	S2[0][7] = 4;
					assign	S2[0][8] = 9;
					assign	S2[0][9] = 7;
					assign	S2[0][10] = 2;
					assign	S2[0][11] = 13;
					assign	S2[0][12] = 12;
					assign	S2[0][13] = 0;
					assign	S2[0][14] = 5;
					assign	S2[0][15] = 10;
					assign	S2[1][0] = 3;
					assign	S2[1][1] = 13;
					assign	S2[1][2] = 4;
					assign	S2[1][3] = 7;
					assign	S2[1][4] = 15;
					assign	S2[1][5] = 2;
					assign	S2[1][6] = 8;
					assign	S2[1][7] = 14;
					assign	S2[1][8] = 12;
					assign	S2[1][9] = 0;
					assign	S2[1][10] = 1;
					assign	S2[1][11] = 10;
					assign	S2[1][12] = 6;
					assign	S2[1][13] = 9;
					assign	S2[1][14] = 11;
					assign	S2[1][15] = 5;
					assign	S2[2][0] = 0;
					assign	S2[2][1] = 14;
					assign	S2[2][2] = 7;
					assign	S2[2][3] = 11;
					assign	S2[2][4] = 10;
					assign	S2[2][5] = 4;
					assign	S2[2][6] = 13;
					assign	S2[2][7] = 1;
					assign	S2[2][8] = 5;
					assign	S2[2][9] = 8;
					assign	S2[2][10] = 12;
					assign	S2[2][11] = 6;
					assign	S2[2][12] = 9;
					assign	S2[2][13] = 3;
					assign	S2[2][14] = 2;
					assign	S2[2][15] = 15;
					assign	S2[3][0] = 13;
					assign	S2[3][1] = 8;
					assign	S2[3][2] = 10;
					assign	S2[3][3] = 1;
					assign	S2[3][4] = 3;
					assign	S2[3][5] = 15;
					assign	S2[3][6] = 4;
					assign	S2[3][7] = 2;
					assign	S2[3][8] = 11;
					assign	S2[3][9] = 6;
					assign	S2[3][10] = 7;
					assign	S2[3][11] = 12;
					assign	S2[3][12] = 0;
					assign	S2[3][13] = 5;
					assign	S2[3][14] = 14;
					assign	S2[3][15] = 9;
					assign	S3[0][0] = 10;
					assign	S3[0][1] = 0;
					assign	S3[0][2] = 9;
					assign	S3[0][3] = 14;
					assign	S3[0][4] = 6;
					assign	S3[0][5] = 3;
					assign	S3[0][6] = 15;
					assign	S3[0][7] = 5;
					assign	S3[0][8] = 1;
					assign	S3[0][9] = 13;
					assign	S3[0][10] = 12;
					assign	S3[0][11] = 7;
					assign	S3[0][12] = 11;
					assign	S3[0][13] = 4;
					assign	S3[0][14] = 2;
					assign	S3[0][15] = 8;
					assign	S3[1][0] = 13;
					assign	S3[1][1] = 7;
					assign	S3[1][2] = 0;
					assign	S3[1][3] = 9;
					assign	S3[1][4] = 3;
					assign	S3[1][5] = 4;
					assign	S3[1][6] = 6;
					assign	S3[1][7] = 10;
					assign	S3[1][8] = 2;
					assign	S3[1][9] = 8;
					assign	S3[1][10] = 5;
					assign	S3[1][11] = 14;
					assign	S3[1][12] = 12;
					assign	S3[1][13] = 11;
					assign	S3[1][14] = 15;
					assign	S3[1][15] = 1;
					assign	S3[2][0] = 13;
					assign	S3[2][1] = 6;
					assign	S3[2][2] = 4;
					assign	S3[2][3] = 9;
					assign	S3[2][4] = 8;
					assign	S3[2][5] = 15;
					assign	S3[2][6] = 3;
					assign	S3[2][7] = 0;
					assign	S3[2][8] = 11;
					assign	S3[2][9] = 1;
					assign	S3[2][10] = 2;
					assign	S3[2][11] = 12;
					assign	S3[2][12] = 5;
					assign	S3[2][13] = 10;
					assign	S3[2][14] = 14;
					assign	S3[2][15] = 7;
					assign	S3[3][0] = 1;
					assign	S3[3][1] = 10;
					assign	S3[3][2] = 13;
					assign	S3[3][3] = 0;
					assign	S3[3][4] = 6;
					assign	S3[3][5] = 9;
					assign	S3[3][6] = 8;
					assign	S3[3][7] = 7;
					assign	S3[3][8] = 4;
					assign	S3[3][9] = 15;
					assign	S3[3][10] = 14;
					assign	S3[3][11] = 3;
					assign	S3[3][12] = 11;
					assign	S3[3][13] = 5;
					assign	S3[3][14] = 2;
					assign	S3[3][15] = 12;
					assign	S4[0][0] = 7;
					assign	S4[0][1] = 13;
					assign	S4[0][2] = 14;
					assign	S4[0][3] = 3;
					assign	S4[0][4] = 0;
					assign	S4[0][5] = 6;
					assign	S4[0][6] = 9;
					assign	S4[0][7] = 10;
					assign	S4[0][8] = 1;
					assign	S4[0][9] = 2;
					assign	S4[0][10] = 8;
					assign	S4[0][11] = 5;
					assign	S4[0][12] = 11;
					assign	S4[0][13] = 12;
					assign	S4[0][14] = 4;
					assign	S4[0][15] = 15;
					assign	S4[1][0] = 13;
					assign	S4[1][1] = 8;
					assign	S4[1][2] = 11;
					assign	S4[1][3] = 5;
					assign	S4[1][4] = 6;
					assign	S4[1][5] = 15;
					assign	S4[1][6] = 0;
					assign	S4[1][7] = 3;
					assign	S4[1][8] = 4;
					assign	S4[1][9] = 7;
					assign	S4[1][10] = 2;
					assign	S4[1][11] = 12;
					assign	S4[1][12] = 1;
					assign	S4[1][13] = 10;
					assign	S4[1][14] = 14;
					assign	S4[1][15] = 9;
					assign	S4[2][0] = 10;
					assign	S4[2][1] = 6;
					assign	S4[2][2] = 9;
					assign	S4[2][3] = 0;
					assign	S4[2][4] = 12;
					assign	S4[2][5] = 11;
					assign	S4[2][6] = 7;
					assign	S4[2][7] = 13;
					assign	S4[2][8] = 15;
					assign	S4[2][9] = 1;
					assign	S4[2][10] = 3;
					assign	S4[2][11] = 14;
					assign	S4[2][12] = 5;
					assign	S4[2][13] = 2;
					assign	S4[2][14] = 8;
					assign	S4[2][15] = 4;
					assign	S4[3][0] = 3;
					assign	S4[3][1] = 15;
					assign	S4[3][2] = 0;
					assign	S4[3][3] = 6;
					assign	S4[3][4] = 10;
					assign	S4[3][5] = 1;
					assign	S4[3][6] = 13;
					assign	S4[3][7] = 8;
					assign	S4[3][8] = 9;
					assign	S4[3][9] = 4;
					assign	S4[3][10] = 5;
					assign	S4[3][11] = 11;
					assign	S4[3][12] = 12;
					assign	S4[3][13] = 7;
					assign	S4[3][14] = 2;
					assign	S4[3][15] = 14;
					assign	S5[0][0] = 2;
					assign	S5[0][1] = 12;
					assign	S5[0][2] = 4;
					assign	S5[0][3] = 1;
					assign	S5[0][4] = 7;
					assign	S5[0][5] = 10;
					assign	S5[0][6] = 11;
					assign	S5[0][7] = 6;
					assign	S5[0][8] = 8;
					assign	S5[0][9] = 5;
					assign	S5[0][10] = 3;
					assign	S5[0][11] = 15;
					assign	S5[0][12] = 13;
					assign	S5[0][13] = 0;
					assign	S5[0][14] = 14;
					assign	S5[0][15] = 9;
					assign	S5[1][0] = 14;
					assign	S5[1][1] = 11;
					assign	S5[1][2] = 2;
					assign	S5[1][3] = 12;
					assign	S5[1][4] = 4;
					assign	S5[1][5] = 7;
					assign	S5[1][6] = 13;
					assign	S5[1][7] = 1;
					assign	S5[1][8] = 5;
					assign	S5[1][9] = 0;
					assign	S5[1][10] = 15;
					assign	S5[1][11] = 10;
					assign	S5[1][12] = 3;
					assign	S5[1][13] = 9;
					assign	S5[1][14] = 8;
					assign	S5[1][15] = 6;
					assign	S5[2][0] = 4;
					assign	S5[2][1] = 2;
					assign	S5[2][2] = 1;
					assign	S5[2][3] = 11;
					assign	S5[2][4] = 10;
					assign	S5[2][5] = 13;
					assign	S5[2][6] = 7;
					assign	S5[2][7] = 8;
					assign	S5[2][8] = 15;
					assign	S5[2][9] = 9;
					assign	S5[2][10] = 12;
					assign	S5[2][11] = 5;
					assign	S5[2][12] = 6;
					assign	S5[2][13] = 3;
					assign	S5[2][14] = 0;
					assign	S5[2][15] = 14;
					assign	S5[3][0] = 11;
					assign	S5[3][1] = 8;
					assign	S5[3][2] = 12;
					assign	S5[3][3] = 7;
					assign	S5[3][4] = 1;
					assign	S5[3][5] = 14;
					assign	S5[3][6] = 2;
					assign	S5[3][7] = 13;
					assign	S5[3][8] = 6;
					assign	S5[3][9] = 15;
					assign	S5[3][10] = 0;
					assign	S5[3][11] = 9;
					assign	S5[3][12] = 10;
					assign	S5[3][13] = 4;
					assign	S5[3][14] = 5;
					assign	S5[3][15] = 3;
					assign	S6[0][0] = 12;
					assign	S6[0][1] = 1;
					assign	S6[0][2] = 10;
					assign	S6[0][3] = 15;
					assign	S6[0][4] = 9;
					assign	S6[0][5] = 2;
					assign	S6[0][6] = 6;
					assign	S6[0][7] = 8;
					assign	S6[0][8] = 0;
					assign	S6[0][9] = 13;
					assign	S6[0][10] = 3;
					assign	S6[0][11] = 4;
					assign	S6[0][12] = 14;
					assign	S6[0][13] = 7;
					assign	S6[0][14] = 5;
					assign	S6[0][15] = 11;
					assign	S6[1][0] = 10;
					assign	S6[1][1] = 15;
					assign	S6[1][2] = 4;
					assign	S6[1][3] = 2;
					assign	S6[1][4] = 7;
					assign	S6[1][5] = 12;
					assign	S6[1][6] = 9;
					assign	S6[1][7] = 5;
					assign	S6[1][8] = 6;
					assign	S6[1][9] = 1;
					assign	S6[1][10] = 13;
					assign	S6[1][11] = 14;
					assign	S6[1][12] = 0;
					assign	S6[1][13] = 11;
					assign	S6[1][14] = 3;
					assign	S6[1][15] = 8;
					assign	S6[2][0] = 9;
					assign	S6[2][1] = 14;
					assign	S6[2][2] = 15;
					assign	S6[2][3] = 5;
					assign	S6[2][4] = 2;
					assign	S6[2][5] = 8;
					assign	S6[2][6] = 12;
					assign	S6[2][7] = 3;
					assign	S6[2][8] = 7;
					assign	S6[2][9] = 0;
					assign	S6[2][10] = 4;
					assign	S6[2][11] = 10;
					assign	S6[2][12] = 1;
					assign	S6[2][13] = 13;
					assign	S6[2][14] = 11;
					assign	S6[2][15] = 6;
					assign	S6[3][0] = 4;
					assign	S6[3][1] = 3;
					assign	S6[3][2] = 2;
					assign	S6[3][3] = 12;
					assign	S6[3][4] = 9;
					assign	S6[3][5] = 5;
					assign	S6[3][6] = 15;
					assign	S6[3][7] = 10;
					assign	S6[3][8] = 11;
					assign	S6[3][9] = 14;
					assign	S6[3][10] = 1;
					assign	S6[3][11] = 7;
					assign	S6[3][12] = 6;
					assign	S6[3][13] = 0;
					assign	S6[3][14] = 8;
					assign	S6[3][15] = 13;
					assign	S7[0][0] = 4;
					assign	S7[0][1] = 11;
					assign	S7[0][2] = 2;
					assign	S7[0][3] = 14;
					assign	S7[0][4] = 15;
					assign	S7[0][5] = 0;
					assign	S7[0][6] = 8;
					assign	S7[0][7] = 13;
					assign	S7[0][8] = 3;
					assign	S7[0][9] = 12;
					assign	S7[0][10] = 9;
					assign	S7[0][11] = 7;
					assign	S7[0][12] = 5;
					assign	S7[0][13] = 10;
					assign	S7[0][14] = 6;
					assign	S7[0][15] = 1;
					assign	S7[1][0] = 13;
					assign	S7[1][1] = 0;
					assign	S7[1][2] = 11;
					assign	S7[1][3] = 7;
					assign	S7[1][4] = 4;
					assign	S7[1][5] = 9;
					assign	S7[1][6] = 1;
					assign	S7[1][7] = 10;
					assign	S7[1][8] = 14;
					assign	S7[1][9] = 3;
					assign	S7[1][10] = 5;
					assign	S7[1][11] = 12;
					assign	S7[1][12] = 2;
					assign	S7[1][13] = 15;
					assign	S7[1][14] = 8;
					assign	S7[1][15] = 6;
					assign	S7[2][0] = 1;
					assign	S7[2][1] = 4;
					assign	S7[2][2] = 11;
					assign	S7[2][3] = 13;
					assign	S7[2][4] = 12;
					assign	S7[2][5] = 3;
					assign	S7[2][6] = 7;
					assign	S7[2][7] = 14;
					assign	S7[2][8] = 10;
					assign	S7[2][9] = 15;
					assign	S7[2][10] = 6;
					assign	S7[2][11] = 8;
					assign	S7[2][12] = 0;
					assign	S7[2][13] = 5;
					assign	S7[2][14] = 9;
					assign	S7[2][15] = 2;
					assign	S7[3][0] = 6;
					assign	S7[3][1] = 11;
					assign	S7[3][2] = 13;
					assign	S7[3][3] = 8;
					assign	S7[3][4] = 1;
					assign	S7[3][5] = 4;
					assign	S7[3][6] = 10;
					assign	S7[3][7] = 7;
					assign	S7[3][8] = 9;
					assign	S7[3][9] = 5;
					assign	S7[3][10] = 0;
					assign	S7[3][11] = 15;
					assign	S7[3][12] = 14;
					assign	S7[3][13] = 2;
					assign	S7[3][14] = 3;
					assign	S7[3][15] = 12;
					assign	S8[0][0] = 13;
					assign	S8[0][1] = 2;
					assign	S8[0][2] = 8;
					assign	S8[0][3] = 4;
					assign	S8[0][4] = 6;
					assign	S8[0][5] = 15;
					assign	S8[0][6] = 11;
					assign	S8[0][7] = 1;
					assign	S8[0][8] = 10;
					assign	S8[0][9] = 9;
					assign	S8[0][10] = 3;
					assign	S8[0][11] = 14;
					assign	S8[0][12] = 5;
					assign	S8[0][13] = 0;
					assign	S8[0][14] = 12;
					assign	S8[0][15] = 7;
					assign	S8[1][0] = 1;
					assign	S8[1][1] = 15;
					assign	S8[1][2] = 13;
					assign	S8[1][3] = 8;
					assign	S8[1][4] = 10;
					assign	S8[1][5] = 3;
					assign	S8[1][6] = 7;
					assign	S8[1][7] = 4;
					assign	S8[1][8] = 12;
					assign	S8[1][9] = 5;
					assign	S8[1][10] = 6;
					assign	S8[1][11] = 11;
					assign	S8[1][12] = 0;
					assign	S8[1][13] = 14;
					assign	S8[1][14] = 9;
					assign	S8[1][15] = 2;
					assign	S8[2][0] = 7;
					assign	S8[2][1] = 11;
					assign	S8[2][2] = 4;
					assign	S8[2][3] = 1;
					assign	S8[2][4] = 9;
					assign	S8[2][5] = 12;
					assign	S8[2][6] = 14;
					assign	S8[2][7] = 2;
					assign	S8[2][8] = 0;
					assign	S8[2][9] = 6;
					assign	S8[2][10] = 10;
					assign	S8[2][11] = 13;
					assign	S8[2][12] = 15;
					assign	S8[2][13] = 3;
					assign	S8[2][14] = 5;
					assign	S8[2][15] = 8;
					assign	S8[3][0] = 2;
					assign	S8[3][1] = 1;
					assign	S8[3][2] = 14;
					assign	S8[3][3] = 7;
					assign	S8[3][4] = 4;
					assign	S8[3][5] = 10;
					assign	S8[3][6] = 8;
					assign	S8[3][7] = 13;
					assign	S8[3][8] = 15;
					assign	S8[3][9] = 12;
					assign	S8[3][10] = 9;
					assign	S8[3][11] = 0;
					assign	S8[3][12] = 3;
					assign	S8[3][13] = 5;
					assign	S8[3][14] = 6;
					assign	S8[3][15] = 11;
			

			      assign i[2:1] = {B[6], B[1]};
			      assign j[4:1] = B[5:2];
			
                //always@(posedge clk)
                always@(i or j)
                begin
			      case(s_table_id)
			        4'b01: S = S1[i][j];
			        4'b10: S = S2[i][j];
			        4'b11: S = S3[i][j];
			        4'b100: S = S4[i][j];
			        4'b101: S = S5[i][j];
			        4'b110: S = S6[i][j];
			        4'b111: S = S7[i][j];
			        4'b1000: S = S8[i][j];
			      endcase
			   end
endmodule

