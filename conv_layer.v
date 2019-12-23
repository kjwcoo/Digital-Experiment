`timescale 1 ps / 1 ps

module conv_layer (
    input clk,
    input start,
    output reg finish,
    output reg [31:0] AHB_INTERFACE_0_haddr,
    output reg [2:0] AHB_INTERFACE_0_hburst,
    output reg [3:0] AHB_INTERFACE_0_hprot,
    input [31:0] AHB_INTERFACE_0_hrdata,
    output reg AHB_INTERFACE_0_hready_in,
    input AHB_INTERFACE_0_hready_out,
    input AHB_INTERFACE_0_hresp,
    output reg [2:0] AHB_INTERFACE_0_hsize,
    output reg [1:0] AHB_INTERFACE_0_htrans,
    output reg [31:0] AHB_INTERFACE_0_hwdata,
    output reg AHB_INTERFACE_0_hwrite,
    output reg AHB_INTERFACE_0_sel
    );
    
/******* Register declaration START *******/     
    /* "count" registers */
    reg [12:0] cntTotal; // count the number of Pool calculation
    reg [12:0] cntConv; // count the number of convolution calculation
    reg [1:0] cntDataPool;  // count the number of data read for Pool
    reg [3:0] cntDataImage; // count the number of data read for Image
    reg [1:0] cntDataImageWarm; // count the number of warm data read for Image
    reg [3:0] cntDataWeight;    // count the number of data read for Weight
    
    /* FSM registers */
    reg [2:0] state;    // Overall FSM
    parameter COLD_WEIGHT = 3'b000;  // obtaining weight matrix 
    parameter COLD_FETCH = 3'b001;   // obtaining whole(3*3) matrix
    parameter WARM_FETCH = 3'b010;   // obtaining partial(3*1) matrix
    parameter CALCULATE = 3'b011;    // do convolution
    parameter POOL = 3'b100;        // do pooling
    
    reg read;   // read FSM
    parameter READ_REQ = 1'b0;  // reqeusting read
    parameter READ_READ = 1'b1; // obatining read data
    
    reg [1:0] warmRead;   // read FSM specially designed for WARM_FETCH
    parameter WARM_UPDATE = 2'b00;  // updating 6 elements
    parameter WARM_REQ = 2'b01;     // requesting read
    parameter WARM_READ = 2'b10;    // obtaining read data
    
    reg [2:0] calc;     // calculating FSM
    parameter NEG = 3'b000;  // 2's complementing data
    parameter MUL_PREPARE = 3'b001; // absoluting data and sign-extending to 16 bits
    parameter MUL = 3'b010; // multiplying data considering original signs
    parameter ADD = 3'b011;  // adding 9 components
    parameter RESULT = 3'b100;    // reduce 16 bits into 8 bits
    parameter LOAD = 3'b101; // loading to array C
    parameter WRAP_UP = 3'b110; // wrapping up write
    
    reg [2:0] poolState;    // pooling FSM
    parameter POOL_WAIT = 3'b000;   // buffering state between wrap_up and req
    parameter POOL_REQ = 3'b001; // requesting for read from BRAM
    parameter POOL_READ = 3'b010;   // reading data and ReLU
    parameter POOL_CALC = 3'b011;   // requesting for write to BRAM (PS)
    parameter POOL_DONE = 3'b100;   // finishing write request
    parameter POOL_WRAP_UP = 3'b101;    // wrapping up write and determine current position
          
    /* registers for memory data */
    reg [71:0] dataImage;
    reg [71:0] dataWeight;
    reg [7:0] dataOut;
    reg [31:0] addrImage;
    reg [31:0] nextAddrImage;   // for next calculation
    reg [31:0] addrWeight;
    reg [31:0] addrOut;
    // parameters for BRAM communicating with PS
    parameter startAddrImage = 32'h4000_0000;
    parameter startAddrWeight = 32'h4001_0000;
    parameter startAddrOut = 32'h4002_0000;
    
    /* BRAM IN/OUT */
    // IN (Port A)
    reg [7:0] dataTemp;
    reg [12:0] addrTemp;
    reg ena;
    reg wea;
    // OUT (Port B)
    wire [7:0] dataPool;
    reg [12:0] addrPool;
    reg [12:0] nextAddrPool;    // for next calculation
    reg enb;
    
    /* registers for convolution calculation */
    reg [8:0] signWeight;   // saving sign information for 9 elements
    reg [8:0] signImage;
    reg [71:0] negDataWeight;   // saving negative values for 9 elements
    reg [71:0] negDataImage;
    reg [15:0] dataTempImage1;  // temporary values for calculation 
    reg [15:0] dataTempWeight1;
    reg [15:0] dataTempImage2;
    reg [15:0] dataTempWeight2;    
    reg [15:0] dataTempImage3;
    reg [15:0] dataTempWeight3;
    reg [15:0] dataTempImage4;
    reg [15:0] dataTempWeight4;
    reg [15:0] dataTempImage5;
    reg [15:0] dataTempWeight5;
    reg [15:0] dataTempImage6;
    reg [15:0] dataTempWeight6;
    reg [15:0] dataTempImage7;
    reg [15:0] dataTempWeight7;
    reg [15:0] dataTempImage8;
    reg [15:0] dataTempWeight8;
    reg [15:0] dataTempImage9;
    reg [15:0] dataTempWeight9;
   
    /* registers for pooling */
    reg pool;   // determines pooling is ready or not
    reg poolReg;    // to get rising edge of pool
    //reg startPool;
    wire startPool;
    reg finishPool;
/******* Register declaration END *******/    

/******* BRAM START *******/
    blk_mem_gen_0 blk_mem_gen_0_i (
        // Port A
        .addra(addrTemp),
        .clka(clk),
        .dina(dataTemp),
        .ena(ena),
        .wea(wea),
        // Port B
        .addrb(addrPool),
        .clkb(clk),
        .doutb(dataPool),
        .enb(enb),
        .web(0)
    );
/******* BRAM END *******/

/******* Data update START *******/
    /* Action FSM for calculating part (1/2 of whole) */
    always @(posedge clk)
    begin
        if(!start)
        begin
            state <= COLD_WEIGHT;
            read <= READ_REQ;
            warmRead <= WARM_UPDATE;
            calc <= NEG;
            
            cntConv <= 0;
            cntDataImage <= 0;
            cntDataImageWarm <= 0;
            cntDataWeight <= 0;
            
            dataImage <= 0;
            dataWeight <= 0;
            
            addrImage <= startAddrImage;
            nextAddrImage <= 0;
            addrWeight <= startAddrWeight;
            addrTemp <= 0;
            
            wea <= 0;   // This part controls port A
            ena <= 0;
            
            cntTotal <= 0;
            cntDataPool <= 0;
            
            
            dataOut <= 0;
            
            poolState <= POOL_REQ;
            
            finish <= 0; 
            
            addrPool <= 0;
            
            addrOut <= startAddrOut;
        end
        else
        begin
            /* FSM for calculating part */
            case(state)
                COLD_WEIGHT:
                    begin
                        case(read)
                            READ_REQ:
                                begin
                                    if(AHB_INTERFACE_0_hready_out == 1) // if ready to read
                                    begin
                                        AHB_INTERFACE_0_htrans <= 2'b10;
                                        AHB_INTERFACE_0_haddr <= addrWeight;
                                        AHB_INTERFACE_0_hburst <= 0;
                                        AHB_INTERFACE_0_hsize <= 3'b000;    // byte-sized transfer
                                        AHB_INTERFACE_0_hready_in <= 1;
                                        AHB_INTERFACE_0_sel <= 1;
                                        AHB_INTERFACE_0_hprot <= 0;
                                        AHB_INTERFACE_0_hwrite <= 0;
                                        
                                        read <= READ_READ;
                                    end
                                    else read <= read;
                                end
                            READ_READ:
                                begin
                                    if(AHB_INTERFACE_0_hready_out == 1) // if requested data return
                                    begin
                                        case(cntDataWeight)
                                            4'b0000: dataWeight[7:0] <= AHB_INTERFACE_0_hrdata;
                                            4'b0001: dataWeight[15:8] <= AHB_INTERFACE_0_hrdata;
                                            4'b0010: dataWeight[23:16] <= AHB_INTERFACE_0_hrdata;
                                            4'b0011: dataWeight[31:24] <= AHB_INTERFACE_0_hrdata;
                                            4'b0100: dataWeight[39:32] <= AHB_INTERFACE_0_hrdata;
                                            4'b0101: dataWeight[47:40] <= AHB_INTERFACE_0_hrdata;
                                            4'b0110: dataWeight[55:48] <= AHB_INTERFACE_0_hrdata;
                                            4'b0111: dataWeight[63:56] <= AHB_INTERFACE_0_hrdata;
                                            4'b1000: dataWeight[71:64] <= AHB_INTERFACE_0_hrdata;
                                        endcase
                                    
                                        AHB_INTERFACE_0_sel <= 0;
                                        AHB_INTERFACE_0_hprot <= 0;
                                        AHB_INTERFACE_0_hready_in <= 1;
                                    
                                        addrWeight <= addrWeight + 1;
                                        if(cntDataWeight != 4'b1000) cntDataWeight <= cntDataWeight + 1;
                                        else begin cntDataWeight <= 0; state <= COLD_FETCH; end
                                        
                                        read <= READ_REQ;
                                    end
                                    else begin AHB_INTERFACE_0_hready_in <= 0; read <= read; end
                                
                                    AHB_INTERFACE_0_htrans <= 2'b00;
                                end
                        endcase
                    end
                COLD_FETCH:
                    begin
                        case(read)
                            READ_REQ:
                                begin
                                    if(AHB_INTERFACE_0_hready_out == 1) // reqeusting read for image
                                    begin
                                        AHB_INTERFACE_0_htrans <= 2'b10;
                                        AHB_INTERFACE_0_haddr <= addrImage;
                                        AHB_INTERFACE_0_hburst <= 0;
                                        AHB_INTERFACE_0_hsize <= 3'b000;
                                        AHB_INTERFACE_0_hready_in <= 1;
                                        AHB_INTERFACE_0_sel <= 1;
                                        AHB_INTERFACE_0_hprot <= 1;
                                        AHB_INTERFACE_0_hwrite <= 0;
                                        
                                        read <= READ_READ;
                                    end
                                    else read <= read;    
                                end
                            READ_READ:
                                begin
                                    if(AHB_INTERFACE_0_hready_out == 1) // if request data return
                                    begin
                                        case(cntDataImage)
                                            4'b0000: dataImage[7:0] <= AHB_INTERFACE_0_hrdata;
                                            4'b0001: dataImage[15:8] <= AHB_INTERFACE_0_hrdata;
                                            4'b0010: dataImage[23:16] <= AHB_INTERFACE_0_hrdata;
                                            4'b0011: dataImage[31:24] <= AHB_INTERFACE_0_hrdata;
                                            4'b0100: dataImage[39:32] <= AHB_INTERFACE_0_hrdata;
                                            4'b0101: dataImage[47:40] <= AHB_INTERFACE_0_hrdata;
                                            4'b0110: dataImage[55:48] <= AHB_INTERFACE_0_hrdata;
                                            4'b0111: dataImage[63:56] <= AHB_INTERFACE_0_hrdata;
                                            4'b1000: dataImage[71:64] <= AHB_INTERFACE_0_hrdata;
                                        endcase
                                
                                        AHB_INTERFACE_0_sel <= 0;
                                        AHB_INTERFACE_0_hprot <= 0;
                                        AHB_INTERFACE_0_hready_in <= 1;
                                
                                        // [0 1 2] => address from element 2 to 3 must jump by 84-2
                                        // [3 4 5] => address from element 5 to 6 must jump by 84-2
                                        // [6 7 8]
                                        if(cntDataImage == 4'b0010 || cntDataImage == 4'b0101) addrImage <= addrImage + 32'd82;
                                        else if(cntDataImage == 4'b0000) begin nextAddrImage <= addrImage + 1; addrImage <= addrImage + 1; end
                                        else if(cntDataImage != 4'b1000) addrImage <= addrImage + 1;
                                        else if(cntDataImage == 4'b1000) begin addrImage <= nextAddrImage; state <= CALCULATE; end
                                        else addrImage <= addrImage;
                                
                                        if(cntDataImage != 4'b1000) cntDataImage <= cntDataImage + 1;
                                        else cntDataImage <= 0;
                                        
                                        read <= READ_REQ;
                                    end
                                    else begin AHB_INTERFACE_0_hready_in <= 0; read <= read; end
                            
                                    AHB_INTERFACE_0_htrans <= 2'b00;                                    
                                end
                        endcase        
                    end
                WARM_FETCH:
                    begin
                        case(warmRead)
                            WARM_UPDATE:
                                begin
                                    dataImage[63:56] <= dataImage[71:64];
                                    dataImage[55:48] <= dataImage[63:56];
                                    dataImage[39:32] <= dataImage[47:40];
                                    dataImage [31:24] <= dataImage[39:32];
                                    dataImage[15:8] <= dataImage[23:16];
                                    dataImage[7:0] <= dataImage[15:8];
                                    
                                    warmRead <= WARM_REQ;
                                end
                            WARM_REQ:   
                                begin
                                    if(AHB_INTERFACE_0_hready_out == 1) // requesting read for image
                                    begin
                                        AHB_INTERFACE_0_htrans <= 2'b10;
                                        AHB_INTERFACE_0_haddr <= addrImage;
                                        AHB_INTERFACE_0_hburst <= 0;
                                        AHB_INTERFACE_0_hsize <= 3'b000;
                                        AHB_INTERFACE_0_hready_in <= 1;
                                        AHB_INTERFACE_0_sel <= 1;
                                        AHB_INTERFACE_0_hprot <= 1;
                                        AHB_INTERFACE_0_hwrite <= 0;
                                        
                                        warmRead <= WARM_READ;
                                    end
                                    else warmRead <= warmRead;
                                end
                            WARM_READ:
                                begin
                                    if(AHB_INTERFACE_0_hready_out == 1) // if request data return
                                    begin
                                        case(cntDataImageWarm)
                                            2'b00: dataImage[23:16] <= AHB_INTERFACE_0_hrdata;
                                            2'b01: dataImage[47:40] <= AHB_INTERFACE_0_hrdata;
                                            2'b10: dataImage[71:64] <= AHB_INTERFACE_0_hrdata;
                                        endcase
                                        
                                        AHB_INTERFACE_0_sel <= 0;
                                        AHB_INTERFACE_0_hprot <= 0;
                                        AHB_INTERFACE_0_hready_in <= 1;
                                        
                                        if(cntDataImageWarm == 2'b00) begin nextAddrImage <= addrImage + 1; addrImage <= addrImage + 32'd84; end    // next image is one coloumn far
                                        else if(cntDataImageWarm == 2'b01) addrImage <= addrImage + 32'd84;
                                        else if(cntDataImageWarm == 2'b10) begin addrImage <= nextAddrImage; state <= CALCULATE; end
                                        else addrImage <= addrImage;
                                        
                                        if(cntDataImageWarm == 2'b10) warmRead <= WARM_REQ;
                                        else warmRead <= WARM_REQ;
                                        
                                        if(cntDataImageWarm != 2'b10) cntDataImageWarm <= cntDataImageWarm + 1;
                                        else cntDataImageWarm <= 0;
                                    end
                                    else begin AHB_INTERFACE_0_hready_in <= 0; warmRead <= warmRead; end
                                    
                                    AHB_INTERFACE_0_htrans <= 2'b00;
                                end
                        endcase         
                    end
                CALCULATE:
                    begin
                        case(calc)
                            NEG:
                                begin
                                    negDataImage <= ~dataImage;
                                    negDataWeight <= ~dataWeight;
                                    calc <= MUL_PREPARE;
                                end
                            MUL_PREPARE:
                                begin
                                    // sign-extended abs
                                    dataTempImage1 <= (negDataImage[7]) ? {4'b0000, dataImage[6:0], 5'b00000} : {4'b0000, negDataImage[6:0], 5'b00000} + 6'b100000;
                                    dataTempWeight1 <= (negDataWeight[7]) ? {9'b000000000, dataWeight[6:0]} : {9'b000000000, negDataWeight[6:0]} + 1;
                                    dataTempImage2 <= (negDataImage[15]) ? {4'b0000, dataImage[14:8], 5'b00000} : {4'b0000, negDataImage[14:8], 5'b00000} + 6'b100000;
                                    dataTempWeight2 <= (negDataWeight[15]) ? {9'b000000000, dataWeight[14:8]} : {9'b000000000, negDataWeight[14:8]} + 1;             
                                    dataTempImage3 <= (negDataImage[23]) ? {4'b0000, dataImage[22:16], 5'b00000} : {4'b0000, negDataImage[22:16], 5'b00000} + 6'b100000;
                                    dataTempWeight3 <= (negDataWeight[23]) ? {9'b000000000, dataWeight[22:16]} : {9'b000000000, negDataWeight[22:16]} + 1;
                                    dataTempImage4 <= (negDataImage[31]) ? {4'b0000, dataImage[30:24], 5'b00000} : {4'b0000, negDataImage[30:24], 5'b00000} + 6'b100000;
                                    dataTempWeight4 <= (negDataWeight[31]) ? {9'b000000000, dataWeight[30:24]} : {9'b000000000, negDataWeight[30:24]} + 1;                                                                                               
                                    dataTempImage5 <= (negDataImage[39]) ? {4'b0000, dataImage[38:32], 5'b00000} : {4'b0000, negDataImage[38:32], 5'b00000} + 6'b100000;
                                    dataTempWeight5 <= (negDataWeight[39]) ? {9'b000000000, dataWeight[38:32]} : {9'b000000000, negDataWeight[38:32]} + 1;
                                    dataTempImage6 <= (negDataImage[47]) ? {4'b0000, dataImage[46:40], 5'b00000} : {4'b0000, negDataImage[46:40], 5'b00000} + 6'b100000;
                                    dataTempWeight6 <= (negDataWeight[47]) ? {9'b000000000, dataWeight[46:40]} : {9'b000000000, negDataWeight[46:40]} + 1;
                                    dataTempImage7 <= (negDataImage[55]) ? {4'b0000, dataImage[54:48], 5'b00000} : {4'b0000, negDataImage[54:48], 5'b00000} + 6'b100000;
                                    dataTempWeight7 <= (negDataWeight[55]) ? {9'b000000000, dataWeight[54:48]} : {9'b000000000, negDataWeight[54:48]} + 1;                 
                                    dataTempImage8 <= (negDataImage[63]) ? {4'b0000, dataImage[62:56], 5'b00000} : {4'b0000, negDataImage[62:56], 5'b00000} + 6'b100000;
                                    dataTempWeight8 <= (negDataWeight[63]) ? {9'b000000000, dataWeight[62:56]} : {9'b000000000, negDataWeight[62:56]} + 1;
                                    dataTempImage9 <= (negDataImage[71]) ? {4'b0000, dataImage[70:64], 5'b00000} : {4'b0000, negDataImage[70:64], 5'b00000} + 6'b100000;
                                    dataTempWeight9 <= (negDataWeight[71]) ? {9'b000000000, dataWeight[70:64]} : {9'b000000000, negDataWeight[70:64]} + 1;                                                                                                                                                               
                                    
                                    // saving sign
                                    signImage <= {dataImage[71], dataImage[63], dataImage[55], dataImage[47], dataImage[39], dataImage[31], dataImage[23], dataImage[15], dataImage[7]};
                                    signWeight <= {dataWeight[71], dataWeight[63], dataWeight[55], dataWeight[47], dataWeight[39], dataWeight[31], dataWeight[23], dataWeight[15], dataWeight[7]};
                                    calc <= MUL;
                                end
                            MUL:
                                begin
                                    // multiplying with sign
                                    dataTempImage1 <= ((signImage[0] ^ signWeight[0]) ? (~(dataTempImage1 * dataTempWeight1) + 1) : (dataTempImage1 * dataTempWeight1)) >>> 12;
                                    dataTempImage2 <= ((signImage[1] ^ signWeight[1]) ? (~(dataTempImage2 * dataTempWeight2) + 1) : (dataTempImage2 * dataTempWeight2)) >>> 12;
                                    dataTempImage3 <= ((signImage[2] ^ signWeight[2]) ? (~(dataTempImage3 * dataTempWeight3) + 1) : (dataTempImage3 * dataTempWeight3)) >>> 12;
                                    dataTempImage4 <= ((signImage[3] ^ signWeight[3]) ? (~(dataTempImage4 * dataTempWeight4) + 1) : (dataTempImage4 * dataTempWeight4)) >>> 12;
                                    dataTempImage5 <= ((signImage[4] ^ signWeight[4]) ? (~(dataTempImage5 * dataTempWeight5) + 1) : (dataTempImage5 * dataTempWeight5)) >>> 12;
                                    dataTempImage6 <= ((signImage[5] ^ signWeight[5]) ? (~(dataTempImage6 * dataTempWeight6) + 1) : (dataTempImage6 * dataTempWeight6)) >>> 12;
                                    dataTempImage7 <= ((signImage[6] ^ signWeight[6]) ? (~(dataTempImage7 * dataTempWeight7) + 1) : (dataTempImage7 * dataTempWeight7)) >>> 12;
                                    dataTempImage8 <= ((signImage[7] ^ signWeight[7]) ? (~(dataTempImage8 * dataTempWeight8) + 1) : (dataTempImage8 * dataTempWeight8)) >>> 12;
                                    dataTempImage9 <= ((signImage[8] ^ signWeight[8]) ? (~(dataTempImage9 * dataTempWeight9) + 1) : (dataTempImage9 * dataTempWeight9)) >>> 12;                                    
                                    calc <= ADD;
                                end
                            ADD:
                                begin
                                    // fixed-point calculation only requries simple addition
                                    dataTempImage1 <= dataTempImage1 + dataTempImage2 + dataTempImage3 + dataTempImage4 + dataTempImage5 + dataTempImage6 + dataTempImage7 + dataTempImage8 + dataTempImage9;
                                    calc <= RESULT;
                                end
                            RESULT:
                                begin
                                    if(dataTempImage1[15])  // if data is negative
                                        dataTemp <= 8'b0000_0000;   // ReLU
                                    else if(dataTempImage1[14:7] > 8'b0001_1111)    // overflow
                                        dataTemp <= 8'b0001_1111;
                                    else if((dataTempImage1[14:7] == 8'b0000_0000) && (7'b001_1111 >= dataTempImage1[6:0]))  // underflow
                                        dataTemp <= 8'b0000_0000;
                                    else dataTemp <= {dataTempImage1[15], dataTempImage1[6:0]};
                                    calc <= LOAD;
                                end
                            LOAD:
                                begin
                                    ena <= 1;
                                    wea <= 1;
                                    addrTemp <= addrTemp + 1;
                                    calc <= WRAP_UP;
                                end
                            WRAP_UP:
                                begin
                                    ena <= 0;
                                    wea <= 0;
                                    
                                    cntConv <= cntConv + 1;
                                    
                                    if(cntConv == 81) state <= COLD_FETCH;
                                    else if((cntConv+1) % 82 == 0) state <= POOL;
                                    else state <= WARM_FETCH;
                                    calc <= NEG;
                                end
                        endcase
                    end
             POOL:
                begin
               case(poolState)
                     POOL_WAIT:
                         begin
                             poolState <= POOL_REQ;
                         end
                     POOL_REQ:
                         begin
                             enb <= 1;
                             poolState <= POOL_READ;
                         end
                     POOL_READ:
                         begin
                             enb <= 0;
                             cntDataPool <= cntDataPool + 1;

                             case(cntDataPool % 4)
                                 2'b00: begin dataOut <= dataPool; addrPool <= addrPool + 1; nextAddrPool <= addrPool + 2; end
                                 2'b01: begin if(dataPool > dataOut) dataOut <= dataPool; else dataOut <= dataOut; addrPool <= addrPool + 12'd81; end    // 81 = 82 - 1
                                 2'b10: begin if(dataPool > dataOut) dataOut <= dataPool; else dataOut <= dataOut; addrPool <= addrPool + 1; end
                                 2'b11: begin if(dataPool > dataOut) dataOut <= dataPool; else dataOut <= dataOut; if((cntDataPool+1)%41 == 0) addrPool <= nextAddrPool + 82; else addrPool <= nextAddrPool; end
                             endcase
                             
                             if(cntDataPool % 4 == 2'b11) poolState <= POOL_CALC;
                             else poolState <= POOL_REQ;
                         end
                     POOL_CALC:
                         begin
                             if(AHB_INTERFACE_0_hready_out == 1 && AHB_INTERFACE_0_hresp == 0)
                             begin
                                 AHB_INTERFACE_0_htrans <= 2'b10;
                                 AHB_INTERFACE_0_haddr <= addrOut;
                                 AHB_INTERFACE_0_hburst <= 0;
                                 AHB_INTERFACE_0_hsize <= 3'b000;
                                 AHB_INTERFACE_0_hready_in <= 1;
                                 AHB_INTERFACE_0_sel <= 1;
                                 AHB_INTERFACE_0_hprot <= 0;
                                 AHB_INTERFACE_0_hwdata <= dataOut;
                                 AHB_INTERFACE_0_hwrite <= 1;
                                 
                                 addrOut <= addrOut + 1;
                                 cntTotal <= cntTotal + 1;
                                 
                                 poolState <= POOL_DONE;
                             end
                             else poolState <= POOL_CALC;
                         end
                     POOL_DONE:
                         begin
                             AHB_INTERFACE_0_htrans <= 0;
                             AHB_INTERFACE_0_hready_in <= 0;
                             poolState <= POOL_WRAP_UP;
                         end
                     POOL_WRAP_UP:
                         begin
                             if(AHB_INTERFACE_0_hready_out == 0 && AHB_INTERFACE_0_hresp == 0)
                             begin
                                 AHB_INTERFACE_0_sel <= 0;
                                 AHB_INTERFACE_0_hprot <= 0;
                                 AHB_INTERFACE_0_hwdata <= 0;
                                 AHB_INTERFACE_0_hwrite <= 0;
                                 AHB_INTERFACE_0_hready_in <= 1;
                                 poolState <= POOL_WRAP_UP;
                             end
                             else if(cntTotal == 1681)   // 6724 = 82 * 82, 1681 = 6724 / 4
                             begin
                                 finish <= 1;
                                 poolState <= 3'b101;
                             end
                             else
                             begin
                                 // 41 elements in one row
                                 if(!((cntTotal+1)%41)) state <= COLD_FETCH;
                                 poolState <= POOL_WAIT;
                             end
                         end
                     default:
                         begin
                             AHB_INTERFACE_0_haddr <= 0;
                             AHB_INTERFACE_0_hburst <= 0;
                             AHB_INTERFACE_0_hprot <= 0;
                             AHB_INTERFACE_0_hready_in <= 0;
                             AHB_INTERFACE_0_hsize <= 0;
                             AHB_INTERFACE_0_htrans <= 0;
                             AHB_INTERFACE_0_hwdata <= 0;
                             AHB_INTERFACE_0_hwrite <= 0;
                             AHB_INTERFACE_0_sel <= 0;
                             
                             poolState <= poolState;
                         end
                 endcase                    
                end
            endcase    
        end                            
    end
    

endmodule
  