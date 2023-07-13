module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );
    //... write your code here ...

    // ProgramCounter
    wire[31:0] w_PCn;

    // Register File 32x32
    wire [4:0] w_A1, w_A2, w_A3;
    wire [31:0] w_WD3;
    wire [31:0] w_rs1, w_rs2;

    // Arithmetic Logic Unit
    wire [31:0] w_ALUinB;
    wire [31:0] w_ALUResult;
    wire w_Zero;

    wire [31:0] w_instr;

    // Control Unit
    wire w_AuiPC, w_LuiImm, w_BranchBlt, w_BranchBeq, w_BranchJal, w_BranchJalr, w_RegWrite, w_MemToReg, w_MemWrite, w_ALUSrc;
    wire [3:0] w_ALUControl;
    wire [2:0] w_immControl;
    
    // immDecode
    wire [31:0] w_immOut;
    wire [31:0] w_immPC, w_PCAdded;
    wire [31:0] w_BranchTarget, w_Res, w_Res2, w_AfterJalX;
    wire w_BranchJalX, w_BranchOut, w_Beq, w_Blt;

    assign w_instr = instruction;
    assign w_A1 = w_instr[19:15];
    assign w_A2 = w_instr[24:20];
    assign w_A3 = w_instr[11:7];

    assign w_immPC = ( w_immOut + PC );
    assign w_PCAdded = ( PC + 4 );
    assign w_BranchJalX = ( w_BranchJal | w_BranchJalr );
    assign w_Beq = ( w_Zero & w_BranchBeq );
    assign w_Blt = ( w_ALUResult & w_BranchBlt );
    assign w_BranchOut = ( w_Beq | w_Blt | w_BranchJalX ); 

    // Modules
    arithmeticLogicUnit alu( w_rs1, w_ALUinB, w_ALUControl, w_Zero, w_ALUResult);
    reg32x32 registerFile( clk, w_RegWrite, w_A1, w_A2, w_A3, w_WD3, w_rs1, w_rs2 );
    controlUnit CU( w_instr, w_AuiPC, w_LuiImm, w_BranchBlt, w_BranchBeq, w_BranchJal, w_BranchJalr, w_RegWrite, w_MemToReg, w_MemWrite, w_ALUSrc, w_ALUControl, w_immControl );
    immDecode immDec( w_instr, w_immControl, w_immOut );
    progCounter progCounter( w_PCn, clk, reset, PC );
 
    // Muxes         0             1             select        result
    mux aluInMux( w_rs2,       w_immOut,       w_ALUSrc,     w_ALUinB );
    mux BrTarMux( w_immPC,     w_ALUResult,    w_BranchJalr, w_BranchTarget);
    mux AuiMux(   w_Res,       w_immPC,        w_AuiPC,      w_Res2);
    mux LuiMux(   w_Res2,      w_immOut,       w_LuiImm,     w_WD3);
    mux JalXMux(  w_ALUResult, w_PCAdded,      w_BranchJalX, w_AfterJalX);
    mux ResMux(   w_AfterJalX, data_from_mem,  w_MemToReg,   w_Res);
    mux PCMux(    w_PCAdded,   w_BranchTarget, w_BranchOut,  w_PCn);
    // mux name( .a(), .b(), .select(), .y());

    // Outputs
    assign address_to_mem = w_ALUResult;
    assign data_to_mem = w_rs2;
    assign WE = w_MemWrite;

endmodule

//My modules
module progCounter(input [31:0] in,
                   input clk, reset,
                   output reg [31:0] PC);
    always@(posedge clk)
        if ( reset == 1)
            PC <= 0;
        else
            PC <= in;
endmodule

// Arithmetic Logic Unit
module arithmeticLogicUnit(input signed [31:0] SrcA, SrcB,
                           input [3:0] ALUCntrl, 
                           output reg Zero, 
                           output reg [31:0] ALUResult );
    always@(*)
        begin
        Zero = 0;
        case(ALUCntrl)
            'b0000 : ALUResult = SrcA & SrcB;
            'b0001 : ALUResult = SrcA | SrcB;
            'b0010 : ALUResult = SrcA + SrcB;
            'b0011 : ALUResult = SrcA ^ SrcB;
            'b0110 : ALUResult = SrcA - SrcB;
            'b0111 : ALUResult = $signed(SrcA) < $signed(SrcB);
            'b1000 : ALUResult = $signed(SrcA) / $signed(SrcB);
            'b1001 : ALUResult = $signed(SrcA) % $signed(SrcB);
            'b1010 : ALUResult = SrcA >>> SrcB;
            'b1011 : ALUResult = SrcA >> SrcB; 
            'b1100 : ALUResult = SrcA << SrcB;
        endcase
        if ( ALUResult == 0 ) 
            Zero = 1;
    end
endmodule

// Multiplexor
module mux(input [31:0] a, b, input select,
          output reg [31:0] y);
        always@(*)
        if(select==0) 
            y = a;
        else
            y = b;
endmodule

// Registers
module reg32x32(input CLK, WE3, 
                input [4:0] A1, A2, A3, 
                input [31:0] WD3,
                output reg [31:0] RD1, RD2 );

    reg [31:0] rf[31:0];

    // Write 
    always@( posedge CLK ) begin
        if ( WE3 == 1 && A3 != 'b0) 
            rf[A3] = WD3;
    end

    // Read
    always@(*) begin
        if ( A1 == 0 )
            RD1 = 0;
        else
            RD1 = rf[A1];
        if ( A2 == 0 )
            RD2 = 0;
        else
            RD2 = rf[A2];
    end
endmodule

// Control Unit
module controlUnit(input [31:0] instr,
                   output reg AuiPC, LuiImm, BranchBlt, BranchBeq, BranchJal, BranchJalr, RegWrite, MemToReg, MemWrite, ALUSrc,
                   output reg [3:0] ALUControl,
                   output reg [2:0] immControl);

    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;

    assign opcode = instr[6:0];
    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];

    always@(*)
        begin
            case (opcode)
                'b0000011 : begin // I-Type - Lw
                    AuiPC = 0;
                    LuiImm = 0;
                    BranchBlt = 0;
                    BranchJal = 0;
                    BranchJalr = 0;
                    BranchBeq = 0;
                    RegWrite = 1;
                    MemToReg = 1;
                    MemWrite = 0;
                    ALUSrc = 1;
                    immControl = 0; // I
                    ALUControl = 'b0010;
                    end 
                'b0010011 : begin // I-Type - Addi
                    AuiPC = 0;
                    LuiImm = 0;
                    BranchBlt = 0;
                    BranchJal = 0;
                    BranchJalr = 0;
                    BranchBeq = 0;
                    RegWrite = 1;
                    MemToReg = 0;
                    MemWrite = 0;
                    ALUSrc = 1;
                    immControl = 0; // I
                    ALUControl = 'b0010;
                    end
                'b0100011 : begin // S-Type - Sw
                    AuiPC = 0;
                    LuiImm = 0;
                    BranchBlt = 0;
                    BranchJal = 0;
                    BranchJalr = 0;
                    BranchBeq = 0;
                    RegWrite = 0;
                    MemToReg = 0;
                    MemWrite = 1;
                    ALUSrc = 1;
                    immControl = 1; // S
                    ALUControl = 'b0010;
                    end
                'b0110011 : begin // R-Type - Add, sub, and, slt, div, rem, 
                    AuiPC = 0;
                    LuiImm = 0;
                    BranchBlt = 0;
                    BranchJal = 0;
                    BranchJalr = 0;
                    BranchBeq = 0;
                    RegWrite = 1;
                    MemToReg = 0;
                    MemWrite = 0;
                    ALUSrc = 0;
                    immControl = 7;
                    case(funct7)
                        'b0000000 : begin // Add, slt, or, and
                            case(funct3)
                                'b000 : ALUControl = 'b0010; // Add
                                'b010 : ALUControl = 'b0111; // Slt
                                'b111 : ALUControl = 'b0000; // And
                                'b001 : ALUControl = 'b1100; // Sll
                                'b101 : ALUControl = 'b1011; // Srl
                            endcase
                        end
                        'b0100000 : begin // Sub
                            case(funct3)
                                'b000 : ALUControl = 'b0110; // Sub
                                'b101 : ALUControl = 'b1010; // Sra
                            endcase
                        end
                        'b0000001 : begin //Div, Rem
                            case(funct3)
                                'b100: ALUControl = 'b1000; // Div
                                'b110: ALUControl = 'b1001; // Rem
                            endcase
                        end
                    endcase
                    end 
                'b0110111 : begin // U-Type - Lui  
                    AuiPC = 0;
                    LuiImm = 1;
                    BranchBlt = 0;
                    BranchJal = 0;
                    BranchJalr = 0;
                    BranchBeq = 0;
                    RegWrite = 1;
                    MemToReg = 0;
                    MemWrite = 0;
                    ALUSrc = 0;
                    immControl = 3; // U - type
                    ALUControl = 'b1111;
                    end 
                'b1100111 : begin // I-Type - Jalr
                    AuiPC = 0;
                    LuiImm = 0;
                    BranchBlt = 0;
                    BranchJal = 0;
                    BranchJalr = 1;
                    BranchBeq = 0;
                    RegWrite = 1;
                    MemToReg = 0;
                    MemWrite = 0;
                    ALUSrc = 1;
                    immControl = 0; // I-Type
                    ALUControl = 'b0010;
                    end
                'b1101111 : begin // J-Type - Jal 
                    AuiPC = 0;
                    LuiImm = 0;
                    BranchBlt = 0;
                    BranchJal = 1;
                    BranchJalr = 0;
                    BranchBeq = 0;
                    RegWrite = 1;
                    MemToReg = 0;
                    MemWrite = 0;
                    ALUSrc = 0;
                    immControl = 4; // J-Type
                    ALUControl = 'b1111;
                    end
                'b1100011 : begin // B-Type - Beq, Blt
                    AuiPC = 0;
                    LuiImm = 0;
                    BranchJal = 0;
                    BranchJalr = 0;
                    RegWrite = 0;
                    MemToReg = 0;
                    MemWrite = 0;
                    ALUSrc = 0;
                    immControl = 2; // B-Type
                    case(funct3)
                        'b000 : begin // Beq
                            BranchBlt = 0;
                            BranchBeq = 1;
                            ALUControl = 'b0110;
                        end
                        'b100 : begin // Blt
                            BranchBlt = 1;
                            BranchBeq = 0;
                            ALUControl = 'b0111;
                        end
                    endcase
                    end
                'b0010111 : begin // AuiPC
                    AuiPC = 1;
                    LuiImm = 0;
                    BranchBlt = 0;
                    BranchJal = 0;
                    BranchJalr = 0;
                    BranchBeq = 0;
                    RegWrite = 1;
                    MemToReg = 0;
                    MemWrite = 0;
                    ALUSrc = 0;
                    immControl = 3; // U - type
                    ALUControl = 'b1111;
                    end
            endcase
        end
endmodule

// ImmDecode
module immDecode (input [31:0] immInput,
                  input [2:0] immControl,
                  output reg [31:0] immOutput);
    reg zero;

    always@(*)
        begin
            zero = 0;
            case(immControl)
                'b000: {immOutput[31:11], immOutput[10:5], immOutput[4:1], immOutput[0]} <= {{21{immInput[31]}} ,immInput[30:25], immInput[24:21],immInput[20]}; // I
                'b001: {immOutput[31:11], immOutput[10:5], immOutput[4:1], immOutput[0]} <= {{21{immInput[31]}} ,immInput[30:25], immInput[11:8],immInput[7]};  // S
                'b010: {immOutput[31:12], immOutput[11], immOutput[10:5], immOutput[4:1], immOutput[0]} <= {{20{immInput[31]}}, immInput[7], immInput[30:25], immInput[11:8], zero}; // B
                'b011: {immOutput[31], immOutput[30:20], immOutput[19:12], immOutput[11:0]} <= { immInput[31], immInput[30:20], immInput[19:12], {12{zero}} };// U
                'b100: {immOutput[31:20], immOutput[19:12],immOutput[11], immOutput[10:5], immOutput[4:1], immOutput[0]} <= {{13{immInput[31]}}, immInput[19:12], immInput[20], immInput[30:25], immInput[24:21], zero}; // J
            endcase
        end
endmodule  