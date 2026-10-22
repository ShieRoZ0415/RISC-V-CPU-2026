// RV32IM decoder. Immediate bit positions follow the C++ reference decoder.
module decoder (
    input wire [31:0] instruction,
    output reg valid,
    output reg uses_rs1,
    output reg uses_rs2,
    output reg writes_rd,
    output reg is_memory,
    output reg is_store,
    output reg is_control,
    output reg [31:0] immediate
);
    wire [6:0] opcode = instruction[6:0];
    wire [2:0] funct3 = instruction[14:12];
    wire [6:0] funct7 = instruction[31:25];
    always @* begin
        valid = 1;
        uses_rs1 = 0;
        uses_rs2 = 0;
        writes_rd = 0;
        is_memory = 0;
        is_store = 0;
        is_control = 0;
        immediate = 0;
        case (opcode)
            7'h37, 7'h17: begin // LUI, AUIPC
                writes_rd = 1;
                immediate = {instruction[31:12], 12'b0};
            end
            7'h6f: begin // JAL
                writes_rd = 1;
                is_control = 1;
                immediate = {{11{instruction[31]}}, instruction[31],
                    instruction[19:12], instruction[20], instruction[30:21], 1'b0};
            end
            7'h67: begin // JALR
                uses_rs1 = 1;
                writes_rd = 1;
                is_control = 1;
                immediate = {{20{instruction[31]}}, instruction[31:20]};
                valid = funct3 == 0;
            end
            7'h63: begin // branches
                uses_rs1 = 1;
                uses_rs2 = 1;
                is_control = 1;
                immediate = {{19{instruction[31]}}, instruction[31], instruction[7],
                    instruction[30:25], instruction[11:8], 1'b0};
                valid = funct3 != 2 && funct3 != 3;
            end
            7'h03: begin // loads
                uses_rs1 = 1;
                writes_rd = 1;
                is_memory = 1;
                immediate = {{20{instruction[31]}}, instruction[31:20]};
                valid = funct3 == 0 || funct3 == 1 || funct3 == 2 ||
                        funct3 == 4 || funct3 == 5;
            end
            7'h23: begin // stores
                uses_rs1 = 1;
                uses_rs2 = 1;
                is_memory = 1;
                is_store = 1;
                immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
                valid = funct3 <= 2;
            end
            7'h13: begin // immediate ALU
                uses_rs1 = 1;
                writes_rd = 1;
                immediate = {{20{instruction[31]}}, instruction[31:20]};
                if (funct3 == 1) valid = funct7 == 0;
                if (funct3 == 5) valid = funct7 == 0 || funct7 == 7'h20;
            end
            7'h33: begin // register ALU and M extension
                uses_rs1 = 1;
                uses_rs2 = 1;
                writes_rd = 1;
                valid = funct7 == 0 || funct7 == 1 ||
                    (funct7 == 7'h20 && (funct3 == 0 || funct3 == 5));
            end
            // Not required by the course; treating fences as no-ops is convenient.
            7'h0f: valid = funct3 == 0 || funct3 == 1;
            default: valid = 0;
        endcase
    end
endmodule
