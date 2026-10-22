// One combinational execution unit. All arithmetic wraps to 32 bits.
module alu (
    input wire [31:0] instruction, pc, immediate, lhs, rhs,
    output reg [31:0] value,
    output reg [31:0] next_pc,
    output reg taken
);
    wire [6:0] opcode = instruction[6:0];
    wire [2:0] funct3 = instruction[14:12];
    wire [31:0] operand2 = opcode == 7'h13 ? immediate : rhs;
    // Extend before multiplying, so MULH/MULHSU/MULHU share one multiplier.
    wire signed [32:0] multiply_lhs =
        {((funct3 != 3) && lhs[31]), lhs};
    wire signed [32:0] multiply_rhs =
        {((funct3 == 1) && rhs[31]), rhs};
    wire signed [65:0] product = multiply_lhs * multiply_rhs;
    always @* begin
        value = 0;
        next_pc = pc + 4;
        taken = 0;
        case (opcode)
            7'h37: value = immediate;
            7'h17: value = pc + immediate;
            7'h6f: begin value = pc + 4; next_pc = pc + immediate; end
            7'h67: begin value = pc + 4; next_pc = (lhs + immediate) & 32'hfffffffe; end
            7'h63: begin
                case (funct3)
                    0: taken = lhs == rhs;
                    1: taken = lhs != rhs;
                    4: taken = $signed(lhs) < $signed(rhs);
                    5: taken = $signed(lhs) >= $signed(rhs);
                    6: taken = lhs < rhs;
                    7: taken = lhs >= rhs;
                    default: taken = 0;
                endcase
                if (taken) next_pc = pc + immediate;
            end
            7'h13, 7'h33: begin
                if (opcode == 7'h33 && instruction[31:25] == 1) begin
                    case (funct3)
                        0: value = product[31:0];
                        1, 2, 3: value = product[63:32];
                        4: begin // DIV: zero divisor and signed overflow are defined by RV32M
                            if (rhs == 0) value = 32'hffffffff;
                            else if (lhs == 32'h80000000 && rhs == 32'hffffffff)
                                value = 32'h80000000;
                            else value = $signed(lhs) / $signed(rhs);
                        end
                        5: value = rhs == 0 ? 32'hffffffff : lhs / rhs;
                        6: begin
                            if (rhs == 0) value = lhs;
                            else if (lhs == 32'h80000000 && rhs == 32'hffffffff) value = 0;
                            else value = $signed(lhs) % $signed(rhs);
                        end
                        7: value = rhs == 0 ? lhs : lhs % rhs;
                    endcase
                end else begin
                    case (funct3)
                        0: if (opcode == 7'h33 && instruction[30]) value = lhs - rhs;
                           else value = lhs + operand2;
                        1: value = lhs << operand2[4:0];
                        2: value = {31'b0, $signed(lhs) < $signed(operand2)};
                        3: value = {31'b0, lhs < operand2};
                        4: value = lhs ^ operand2;
                        5: if (instruction[30]) value = $signed(lhs) >>> operand2[4:0];
                           else value = lhs >> operand2[4:0];
                        6: value = lhs | operand2;
                        7: value = lhs & operand2;
                    endcase
                end
            end
            default: value = 0;
        endcase
    end
endmodule
