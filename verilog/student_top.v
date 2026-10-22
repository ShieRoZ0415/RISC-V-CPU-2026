// Small Tomasulo CPU: one dispatch, one ALU, one in-order memory port,
// and one retirement per cycle. ROB tags also name speculative values.
module student_top #(
    parameter ROB_DEPTH = 16,
    parameter RS_DEPTH = 12,
    parameter LSQ_DEPTH = 12,
    parameter ICACHE_WORDS = 1024,
    parameter DCACHE_WORDS = 256,
    parameter PREDICTOR_SIZE = 256
)(
    input wire clock, reset,
    output wire [31:0] araddr,
    output wire arvalid,
    input wire arready,
    input wire [31:0] rdata,
    input wire [1:0] rresp,
    input wire rvalid,
    output wire rready,
    output wire [31:0] awaddr,
    output wire awvalid,
    input wire awready,
    output wire [31:0] wdata,
    output wire [3:0] wstrb,
    output wire wvalid,
    input wire wready,
    input wire [1:0] bresp,
    input wire bvalid,
    output wire bready
);
    localparam IC_BITS = $clog2(ICACHE_WORDS);
    localparam DC_BITS = $clog2(DCACHE_WORDS);
    localparam BP_BITS = $clog2(PREDICTOR_SIZE);
    localparam BUS_IDLE = 0, READ_ADDRESS = 1, READ_DATA = 2,
               WRITE_CHANNELS = 3, WRITE_RESPONSE = 4;

    reg [31:0] pc;
    reg fault; // Stop issuing after a data bus error; no exception system required.
    reg [31:0] registers [0:31];
    reg rat_busy [0:31];
    integer rat_tag [0:31];

    integer rob_head, rob_tail, rob_count;
    reg rob_busy [0:ROB_DEPTH-1];
    reg rob_ready [0:ROB_DEPTH-1];
    reg [4:0] rob_rd [0:ROB_DEPTH-1];
    reg [31:0] rob_value [0:ROB_DEPTH-1];
    reg rob_control [0:ROB_DEPTH-1];
    reg [31:0] rob_pc [0:ROB_DEPTH-1];
    reg [31:0] rob_prediction [0:ROB_DEPTH-1];
    reg [31:0] rob_next_pc [0:ROB_DEPTH-1];
    reg rob_branch [0:ROB_DEPTH-1];
    reg rob_taken [0:ROB_DEPTH-1];
    reg rob_memory [0:ROB_DEPTH-1];
    reg rob_store [0:ROB_DEPTH-1];
    reg rob_address_ready [0:ROB_DEPTH-1];
    reg [31:0] rob_address [0:ROB_DEPTH-1];
    reg [31:0] rob_store_data [0:ROB_DEPTH-1];
    reg [2:0] rob_funct3 [0:ROB_DEPTH-1];

    reg rs_busy [0:RS_DEPTH-1];
    integer rs_rob [0:RS_DEPTH-1];
    reg [31:0] rs_instruction [0:RS_DEPTH-1];
    reg [31:0] rs_pc [0:RS_DEPTH-1];
    reg [31:0] rs_imm [0:RS_DEPTH-1];
    reg rs_ready1 [0:RS_DEPTH-1], rs_ready2 [0:RS_DEPTH-1];
    reg [31:0] rs_value1 [0:RS_DEPTH-1], rs_value2 [0:RS_DEPTH-1];
    integer rs_tag1 [0:RS_DEPTH-1], rs_tag2 [0:RS_DEPTH-1];

    reg lsq_busy [0:LSQ_DEPTH-1];
    integer lsq_rob [0:LSQ_DEPTH-1];
    reg [31:0] lsq_imm [0:LSQ_DEPTH-1];
    reg lsq_ready1 [0:LSQ_DEPTH-1], lsq_ready2 [0:LSQ_DEPTH-1];
    reg [31:0] lsq_value1 [0:LSQ_DEPTH-1], lsq_value2 [0:LSQ_DEPTH-1];
    integer lsq_tag1 [0:LSQ_DEPTH-1], lsq_tag2 [0:LSQ_DEPTH-1];

    reg [ICACHE_WORDS-1:0] ic_valid;
    reg [31:0] ic_address [0:ICACHE_WORDS-1], ic_data [0:ICACHE_WORDS-1];
    reg [DCACHE_WORDS-1:0] dc_valid;
    reg [31:0] dc_address [0:DCACHE_WORDS-1], dc_data [0:DCACHE_WORDS-1];
    reg [1:0] predictor [0:PREDICTOR_SIZE-1];
    wire [IC_BITS-1:0] ic_index = pc[IC_BITS+1:2];
    wire ic_hit = ic_valid[ic_index] && ic_address[ic_index] == pc;
    wire [31:0] instruction = ic_data[ic_index];
    wire [4:0] source1 = instruction[19:15], source2 = instruction[24:20];
    wire [4:0] destination = instruction[11:7];
    wire dec_valid, uses_rs1, uses_rs2, writes_rd, dec_memory, dec_store, dec_control;
    wire [31:0] immediate;
    decoder decode(instruction, dec_valid, uses_rs1, uses_rs2, writes_rd,
                   dec_memory, dec_store, dec_control, immediate);

    // The reference's Operand {ready, value, tag}, expressed as plain arrays.
    reg operand_ready1, operand_ready2;
    reg [31:0] operand_value1, operand_value2;
    integer operand_tag1, operand_tag2;
    always @* begin
        operand_ready1 = 1;
        operand_value1 = 0;
        operand_tag1 = 0;
        if (uses_rs1 && source1 != 0) begin
            if (!rat_busy[source1]) operand_value1 = registers[source1];
            else begin
                operand_tag1 = rat_tag[source1];
                operand_ready1 = rob_busy[operand_tag1] && rob_ready[operand_tag1];
                operand_value1 = rob_value[operand_tag1];
            end
        end
        operand_ready2 = 1;
        operand_value2 = 0;
        operand_tag2 = 0;
        if (uses_rs2 && source2 != 0) begin
            if (!rat_busy[source2]) operand_value2 = registers[source2];
            else begin
                operand_tag2 = rat_tag[source2];
                operand_ready2 = rob_busy[operand_tag2] && rob_ready[operand_tag2];
                operand_value2 = rob_value[operand_tag2];
            end
        end
    end

    // Age in the circular ROB replaces the C++ simulator's unbounded sequence.
    function integer age;
        input integer tag;
        begin
            if (tag >= rob_head) age = tag - rob_head;
            else age = tag + ROB_DEPTH - rob_head;
        end
    endfunction
    integer free_rs, execute_rs, free_lsq, execute_lsq;
    always @* begin
        free_rs = -1;
        execute_rs = -1;
        for (integer n = 0; n < RS_DEPTH; n = n + 1) begin
            if (!rs_busy[n] && free_rs == -1) free_rs = n;
            if (rs_busy[n] && rs_ready1[n] && rs_ready2[n]) begin
                if (execute_rs == -1) execute_rs = n;
                else if (age(rs_rob[n]) < age(rs_rob[execute_rs])) execute_rs = n;
            end
        end
        free_lsq = -1;
        execute_lsq = -1;
        for (integer n = 0; n < LSQ_DEPTH; n = n + 1) begin
            if (!lsq_busy[n] && free_lsq == -1) free_lsq = n;
            if (lsq_busy[n] && lsq_ready1[n] && lsq_ready2[n]) begin
                if (execute_lsq == -1) execute_lsq = n;
                else if (age(lsq_rob[n]) < age(lsq_rob[execute_lsq])) execute_lsq = n;
            end
        end
    end
    // Use entry zero as a harmless input when the unit is idle.
    wire [31:0] alu_value, alu_next_pc;
    wire alu_taken;
    wire [31:0] alu_instruction = rs_instruction[execute_rs < 0 ? 0 : execute_rs];
    wire [31:0] alu_pc = rs_pc[execute_rs < 0 ? 0 : execute_rs];
    wire [31:0] alu_imm = rs_imm[execute_rs < 0 ? 0 : execute_rs];
    wire [31:0] alu_lhs = rs_value1[execute_rs < 0 ? 0 : execute_rs];
    wire [31:0] alu_rhs = rs_value2[execute_rs < 0 ? 0 : execute_rs];
    alu execute(alu_instruction, alu_pc, alu_imm, alu_lhs, alu_rhs, alu_value, alu_next_pc, alu_taken);

    reg [31:0] predicted_pc;
    always @* begin
        predicted_pc = pc + 4;
        case (instruction[6:0])
            7'h63: if (predictor[pc[BP_BITS+1:2]][1]) predicted_pc = pc + immediate;
            7'h6f: predicted_pc = pc + immediate;
            7'h67: if (operand_ready1) predicted_pc = (operand_value1 + immediate) & 32'hfffffffe;
            default: predicted_pc = pc + 4;
        endcase
    end

    wire retire = rob_count != 0 && rob_ready[rob_head] && !fault;
    wire recover = retire && rob_control[rob_head] &&
                   rob_next_pc[rob_head] != rob_prediction[rob_head];
    wire dispatch = !reset && !fault && !recover && ic_hit && dec_valid &&
                    rob_count < ROB_DEPTH && (dec_memory ? free_lsq >= 0 : free_rs >= 0);

    // One external transaction at a time. Payloads live in registers until handshake.
    reg [2:0] bus_state;
    reg bus_load;
    reg [31:0] bus_address, bus_write_data;
    reg [3:0] bus_strobe;
    reg aw_sent, w_sent;
    wire head_memory = rob_count != 0 && rob_memory[rob_head] &&
                       rob_address_ready[rob_head] && !rob_ready[rob_head];
    wire [31:0] head_address = rob_address[rob_head];
    wire [DC_BITS-1:0] head_dc_index = head_address[DC_BITS+1:2];
    wire dc_hit = dc_valid[head_dc_index] &&
                  dc_address[head_dc_index] == {head_address[31:2], 2'b0};
    wire [DC_BITS-1:0] bus_dc_index = bus_address[DC_BITS+1:2];
    wire [IC_BITS-1:0] bus_ic_index = bus_address[IC_BITS+1:2];
    wire [31:0] load_word = dc_hit ? dc_data[head_dc_index] : rdata;
    wire [31:0] shifted_load = load_word >> {head_address[1:0], 3'b0};
    reg [31:0] load_value;
    always @* begin
        case (rob_funct3[rob_head])
            0: load_value = {{24{shifted_load[7]}}, shifted_load[7:0]};
            1: load_value = {{16{shifted_load[15]}}, shifted_load[15:0]};
            4: load_value = {24'b0, shifted_load[7:0]};
            5: load_value = {16'b0, shifted_load[15:0]};
            default: load_value = shifted_load;
        endcase
    end
    assign araddr = bus_address;
    assign arvalid = !reset && bus_state == READ_ADDRESS;
    assign rready = !reset && bus_state == READ_DATA;
    assign awaddr = bus_address;
    assign awvalid = !reset && bus_state == WRITE_CHANNELS && !aw_sent;
    assign wdata = bus_write_data;
    assign wstrb = bus_strobe;
    assign wvalid = !reset && bus_state == WRITE_CHANNELS && !w_sent;
    assign bready = !reset && bus_state == WRITE_RESPONSE;

    // Independent two-bit saturating counters, initially weakly not taken.
    generate
        for (genvar p = 0; p < PREDICTOR_SIZE; p = p + 1) begin: branch_predictor
            always @(posedge clock) begin
                if (reset) predictor[p] <= 1;
                else if (retire && rob_branch[rob_head] && rob_pc[rob_head][BP_BITS+1:2] == p) begin
                    if (rob_taken[rob_head]) begin
                        if (predictor[p] != 3) predictor[p] <= predictor[p] + 1'b1;
                    end else if (predictor[p] != 0) predictor[p] <= predictor[p] - 1'b1;
                end
            end
        end
    endgenerate

    always @(posedge clock) begin
        if (reset) begin
            pc <= 0;
            fault <= 0;
            rob_head <= 0;
            rob_tail <= 0;
            rob_count <= 0;
            bus_state <= BUS_IDLE;
            bus_load <= 0;
            bus_address <= 0;
            bus_write_data <= 0;
            bus_strobe <= 0;
            aw_sent <= 0;
            w_sent <= 0;
            for (integer n = 0; n < 32; n = n + 1) begin
                registers[n] <= 0;
                rat_busy[n] <= 0;
                rat_tag[n] <= 0;
            end
            for (integer n = 0; n < ROB_DEPTH; n = n + 1) begin
                rob_busy[n] <= 0;
                rob_ready[n] <= 0;
                rob_rd[n] <= 0;
                rob_value[n] <= 0;
                rob_control[n] <= 0;
                rob_pc[n] <= 0;
                rob_prediction[n] <= 0;
                rob_next_pc[n] <= 0;
                rob_branch[n] <= 0;
                rob_taken[n] <= 0;
                rob_memory[n] <= 0;
                rob_store[n] <= 0;
                rob_address_ready[n] <= 0;
                rob_address[n] <= 0;
                rob_store_data[n] <= 0;
                rob_funct3[n] <= 0;
            end
            for (integer n = 0; n < RS_DEPTH; n = n + 1) begin
                rs_busy[n] <= 0;
                rs_rob[n] <= 0;
                rs_instruction[n] <= 0;
                rs_pc[n] <= 0;
                rs_imm[n] <= 0;
                rs_ready1[n] <= 0; rs_ready2[n] <= 0;
                rs_value1[n] <= 0; rs_value2[n] <= 0;
                rs_tag1[n] <= 0; rs_tag2[n] <= 0;
            end
            for (integer n = 0; n < LSQ_DEPTH; n = n + 1) begin
                lsq_busy[n] <= 0;
                lsq_rob[n] <= 0;
                lsq_imm[n] <= 0;
                lsq_ready1[n] <= 0; lsq_ready2[n] <= 0;
                lsq_value1[n] <= 0; lsq_value2[n] <= 0;
                lsq_tag1[n] <= 0; lsq_tag2[n] <= 0;
            end
            ic_valid <= 0;
            dc_valid <= 0;
        end else begin
            registers[0] <= 0;
            // Poll completed ROB values: equivalent to tag matching on a result bus,
            // with one extra cycle of wakeup latency and no same-cycle bypass logic.
            for (integer n = 0; n < RS_DEPTH; n = n + 1) begin
                if (rs_busy[n]) begin
                    if (!rs_ready1[n] && rob_busy[rs_tag1[n]] && rob_ready[rs_tag1[n]]) begin
                        rs_ready1[n] <= 1; rs_value1[n] <= rob_value[rs_tag1[n]];
                    end
                    if (!rs_ready2[n] && rob_busy[rs_tag2[n]] && rob_ready[rs_tag2[n]]) begin
                        rs_ready2[n] <= 1; rs_value2[n] <= rob_value[rs_tag2[n]];
                    end
                end
            end
            for (integer n = 0; n < LSQ_DEPTH; n = n + 1) begin
                if (lsq_busy[n]) begin
                    if (!lsq_ready1[n] && rob_busy[lsq_tag1[n]] && rob_ready[lsq_tag1[n]]) begin
                        lsq_ready1[n] <= 1; lsq_value1[n] <= rob_value[lsq_tag1[n]];
                    end
                    if (!lsq_ready2[n] && rob_busy[lsq_tag2[n]] && rob_ready[lsq_tag2[n]]) begin
                        lsq_ready2[n] <= 1; lsq_value2[n] <= rob_value[lsq_tag2[n]];
                    end
                end
            end
            if (execute_rs >= 0 && !fault) begin
                rob_value[rs_rob[execute_rs]] <= alu_value;
                rob_next_pc[rs_rob[execute_rs]] <= alu_next_pc;
                rob_taken[rs_rob[execute_rs]] <= alu_taken;
                rob_ready[rs_rob[execute_rs]] <= 1;
                rs_busy[execute_rs] <= 0;
            end
            if (execute_lsq >= 0 && !fault) begin
                rob_address[lsq_rob[execute_lsq]] <= lsq_value1[execute_lsq] + lsq_imm[execute_lsq];
                rob_store_data[lsq_rob[execute_lsq]] <= lsq_value2[execute_lsq];
                rob_address_ready[lsq_rob[execute_lsq]] <= 1;
                lsq_busy[execute_lsq] <= 0;
            end

            if (retire) begin
                rob_busy[rob_head] <= 0;
                if (rob_rd[rob_head] != 0) begin
                    registers[rob_rd[rob_head]] <= rob_value[rob_head];
                    // A younger writer may already own this architectural register.
                    if (rat_busy[rob_rd[rob_head]] && rat_tag[rob_rd[rob_head]] == rob_head)
                        rat_busy[rob_rd[rob_head]] <= 0;
                end
                rob_head <= rob_head == ROB_DEPTH-1 ? 0 : rob_head + 1;
            end
            case ({dispatch, retire})
                2'b10: rob_count <= rob_count + 1;
                2'b01: rob_count <= rob_count - 1;
                default: rob_count <= rob_count;
            endcase
            if (dispatch) begin
                pc <= predicted_pc;
                rob_tail <= rob_tail == ROB_DEPTH-1 ? 0 : rob_tail + 1;
                rob_busy[rob_tail] <= 1;
                rob_ready[rob_tail] <= 0;
                rob_rd[rob_tail] <= writes_rd ? destination : 5'b0;
                rob_control[rob_tail] <= dec_control;
                rob_branch[rob_tail] <= instruction[6:0] == 7'h63;
                rob_pc[rob_tail] <= pc;
                rob_prediction[rob_tail] <= predicted_pc;
                rob_memory[rob_tail] <= dec_memory;
                rob_store[rob_tail] <= dec_store;
                rob_funct3[rob_tail] <= instruction[14:12];
                rob_address_ready[rob_tail] <= 0;
                if (writes_rd && destination != 0) begin
                    rat_busy[destination] <= 1;
                    rat_tag[destination] <= rob_tail;
                end
                if (dec_memory) begin
                    lsq_busy[free_lsq] <= 1;
                    lsq_rob[free_lsq] <= rob_tail;
                    lsq_imm[free_lsq] <= immediate;
                    lsq_ready1[free_lsq] <= operand_ready1;
                    lsq_ready2[free_lsq] <= operand_ready2;
                    lsq_value1[free_lsq] <= operand_value1;
                    lsq_value2[free_lsq] <= operand_value2;
                    lsq_tag1[free_lsq] <= operand_tag1;
                    lsq_tag2[free_lsq] <= operand_tag2;
                end else begin
                    rs_busy[free_rs] <= 1;
                    rs_rob[free_rs] <= rob_tail;
                    rs_instruction[free_rs] <= instruction;
                    rs_pc[free_rs] <= pc;
                    rs_imm[free_rs] <= immediate;
                    rs_ready1[free_rs] <= operand_ready1;
                    rs_ready2[free_rs] <= operand_ready2;
                    rs_value1[free_rs] <= operand_value1;
                    rs_value2[free_rs] <= operand_value2;
                    rs_tag1[free_rs] <= operand_tag1;
                    rs_tag2[free_rs] <= operand_tag2;
                end
            end

            case (bus_state)
                BUS_IDLE: if (!fault && !recover) begin
                    if (head_memory) begin
                        if (!rob_store[rob_head] && dc_hit) begin
                            rob_value[rob_head] <= load_value;
                            rob_ready[rob_head] <= 1;
                        end else begin
                            bus_address <= {head_address[31:2], 2'b0};
                            bus_load <= 1;
                            if (rob_store[rob_head]) begin
                                bus_write_data <= rob_store_data[rob_head] << {head_address[1:0], 3'b0};
                                case (rob_funct3[rob_head])
                                    0: bus_strobe <= 4'b0001 << head_address[1:0];
                                    1: bus_strobe <= 4'b0011 << head_address[1:0];
                                    default: bus_strobe <= 4'b1111;
                                endcase
                                aw_sent <= 0; w_sent <= 0;
                                bus_state <= WRITE_CHANNELS;
                            end else bus_state <= READ_ADDRESS;
                        end
                    end else if (!ic_hit) begin
                        bus_address <= {pc[31:2], 2'b0};
                        bus_load <= 0;
                        bus_state <= READ_ADDRESS;
                    end
                end
                READ_ADDRESS: if (arready) bus_state <= READ_DATA;
                READ_DATA: if (rvalid) begin
                    bus_state <= BUS_IDLE;
                    if (bus_load) begin
                        if (rresp != 0) fault <= 1;
                        else begin
                            rob_value[rob_head] <= load_value;
                            rob_ready[rob_head] <= 1;
                            dc_valid[bus_dc_index] <= 1;
                            dc_address[bus_dc_index] <= bus_address;
                            dc_data[bus_dc_index] <= rdata;
                        end
                    end else begin
                        // An obsolete speculative fetch may still fill the cache.
                        // It never changes PC, ROB, or the architectural registers.
                        ic_valid[bus_ic_index] <= 1;
                        ic_address[bus_ic_index] <= bus_address;
                        ic_data[bus_ic_index] <= rresp == 0 ? rdata : 32'b0;
                    end
                end
                WRITE_CHANNELS: begin
                    if (awready) aw_sent <= 1;
                    if (wready) w_sent <= 1;
                    if ((aw_sent || awready) && (w_sent || wready)) bus_state <= WRITE_RESPONSE;
                end
                WRITE_RESPONSE: if (bvalid) begin
                    bus_state <= BUS_IDLE;
                    if (bresp != 0) fault <= 1;
                    else begin
                        rob_ready[rob_head] <= 1;
                        if (dc_valid[bus_dc_index] && dc_address[bus_dc_index] == bus_address)
                            for (integer byte_index = 0; byte_index < 4; byte_index = byte_index + 1)
                                if (bus_strobe[byte_index])
                                    dc_data[bus_dc_index][byte_index*8 +: 8] <= bus_write_data[byte_index*8 +: 8];
                        if (ic_address[bus_ic_index] == bus_address) ic_valid[bus_ic_index] <= 0;
                    end
                end
                default: bus_state <= BUS_IDLE;
            endcase

            // Recovery at retirement is deliberately simple: every remaining entry
            // is younger, so clear all queues and RAT. No RAT checkpoint is needed.
            if (recover) begin
                pc <= rob_next_pc[rob_head];
                rob_head <= 0; rob_tail <= 0; rob_count <= 0;
                for (integer n = 0; n < ROB_DEPTH; n = n + 1) begin
                    rob_busy[n] <= 0; rob_ready[n] <= 0;
                end
                for (integer n = 0; n < RS_DEPTH; n = n + 1) rs_busy[n] <= 0;
                for (integer n = 0; n < LSQ_DEPTH; n = n + 1) lsq_busy[n] <= 0;
                for (integer n = 0; n < 32; n = n + 1) rat_busy[n] <= 0;
            end
        end
    end
endmodule
