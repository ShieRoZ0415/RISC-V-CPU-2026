// Reuse the official RAM model without editing the framework.
#define main course_simulator_main
#include "../scripts/sim.cpp"
#undef main
#include "Vstudent_top___024root.h"

struct Stability {
    bool stalled = false;
    uint64_t payload = 0;
    void check(bool valid, bool ready, uint64_t current) {
        if (stalled && (!valid || payload != current))
            throw std::runtime_error("AXI VALID/payload changed under backpressure");
        stalled = valid && !ready;
        payload = current;
    }
};

int main(int argc, char **argv) {
    try {
        if (argc != 3) throw std::runtime_error("usage: axi_stress IMAGE EXPECTED");
        Memory memory;
        memory.load(argv[1]);
        VerilatedContext context;
        Vstudent_top top{&context};
        Stability ar_check, aw_check, w_check;
        bool aw_seen = false, w_seen = false;
        unsigned writes = 0, aw_first = 0, w_first = 0;
        bool saw_ooo = false;
        unsigned recoveries = 0;
        auto tick = [&]() {
            top.clock = 0;
            memory.drive(top);
            if (!top.reset) {
                top.arready = top.arready && memory.cycle % 7 == 0;
                // Alternate strictly AW-before-W and W-before-AW transactions.
                // The second channel waits many cycles after the first handshakes.
                if (writes % 2 == 0) {
                    top.awready = top.awready && memory.cycle % 3 == 0;
                    top.wready = top.wready && aw_seen && memory.cycle % 13 == 0;
                } else {
                    top.wready = top.wready && memory.cycle % 3 == 0;
                    top.awready = top.awready && w_seen && memory.cycle % 13 == 0;
                }
            }
            top.eval();
            if (!top.reset) {
                const auto &state = *top.rootp;
                const int selected = int32_t(state.student_top__DOT__execute_rs);
                const auto head = state.student_top__DOT__rob_head;
                if (selected >= 0 && state.student_top__DOT__rs_rob[selected] != head &&
                    !state.student_top__DOT__rob_ready[head]) saw_ooo = true;
                if (state.student_top__DOT__recover) ++recoveries;
                ar_check.check(top.arvalid, top.arready, top.araddr);
                aw_check.check(top.awvalid, top.awready, top.awaddr);
                w_check.check(top.wvalid, top.wready, (uint64_t(top.wstrb) << 32) | top.wdata);
                if (top.awvalid && top.awready) {
                    if (!w_seen) ++aw_first;
                    aw_seen = true;
                }
                if (top.wvalid && top.wready) {
                    if (!aw_seen) ++w_first;
                    w_seen = true;
                }
                if (top.bvalid && top.bready) {
                    ++writes;
                    aw_seen = false; w_seen = false;
                }
            }
            memory.step(top, 17);
            top.clock = 1;
            top.eval();
            context.timeInc(2);
        };
        top.reset = 1;
        for (int n = 0; n < 5; ++n) tick();
        top.reset = 0;
        while (!memory.done && memory.cycle < 10000000) tick();
        top.final();
        if (!memory.done || memory.result != std::stoul(argv[2]) || !aw_first || !w_first || !saw_ooo || !recoveries)
            throw std::runtime_error("wrong result, timeout, or missing AXI/OoO/recovery coverage");
        std::cout << "PASS: cycles=" << memory.cycle << " AW-first=" << aw_first
                  << " W-first=" << w_first << " OoO=" << saw_ooo << " recoveries=" << recoveries << " result=" << memory.result << '\n';
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
