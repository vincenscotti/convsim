#include <iostream>
#include <systemc>

using namespace std;
using namespace sc_core;
using namespace sc_dt;

SC_MODULE(PE) {
    sc_in<bool> clk;
    sc_in<bool> rst;

    SC_CTOR(PE) {
        SC_METHOD(print);
        sensitive << clk.pos();
        dont_initialize();
    }

    void print() {
        if (!rst) {
            cout << "clock edge at " << sc_time_stamp().to_string() << "!" << endl;
        }
    }
};

int sc_main (int, char *[]) {
    constexpr double clk_period = 10;

    sc_set_time_resolution(1, SC_NS);

    PE pe("p");
    sc_clock clk("clk", clk_period, SC_NS);
    sc_signal<bool> rst;

    pe.clk(clk);
    pe.rst(rst);

    rst = true;

    auto trace_file = sc_create_vcd_trace_file("dump");
    sc_trace(trace_file, pe.clk, "pe.clk");
    sc_trace(trace_file, pe.rst, "pe.rst");

    sc_start((2 * clk_period) + (clk_period / 2), SC_NS);

    rst = false;

    sc_start(10 * clk_period, SC_NS);

    sc_close_vcd_trace_file(trace_file);

    return 0;
}
