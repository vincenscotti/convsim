#include <iostream>
#include <array>
#include <functional>
#include <memory>

#include <systemc>

#include "row_stationary.h"
#include "tests.h"

using namespace std;
using namespace sc_core;
using namespace sc_dt;

using namespace convsim;
using namespace convsim::row_stationary;
using namespace convsim::tests;

typedef uint8_t weight_t;
typedef uint8_t iact_t;
typedef uint8_t psum_t;

typedef router<weight_t> wrouter;
typedef router_cluster<weight_t, iact_t, psum_t, 3, 4> cluster;

int sc_main (int, char *[]) {
    const double clk_period = 10;
    sc_clock clk("clk", clk_period, SC_NS);

    router_tb r_tb("r_tb", true, false);
    r_tb.clk(clk);

    pe_cluster_tb pe_tb("pe_tb", false, false);
    pe_tb.clk(clk);

    pe_cluster_conv1 pe_conv1("pe_conv1", false, true);
    pe_conv1.clk(clk);

    pe_tb.start = &r_tb.end;
    pe_conv1.start = &pe_tb.end;

    sc_start();

    return 0;
}
