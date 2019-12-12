#include <iostream>
#include <array>
#include <functional>
#include <memory>

#include <systemc>

#include "tests.h"

using namespace std;
using namespace sc_core;
using namespace sc_dt;

#include "eyeriss_v2.h"

using namespace eyeriss::v2;

void router_test(sc_clock &clk) {
    typedef router<uint32_t> trouter;

    // route setup
    trouter::config c;
    c.groupEnable(GLB, {PE});

    // 1-slot IO fifos
    typedef sc_fifo<trouter::data_type> dfifo;
    array<dfifo, N_DIRECTIONS> inputs{dfifo(1), dfifo(1), dfifo(1), dfifo(1), dfifo(1), dfifo(1)};
    array<dfifo, N_DIRECTIONS> outputs{dfifo(1), dfifo(1), dfifo(1), dfifo(1), dfifo(1), dfifo(1)};

    // router initialization
    trouter r("r");
    r.set_config(c);
    r.clk(clk);

    for (size_t i = 0; i < N_DIRECTIONS; i++) {
        r.in[i](inputs[i]);
        r.out[i](outputs[i]);
    }

    sc_start(clk.period());

    inputs[GLB].write(100);

    sc_start(3 * clk.period());

    // it should leave from the PE port
    assert(outputs[N].num_free() == 1);
    assert(outputs[E].num_free() == 1);
    assert(outputs[S].num_free() == 1);
    assert(outputs[W].num_free() == 1);
    assert(outputs[GLB].num_free() == 1);
    assert(outputs[PE].num_free() == 0);

    uint32_t readback;
    outputs[PE].read(readback);
    assert(readback == 100);
}

typedef uint8_t weight_t;
typedef uint8_t iact_t;
typedef uint8_t psum_t;

typedef router<weight_t> wrouter;
typedef router_cluster<weight_t, iact_t, psum_t, 3, 4> cluster;

int sc_main (int, char *[]) {
    const double clk_period = 10;
    sc_clock clk("clk", clk_period, SC_NS);

    //pe_cluster_tb pe_tb("pe_tb");
    //pe_tb.clk(clk);

    pe_cluster_conv1 pe_conv1("pe_conv1");
    pe_conv1.clk(clk);

    //router_test(clk);

    sc_start(1000 * clk.period());

    return 0;
}
