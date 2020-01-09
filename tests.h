#pragma once

#include <systemc>
#include <array>

#include "row_stationary.h"

namespace convsim {
namespace tests {

using namespace std;
using namespace sc_core;

SC_MODULE(testbench) {
    sc_in<bool> clk;

    SC_CTOR(testbench);
    testbench(sc_module_name name, bool first, bool last);

    sc_event *start;
    sc_event end;

    virtual bool run() = 0;

protected:
    void aux_thread_wait();

private:
    void run_thread();

    bool wait_start;
    bool trigger_stop;
};

struct router_tb : testbench {
    SC_CTOR(router_tb);
    router_tb(sc_module_name name, bool first, bool last);

    virtual bool run() override;

private:
    typedef convsim::router<uint32_t> trouter;
    typedef sc_fifo<trouter::data_type> dfifo;

    trouter r;
    array<dfifo, convsim::N_DIRECTIONS> inputs;
    array<dfifo, convsim::N_DIRECTIONS> outputs;
};

struct pe_cluster_tb : testbench {
    SC_CTOR(pe_cluster_tb);
    pe_cluster_tb(sc_module_name name, bool first, bool last);

    virtual bool run() override;

private:
    static constexpr size_t rows = 3;
    static constexpr size_t cols = 4;
    static constexpr size_t banks = 3;

    typedef sc_fifo<uint32_t> fifo;
    typedef convsim::row_stationary::pe_cluster<uint32_t, uint32_t, uint32_t, rows, cols, banks> cluster;

    cluster c;
    array<fifo, rows> iact;
    array<fifo, rows> weight;
    array<fifo, cols> psum_in;
    array<fifo, cols> psum_out;
};

struct pe_cluster_conv1 : testbench {
    SC_CTOR(pe_cluster_conv1);
    pe_cluster_conv1(sc_module_name name, bool first, bool last);

    virtual bool run() override;

private:
    static constexpr size_t ifmap_r = 3;
    static constexpr size_t ifmap_c = 3;
    static constexpr size_t kernel_r = 2;
    static constexpr size_t kernel_c = 2;
    static constexpr size_t ofmap_r = 2;
    static constexpr size_t ofmap_c = 2;

    static constexpr size_t rows = kernel_r;
    static constexpr size_t cols = ofmap_r;
    static constexpr size_t banks = rows + cols - 1;

    typedef sc_fifo<uint32_t> fifo;
    typedef convsim::row_stationary::pe_cluster<uint32_t, uint32_t, uint32_t, rows, cols, banks> cluster;

    void weight_write_thread(int bank);
    void iact_write_thread(int bank);
    void psum_read_thread(int bank);
    void inject(fifo *f, vector<uint32_t> data);

    sc_event_queue read_done;

    array<array<uint32_t, ifmap_c>, ifmap_r> ifmap;
    array<array<uint32_t, kernel_c>, kernel_r> kernel;
    array<array<uint32_t, ofmap_c>, ofmap_r> ofmap;

    cluster c;
    array<fifo, banks> iact_fifo;
    array<fifo, rows> weight_fifo;
    array<fifo, cols> psum_in_fifo;
    array<fifo, cols> psum_out_fifo;
};

}
}
