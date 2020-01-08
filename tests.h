#pragma once

#include <systemc>
#include "eyeriss_v2.h"

SC_MODULE(testbench) {
    sc_in<bool> clk;

    SC_CTOR(testbench);
    testbench(sc_module_name name, bool first, bool last);

    sc_event *start;
    sc_event end;

    virtual bool run() = 0;

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
    typedef eyeriss::v2::router<uint32_t> trouter;
    typedef sc_fifo<trouter::data_type> dfifo;

    trouter r;
    array<dfifo, eyeriss::v2::N_DIRECTIONS> inputs;
    array<dfifo, eyeriss::v2::N_DIRECTIONS> outputs;
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
    typedef eyeriss::v2::pe_cluster<uint32_t, uint32_t, uint32_t, rows, cols, banks> cluster;

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
    typedef eyeriss::v2::pe_cluster<uint32_t, uint32_t, uint32_t, rows, cols, banks> cluster;

    cluster c;
    array<fifo, banks> iact;
    array<fifo, rows> weight;
    array<fifo, cols> psum_in;
    array<fifo, cols> psum_out;
};
