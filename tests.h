#pragma once

#include <systemc>
#include "eyeriss_v2.h"

SC_MODULE(pe_cluster_tb) {
    sc_in<bool> clk;

    SC_CTOR(pe_cluster_tb);

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

    void main();
};

SC_MODULE(pe_cluster_conv1) {
    sc_in<bool> clk;

    SC_CTOR(pe_cluster_conv1);

private:
    static constexpr size_t rows = 2;
    static constexpr size_t cols = 2;
    static constexpr size_t banks = 3;

    static constexpr size_t ifmap_r = 3;
    static constexpr size_t ifmap_c = 3;
    static constexpr size_t kernel_r = 2;
    static constexpr size_t kernel_c = 2;
    static constexpr size_t ofmap_r = 2;
    static constexpr size_t ofmap_c = 2;

    typedef sc_fifo<uint32_t> fifo;
    typedef eyeriss::v2::pe_cluster<uint32_t, uint32_t, uint32_t, rows, cols, banks> cluster;

    cluster c;
    array<fifo, banks> iact;
    array<fifo, rows> weight;
    array<fifo, cols> psum_in;
    array<fifo, cols> psum_out;

    void main();
};
