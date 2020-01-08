#include "tests.h"

using namespace eyeriss::v2;

testbench::testbench(sc_module_name name) : testbench(name, false, false) {
}

testbench::testbench(sc_module_name name, bool first, bool last) {
    wait_start = !first;
    trigger_stop = last;

    SC_THREAD(run_thread);
    sensitive << clk.pos();
}

void testbench::run_thread() {
    if (wait_start) wait(*start);

    sc_time start_time = sc_time_stamp();
    bool success = run();
    sc_time end_time = sc_time_stamp();

    if (success) {
        cerr << "Testbench " << name() << " PASSED in " << end_time - start_time << endl << endl;
    } else {
        cerr << "Testbench " << name() << " FAILED!!!" << endl;
    }

    if (trigger_stop) sc_stop();
    else end.notify();
}

router_tb::router_tb(sc_core::sc_module_name name) : router_tb(name, false, false) {

}

router_tb::router_tb(sc_core::sc_module_name name, bool first, bool last) : testbench(name, first, last), r("r"),
                                            inputs{dfifo(1), dfifo(1), dfifo(1), dfifo(1), dfifo(1), dfifo(1)},
                                            outputs{dfifo(1), dfifo(1), dfifo(1), dfifo(1), dfifo(1), dfifo(1)}
{
    // route setup
    trouter::config c;
    c.groupEnable(GLB, {PE});

    r.set_config(c);
    r.clk(clk);

    for (size_t i = 0; i < N_DIRECTIONS; i++) {
        r.in[i](inputs[i]);
        r.out[i](outputs[i]);
    }
}

bool router_tb::run() {

    inputs[GLB].write(100);

    constexpr int cycles = 10;
    for (int i = 0; i < 2 * cycles; i++) {
        wait(clk.value_changed_event());
    }

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

    return true;

}

pe_cluster_tb::pe_cluster_tb(sc_core::sc_module_name name) : pe_cluster_tb(name, false, false) {

}

pe_cluster_tb::pe_cluster_tb(sc_core::sc_module_name name, bool first, bool last) : testbench(name, first, last), c("c") {

    c.clk(clk);

    for (size_t i = 0; i < banks; i++) c.iact_in[i](iact[i]);
    for (size_t i = 0; i < rows; i++) c.weight_in[i](weight[i]);
    for (size_t i = 0; i < cols; i++) c.psum_in[i](psum_in[i]);
    for (size_t i = 0; i < cols; i++) c.psum_out[i](psum_out[i]);

    cluster::config cfg;
    cfg.iact_propagation.groupEnable(0, {0});
    cfg.weight_propagation[0].groupEnable(0, {0});
    cfg.pe_config.kernel_w = 1;
    cfg.pe_config.kernel_h = 1;

    c.set_config(cfg);

}

bool pe_cluster_tb::run() {
    wait(1);

    iact[0].write(10);
    weight[0].write(10);

    uint32_t r1 = psum_out[0].read();

    assert(r1 == 100);

    return true;
}

pe_cluster_conv1::pe_cluster_conv1(sc_core::sc_module_name name) : pe_cluster_conv1(name, false, false) {

}

pe_cluster_conv1::pe_cluster_conv1(sc_core::sc_module_name name, bool first, bool last) : testbench(name, first, last), c("c") {

    c.clk(clk);

    for (size_t i = 0; i < banks; i++) c.iact_in[i](iact[i]);
    for (size_t i = 0; i < rows; i++) c.weight_in[i](weight[i]);
    for (size_t i = 0; i < cols; i++) c.psum_in[i](psum_in[i]);
    for (size_t i = 0; i < cols; i++) c.psum_out[i](psum_out[i]);

    cluster::config cfg;

    cfg.iact_propagation.groupEnable(0, {0});
    cfg.iact_propagation.groupEnable(1, {1, 2});
    cfg.iact_propagation.groupEnable(2, {3});

    cfg.weight_propagation[0].groupEnable(0, {0, 1});
    cfg.weight_propagation[1].groupEnable(0, {0, 1});

    cfg.pe_config.kernel_w = kernel_c;
    cfg.pe_config.kernel_h = kernel_r;

    c.set_config(cfg);

}

bool pe_cluster_conv1::run() {
    wait(1);

    array<array<uint32_t, ifmap_c>, ifmap_r> ifmap;
    array<array<uint32_t, kernel_c>, kernel_r> kernel;
    array<array<uint32_t, ofmap_c>, ofmap_r> ofmap;

    // precompute 2d conv
    for (size_t i_r = 0; i_r < ifmap_r; i_r++) {
        for (size_t i_c = 0; i_c < ifmap_c; i_c++) {
            ifmap[i_r][i_c] = i_r * 3 + i_c + 1;
        }
    }

    for (size_t k_r = 0; k_r < kernel_r; k_r++) {
        for (size_t k_c = 0; k_c < kernel_c; k_c++) {
            kernel[k_r][k_c] = k_r * 2 + k_c + 1;
        }
    }

    for (size_t o_r = 0; o_r < ofmap_r; o_r++) {
        for (size_t o_c = 0; o_c < ofmap_c; o_c++) {
            ofmap[o_r][o_c] = 0;

            for (size_t k_r = 0; k_r < kernel_r; k_r++) {
                for (size_t k_c = 0; k_c < kernel_c; k_c++) {
                    ofmap[o_r][o_c] += ifmap[o_r + k_r][o_c + k_c] * kernel[k_r][k_c];
                }
            }
        }
    }

    // first weight row
    weight[0].write(1);
    weight[0].write(2);
    // second weight row
    weight[1].write(3);
    weight[1].write(4);
    // first iact row
    iact[0].write(1);
    iact[0].write(2);
    iact[0].write(3);
    // second iact row
    iact[1].write(4);
    iact[1].write(5);
    iact[1].write(6);
    // third iact row
    iact[2].write(7);
    iact[2].write(8);
    iact[2].write(9);

    for (size_t o_r = 0; o_r < ofmap_r; o_r++) {
        for (size_t o_c = 0; o_c < ofmap_c; o_c++) {
            // we read the row elem by elem (so column-wise)
            uint32_t val = psum_out[o_r].read();
            assert(val == ofmap[o_r][o_c]);
        }
    }

    return true;
}
