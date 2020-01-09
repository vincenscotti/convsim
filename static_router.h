#pragma once

#include <systemc>

#include <array>

namespace convsim {

using namespace std;
using namespace sc_core;

template <size_t Srcs, size_t Dsts>
struct mcast_config {
    typedef array<array<bool, Dsts>, Srcs> routing_matrix;

    mcast_config() {
        for (auto &row : m) { row.fill(false); }
    }

    inline bool path(size_t src, size_t dst) {
        return m[src][dst];
    }

    void groupEnable(size_t src, initializer_list<size_t> dsts) {
        for (auto dst : dsts) {
            assert(dst < Dsts);
            m[src][dst] = true;
        }
    }

    void print(ostream &os = cout) {
        for (size_t src = 0; src < Srcs; src++) {
            os << "source " << src << ": ";
            for (size_t dst = 0; dst < Dsts; dst++) {
                os << m[src][dst] << " ";
            }
            os << endl;
        }
    }

    bool valid() {
        for (size_t dst = 0; dst < Dsts; dst++) {
            size_t routes_for_dst = 0;

            for (size_t src = 0; src < Srcs; src++) {
                if (m[src][dst]) routes_for_dst++;
            }

            if (routes_for_dst > 1) return false;
        }

        return true;
    }

private:
    routing_matrix m;
};

typedef enum {
    N, E, S, W, GLB, PE, N_DIRECTIONS
} direction;

template <typename DataType>
SC_MODULE(router) {
    // configuration matrix: a row for each src port, a col for each dst port
    typedef mcast_config<N_DIRECTIONS, N_DIRECTIONS> config;
    typedef DataType data_type;

    // router interface
    // a clk signal to know the propagation delay to model
    sc_in<bool> clk;
    // N input fifos, one for each source port
    array<sc_fifo_in<DataType>, N_DIRECTIONS> in;
    // N output fifos, one for each output port
    array<sc_fifo_out<DataType>, N_DIRECTIONS> out;

    SC_CTOR(router) : clk("clk") {
        // each source port has its own control flow (so a stall on one port doesn't stall the whole router)
        for (size_t i = 0; i < N_DIRECTIONS; i++) {
            sc_spawn_options opts;
            opts.set_sensitivity(&clk.pos());

            direction dir = static_cast<direction>(i);

            sc_spawn(bind(&router::port_thread, this, dir), 0, &opts);
        }
    }

    void set_config(config new_cfg) {
        cfg = new_cfg;

        // first we validate the new configuration
        if (!cfg.valid()) throw runtime_error(string(name()) + " invalid router configuration");

        cerr << "Router " << name() << endl;
        cerr << "Setting new circuit configuration" << endl;
        cfg.print(cerr);
    }

private:
    // the route configuration
    config cfg;

    void port_thread(direction src) {
        DataType data_in;

        while (true) {
            in[src].read(data_in);
            wait(1);

            for (size_t dst = 0; dst < N_DIRECTIONS; dst++) {
                // for each dst port, check if we should send there
                if (cfg.path(src, dst)) {
                    out[dst].write(data_in);
                }
            }
        }
    }
};

}
