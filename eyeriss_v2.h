#pragma once

#include <systemc>

#include <memory>
#include <functional>
#include <list>

#define MOD_DBG(x) cerr << "module " << name() << " @ " << sc_time_stamp() << ": " << x << endl;
//#define MOD_DBG(x)

using namespace std;
using namespace sc_core;
using namespace sc_dt;

namespace eyeriss {
namespace v2 {

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
#define LAUNCH_SRC_PORT_THREAD(DIR) SC_THREAD(forward_##DIR); sensitive << clk.pos();

        LAUNCH_SRC_PORT_THREAD(N);
        LAUNCH_SRC_PORT_THREAD(E);
        LAUNCH_SRC_PORT_THREAD(S);
        LAUNCH_SRC_PORT_THREAD(W);
        LAUNCH_SRC_PORT_THREAD(GLB);
        LAUNCH_SRC_PORT_THREAD(PE);
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

#define DEFINE_SRC_PORT_FN(DIR) void forward_##DIR() { forward(DIR); }
    DEFINE_SRC_PORT_FN(N);
    DEFINE_SRC_PORT_FN(E);
    DEFINE_SRC_PORT_FN(S);
    DEFINE_SRC_PORT_FN(W);
    DEFINE_SRC_PORT_FN(GLB);
    DEFINE_SRC_PORT_FN(PE);

    void forward(direction src) {
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

template <typename W_t, typename IAct_t, typename PSum_t, size_t PERows, size_t PECols>
SC_MODULE(router_cluster) {
    typedef router<W_t> wrouter;
    typedef router<IAct_t> irouter;
    typedef router<PSum_t> prouter;

    // we make an array of pointers, as we need to dynamically pick modules names
    array<wrouter *, PERows> wrouters;
    array<irouter *, PERows> irouters;
    array<prouter *, PECols> prouters;

    SC_CTOR(router_cluster) {
        for (size_t i = 0; i < wrouters.size(); i++) {
            const string name = "wr_" + to_string(i);
            wrouters[i] = new wrouter(name.c_str());
        }

        for (size_t i = 0; i < irouters.size(); i++) {
            const string name = "ir_" + to_string(i);
            irouters[i] = new irouter(name.c_str());
        }

        for (size_t i = 0; i < prouters.size(); i++) {
            const string name = "pr_" + to_string(i);
            prouters[i] = new prouter(name.c_str());
        }
    }

    ~router_cluster() {
        for_each(wrouters.begin(), wrouters.end(), default_delete<wrouter>());
        for_each(irouters.begin(), irouters.end(), default_delete<irouter>());
        for_each(prouters.begin(), prouters.end(), default_delete<prouter>());
    }
};

template <typename W_t, typename IAct_t, typename PSum_t>
SC_MODULE(processing_element) {
    struct config {
        size_t kernel_w;
        size_t kernel_h;
        bool psum_acc_in;

        bool valid() {
            return kernel_w > 0 && kernel_h > 0;
        }
    };

    // PE interface
    // clock signal
    sc_in<bool> clk;
    // activations input FIFO
    sc_fifo_in<IAct_t> iact_in;
    // weights input FIFO
    sc_fifo_in<W_t> weight_in;
    // psums input FIFO
    sc_fifo_in<PSum_t> psum_in;
    // psums output FIFO
    sc_fifo_out<PSum_t> psum_out;

private:
    // internal structure
    config cfg;
    // pipe stage1 to stage2 fifo
    sc_fifo<IAct_t> fifo_1to2;
    // sliding window - max KW-1 elements
    list<IAct_t> iact_win;
    // weight storage
    vector<W_t> weight_row;
    // pipe stage2 to stage3 fifo
    sc_fifo<IAct_t> fifo_2to3_act;
    sc_fifo<W_t> fifo_2to3_w;

public:
    SC_CTOR(processing_element) : clk("clk"), iact_in("iact_in"), weight_in("weight_in"), psum_in("psum_in"),
                                  psum_out("psum_out"), fifo_1to2(1), fifo_2to3_act(1), fifo_2to3_w(1) {
        SC_THREAD(stage1);
        sensitive << clk.pos();

        SC_THREAD(stage2);
        sensitive << clk.pos();

        SC_THREAD(stage3);
        sensitive << clk.pos();
    }

    void set_config(config new_cfg) {
        assert(new_cfg.kernel_w > 0);
        assert(new_cfg.kernel_h > 0);

        cfg = new_cfg;
    }

private:
    void stage1() {
        //while (true) {
        //    IAct_t iact;

        //    iact_in.read(iact);
        //    wait(1);
        //    fifo_1to2.write(iact);
        //    MOD_DBG("stage 1: propagate iact");
        //}

        // first sliding window generation
        for (size_t i = 0; i < cfg.kernel_w; i++) {
            IAct_t iact;

            iact_in.read(iact);
            wait(1);
            fifo_1to2.write(iact);
            if (i > 0) iact_win.push_back(iact);
            MOD_DBG("stage 1: propagate iact");
        }

        while (true) {
            // we send first KW-1 window elements (which we already saved)
            for (auto iact : iact_win) {
                wait(1);
                fifo_1to2.write(iact);
                MOD_DBG("stage 1: propagate iact");
            }

            // then the last one
            {
                IAct_t iact;

                iact_in.read(iact);
                wait(1);
                fifo_1to2.write(iact);
                iact_win.pop_front();
                iact_win.push_back(iact);
                MOD_DBG("stage 1: propagate iact");
            }
        }
    }

    void stage2() {
        size_t next_weight_ptr = 0;

        while (true) {
            IAct_t iact;
            W_t w;

            fifo_1to2.read(iact);

            if (weight_row.size() < next_weight_ptr + 1) {
                weight_in.read(w);
                weight_row.push_back(w);
            }

            w = weight_row[next_weight_ptr];

            wait(1);
            fifo_2to3_act.write(iact);
            MOD_DBG("stage 2: propagate iact");
            fifo_2to3_w.write(w);
            MOD_DBG("stage 2: propagate weight column " << next_weight_ptr);

            next_weight_ptr = (next_weight_ptr + 1) % cfg.kernel_w;
        }
    }

    void stage3() {
        PSum_t local_psum = 0;
        PSum_t remote_psum = 0;

        while (true) {
            local_psum = 0;

            for (size_t i = 0; i < cfg.kernel_w; i++) {
                IAct_t iact;
                W_t w;

                fifo_2to3_act.read(iact);
                fifo_2to3_w.read(w);

                local_psum = local_psum + iact * w;
                wait(1);

                if (i == cfg.kernel_w - 1) {
                    if (cfg.psum_acc_in) {
                        psum_in.read(remote_psum);
                        local_psum += remote_psum;
                        wait(1);
                    }

                    psum_out.write(local_psum);
                    MOD_DBG("stage 3: propagate psum");
                }
            }
        }
    }
};

template <typename W_t, typename IAct_t, typename PSum_t, size_t PERows, size_t PECols, size_t IActBanks>
SC_MODULE(pe_cluster) {
    typedef processing_element<W_t, IAct_t, PSum_t> pe;
    typedef sc_fifo<IAct_t> ififo;
    typedef sc_fifo<W_t> wfifo;
    typedef sc_fifo<PSum_t> pfifo;
    typedef sc_fifo_in<IAct_t> ififo_in;
    typedef sc_fifo_in<W_t> wfifo_in;
    typedef sc_fifo_in<PSum_t> pfifo_in;
    typedef sc_fifo_out<PSum_t> pfifo_out;

    struct config {
        mcast_config<IActBanks, PERows * PECols> iact_propagation;
        array<mcast_config<1, PECols>, PERows> weight_propagation;
        typename pe::config pe_config;
    };

    // PE cluster interface
    // clock signal
    sc_in<bool> clk;
    // activations input FIFO
    array<ififo_in, IActBanks> iact_in;
    // weights input FIFO
    array<wfifo_in, PERows> weight_in;
    // psums input FIFO
    array<pfifo_in, PECols> psum_in;
    // psums output FIFO
    array<pfifo_out, PECols> psum_out;

private:
    // internal structure
    array<array<pe *, PECols>, PERows> grid;
    // iact propagation FIFOs - 1 per PE
    array<array<ififo, PECols>, IActBanks> iact_fifos;
    // weight propagation fifos - 1 per PE
    array<array<wfifo, PECols>, PERows> weight_fifos;
    // psum propagation fifos - 1 per PE minus row 0
    array<array<pfifo, PECols>, PERows - 1> psum_fifos;
    // propagation configuration
    config cfg;

public:
    SC_CTOR(pe_cluster) {
        // we generate rows from the last one
        for (ssize_t row = PERows - 1; row >= 0; row--) {
            for (size_t col = 0; col < PECols; col++) {
                const string name = "pe_" + to_string(row) + "_" + to_string(col);
                pe *p = new pe(name.c_str());

                p->clk(clk);

                // iacts are broadcasted to all PEs
                p->iact_in(iact_fifos[row][col]);

                // weights are broadcasted on a row
                p->weight_in(weight_fifos[row][col]);

                // psums are systolically propagated along the grid height
                if (row < static_cast<ssize_t>(PERows) - 1) {
                    p->psum_in(psum_fifos[row][col]);
                } else {
                    p->psum_in(psum_in[col]);
                }

                // psums are systolically propagated along the grid height
                if (row > 0) {
                    p->psum_out(psum_fifos[row - 1][col]);
                } else {
                    p->psum_out(psum_out[col]);
                }

                grid[row][col] = p;
            }
        }

        // one iact propagation thread per bank
        for (size_t i = 0; i < IActBanks; i++) {
            sc_spawn_options opts;
            opts.set_sensitivity(&clk.pos());

            sc_spawn(bind(&pe_cluster::iact_thread, this, i), 0, &opts);
        }

        // one weight propagation thread per row
        for (size_t i = 0; i < PERows; i++) {
            sc_spawn_options opts;
            opts.set_sensitivity(&clk.pos());

            sc_spawn(bind(&pe_cluster::weight_thread, this, i), 0, &opts);
        }
    }

    ~pe_cluster() {
        for (auto &row : grid) {
            for_each(row.begin(), row.end(), default_delete<pe>());
        }
    }

    void set_config(config new_cfg) {
        cfg = new_cfg;

        // first we validate the new configuration
        if (!cfg.iact_propagation.valid()) {
            throw runtime_error(string(name()) + " invalid PE cluster configuration (iact)");
        }

        for (auto &row : cfg.weight_propagation) {
            if (!row.valid()) {
                throw runtime_error(string(name()) + " invalid PE cluster configuration (weights)");
            }
        }

        if (!cfg.pe_config.valid()) {
            throw runtime_error(string(name()) + " invalid PE cluster configuration (PE)");
        }

        cerr << "PE cluster " << name() << endl;
        cerr << "Setting new iact multicast configuration" << endl;
        cfg.iact_propagation.print(cerr);
        cerr << "Setting new weight multicast configuration" << endl;
        for (auto &row : cfg.weight_propagation) {
            row.print(cerr);
        }

        cerr << "Setting new PE configuration" << endl;
        for (size_t row = 0; row < PERows; row++) {
            for (size_t col = 0; col < PECols; col++) {
                cfg.pe_config.psum_acc_in = row < (cfg.pe_config.kernel_h - 1);
                grid[row][col]->set_config(cfg.pe_config);
            }
        }
    }

private:
    void iact_thread(int bank) {
        IAct_t iact;

        while (true) {
            iact_in[bank].read(iact);
            wait(1);

            for (size_t pos = 0; pos < PERows * PECols; pos++) {
                // each PE has an iact fifo... check if we should send there
                if (cfg.iact_propagation.path(bank, pos)) {
                    iact_fifos[pos / PECols][pos % PECols].write(iact);
                }
            }
        }
    }

    void weight_thread(int row) {
        W_t weight;

        while (true) {
            weight_in[row].read(weight);
            wait(1);

            for (size_t pos = 0; pos < PECols; pos++) {
                // each PE in this row has a weight fifo... check if we should send there
                if (cfg.weight_propagation[row].path(0, pos)) {
                    weight_fifos[row][pos % PECols].write(weight);
                }
            }
        }
    }
};

}
}
