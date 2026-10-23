# PCIe Gen4 HDMA DMA engine — lint/build (Verilator).
# RTL is fully synthesizable; Verilator is the reference lint/sim tool.

PKG     := rtl/dma_pkg.sv
BLOCKS  := rtl/blocks/dma_channel.sv \
           rtl/blocks/chan_arbiter.sv \
           rtl/blocks/ats_pri_subsys.sv \
           rtl/blocks/csr_block.sv
TOP     := rtl/dma_top.sv
IFS     := rtl/if/pcie_req_if.sv \
           rtl/if/ats_pri_if.sv \
           rtl/if/axi4_if.sv \
           rtl/if/axi4s_if.sv \
           rtl/if/csr_apb_if.sv
LINTTOP := sim/lint_top.sv

RTL     := $(PKG) $(BLOCKS) $(TOP)

# Waivers for blocks that are still structural stubs (unused/undriven signals,
# params defined ahead of the RTL that consumes them). Drop as each block is
# fleshed out — csr_block is already real logic and must pass without relying
# on these for anything but its not-yet-consumed storage regs.
SKEL_WAIVE := -Wno-UNUSEDSIGNAL -Wno-UNDRIVEN -Wno-UNUSEDPARAM

.PHONY: lint lint-if lint-strict sim-csr sim-m2 sim-m3 sim-m4 synth-csr clean

# Milestone M4 directed simulation (ATS, PASID, ATC)
sim-m4:
	verilator --binary -sv --timing $(SKEL_WAIVE) \
	  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
	  $(RTL) sim/axi_ram.sv sim/tb_m4.sv --top-module tb_m4 -o tb_m4
	@./obj_dir/tb_m4

# Milestone M3 directed simulation
sim-m3:
	verilator --binary -sv --timing $(SKEL_WAIVE) \
	  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
	  $(RTL) sim/axi_ram.sv sim/tb_m3.sv --top-module tb_m3 -o tb_m3
	@./obj_dir/tb_m3

# Milestone M2 directed simulation
sim-m2:
	verilator --binary -sv --timing $(SKEL_WAIVE) \
	  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
	  $(RTL) sim/axi_ram.sv sim/tb_m2.sv --top-module tb_m2 -o tb_m2
	@./obj_dir/tb_m2

# Behavioral sim of the register file (self-checking APB testbench).
sim-csr:
	verilator --binary -sv --timing -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-UNDRIVEN \
	  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
	  $(PKG) rtl/blocks/csr_block.sv sim/tb_csr.sv --top-module tb_csr -o tb_csr
	@./obj_dir/tb_csr

# Full RTL hierarchy from dma_top — must be clean (synth-oriented -Wall).
lint:
	verilator --lint-only -sv -Wall $(SKEL_WAIVE) $(RTL) --top-module dma_top

# TB-facing interface contracts (rtl/if) via lint_top.
lint-if:
	verilator --lint-only -sv -Wall $(SKEL_WAIVE) $(PKG) $(IFS) $(LINTTOP) --top-module lint_top

# Full -Wall incl. skeleton noise (informational; non-zero exit expected).
lint-strict:
	-verilator --lint-only -sv -Wall $(RTL) --top-module dma_top

# Synthesis check via sv2v -> yosys (generic mapping). Proves synthesizability.
SV2V ?= $(shell command -v sv2v || echo /home/linuxbrew/.linuxbrew/bin/sv2v)
synth-csr:
	@mkdir -p build
	$(SV2V) $(PKG) rtl/blocks/csr_block.sv --write=build/csr_block.v
	yosys -p "read_verilog build/csr_block.v; synth -top csr_block -flatten; check -assert; stat"

clean:
	rm -rf obj_dir build *.vvp
