# =============================================================================
# Makefile: Multi-Clock UART Controller Simulation
# =============================================================================

RTL_DIR   = ../rtl
VERIF_DIR = ../verif
TOP       = tb_uart_top

# RTL source files (order matters for dependencies)
RTL_SRCS = \
	$(RTL_DIR)/sync_2ff.sv \
	$(RTL_DIR)/async_fifo.sv \
	$(RTL_DIR)/baud_gen.sv \
	$(RTL_DIR)/uart_tx.sv \
	$(RTL_DIR)/uart_rx.sv \
	$(RTL_DIR)/uart_interrupt.sv \
	$(RTL_DIR)/apb_uart_regs.sv \
	$(RTL_DIR)/uart_top.sv

# Verification sources
VERIF_SRCS = \
	$(VERIF_DIR)/uart_sva_props.sv \
	$(VERIF_DIR)/tb_uart_top.sv

ALL_SRCS = $(RTL_SRCS) $(VERIF_SRCS)

# =============================================================================
# Vivado xsim (default for FPGA flow)
# =============================================================================
.PHONY: xsim xelab xsim_run clean

xsim: xelab xsim_run

xelab:
	xvlog -sv $(ALL_SRCS)
	xelab $(TOP) -s sim_snapshot -debug all

xsim_run:
	xsim sim_snapshot -runall -wdb waves.wdb

# =============================================================================
# Icarus Verilog (open-source, no SVA support)
# =============================================================================
.PHONY: iverilog

iverilog:
	iverilog -g2012 -o sim.vvp $(RTL_SRCS) $(VERIF_DIR)/tb_uart_top.sv
	vvp sim.vvp

# =============================================================================
# VCS (for SVA, coverage - EDA Playground or local license)
# =============================================================================
.PHONY: vcs

vcs:
	vcs -sverilog -assert svaext -cm line+cond+fsm+tgl+branch \
	    -debug_access+all $(ALL_SRCS) -o simv
	./simv -cm line+cond+fsm+tgl+branch +fsdbDumpfile=waves.fsdb

# =============================================================================
# Xcelium (for SVA, coverage)
# =============================================================================
.PHONY: xcelium

xcelium:
	xrun -sv -access +rwc -coverage all -svseed random \
	     $(ALL_SRCS) -top $(TOP)

# =============================================================================
# Yosys (ASIC synthesis check)
# =============================================================================
.PHONY: synth_yosys

synth_yosys:
	yosys -p "read_verilog -sv $(RTL_SRCS); \
	          synth -top uart_top; \
	          stat; \
	          write_verilog synth_out.v"

# =============================================================================
# Vivado FPGA synthesis (batch mode)
# =============================================================================
.PHONY: synth_vivado

synth_vivado:
	vivado -mode batch -source synth_vivado.tcl

# =============================================================================
# Clean
# =============================================================================
clean:
	rm -rf xsim.dir *.wdb *.vcd *.log *.jou *.pb
	rm -rf simv simv.daidir csrc *.fsdb ucli.key
	rm -rf xceliumd INCA_libs xrun.* *.shm
	rm -rf sim.vvp *.vvp a.out
	rm -rf synth_out.v
	rm -rf .Xil
