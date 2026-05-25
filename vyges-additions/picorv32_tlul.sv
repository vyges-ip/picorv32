// picorv32_tlul.sv — Thin TL-UL host wrapper around YosysHQ picorv32
//
// PicoRV32 has a single unified simple memory interface (mem_valid /
// mem_ready / mem_addr / mem_wdata / mem_wstrb / mem_rdata) used for
// both instruction fetch and data access — distinguished only by the
// mem_instr flag. This wrapper bridges that interface to a single
// TL-UL host port via tlul_adapter_host.
//
// The boot ROM is NOT inside this wrapper (unlike rv_core_ibex_tlul,
// which carries an inline BRAM boot ROM). PicoRV32's reset vector is
// the PROGADDR_RESET parameter; the SoC generator points it at
// memory.rom.base_address and instruction fetches travel through the
// crossbar to the u_rom TL-UL slave like any other access.
//
// Interrupt model: PicoRV32's irq input is 32 bits. The PLIC's claimed
// external IRQ is wired to irq[31]; the other 31 lines are masked off
// via MASKED_IRQ so PicoRV32's built-in timer / ebreak / bus-error IRQs
// don't fire (we don't have firmware support for them yet).
//
// References:
//   - Native interface contract: YosysHQ/picorv32 README.md "PicoRV32
//     Native Memory Interface" section.
//   - TL-UL host adapter: opentitan-tlul/rtl/tlul_adapter_host.sv

module picorv32_tlul #(
  parameter logic [31:0] PROGADDR_RESET    = 32'h0000_8000,
  parameter logic [31:0] STACKADDR         = 32'h1000_4000,
  parameter logic        ENABLE_MUL        = 1'b1,
  parameter logic        ENABLE_DIV        = 1'b1,
  parameter logic        ENABLE_IRQ        = 1'b1,
  parameter logic        ENABLE_PCPI       = 1'b0,
  parameter logic        ENABLE_REGS_16_31 = 1'b1,
  parameter logic        ENABLE_COUNTERS   = 1'b1,
  parameter logic        TWO_STAGE_SHIFT   = 1'b0,
  parameter logic        COMPRESSED_ISA    = 1'b1
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // TL-UL host bus to crossbar
  output tlul_pkg::tl_h2d_t tl_o,
  input  tlul_pkg::tl_d2h_t tl_i,

  // External IRQ (PLIC claimed-source line); routed to picorv32 irq[31]
  input  logic              irq_external_i,

  // Trap status — high when picorv32 has trapped (illegal insn, misalign,
  // ebreak). Observable for post-silicon bring-up; not wired to GPIO yet.
  output logic              trap_o
);

  // ── PicoRV32 native simple memory interface ─────────────────────────────
  logic        mem_valid;
  logic        mem_instr;
  logic        mem_ready;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [ 3:0] mem_wstrb;
  logic [31:0] mem_rdata;

  logic [31:0] eoi_unused;
  logic        trace_valid_unused;
  logic [35:0] trace_data_unused;

  logic [31:0] irq_lines;
  always_comb begin
    irq_lines        = '0;
    irq_lines[31]    = irq_external_i;   // PLIC claim → external trap
  end

`ifdef PICORV32_BLACKBOX
  // When PICORV32_BLACKBOX is defined, instantiate the bare picorv32 module
  // without a parameter list. Use this when this wrapper is synthesized
  // against a picorv32 blackbox stub that has no parameter declarations.
  // Default path (else branch) keeps the parameterised instantiation so
  // PROGADDR_RESET, ENABLE_IRQ, etc. on this wrapper reach the core.
  picorv32 u_picorv32 (
`else
  picorv32 #(
    .ENABLE_COUNTERS      (ENABLE_COUNTERS),
    .ENABLE_COUNTERS64    (1'b0),
    .ENABLE_REGS_16_31    (ENABLE_REGS_16_31),
    .ENABLE_REGS_DUALPORT (1'b1),
    .LATCHED_MEM_RDATA    (1'b0),
    .TWO_STAGE_SHIFT      (TWO_STAGE_SHIFT),
    .BARREL_SHIFTER       (1'b0),
    .TWO_CYCLE_COMPARE    (1'b0),
    .TWO_CYCLE_ALU        (1'b0),
    .COMPRESSED_ISA       (COMPRESSED_ISA),
    .CATCH_MISALIGN       (1'b1),
    .CATCH_ILLINSN        (1'b1),
    .ENABLE_PCPI          (ENABLE_PCPI),
    .ENABLE_MUL           (ENABLE_MUL),
    .ENABLE_FAST_MUL      (1'b0),
    .ENABLE_DIV           (ENABLE_DIV),
    .ENABLE_IRQ           (ENABLE_IRQ),
    .ENABLE_IRQ_QREGS     (1'b1),
    .ENABLE_IRQ_TIMER     (1'b0),                // PLIC handles timers externally
    .ENABLE_TRACE         (1'b0),
    .REGS_INIT_ZERO       (1'b0),
    .MASKED_IRQ           (32'h7fff_ffff),       // mask irq[30:0]; only irq[31] active
    .LATCHED_IRQ          (32'hffff_ffff),
    .PROGADDR_RESET       (PROGADDR_RESET),
    .PROGADDR_IRQ         (PROGADDR_RESET + 32'h10),
    .STACKADDR            (STACKADDR)
  ) u_picorv32 (
`endif
    .clk           (clk_i),
    .resetn        (rst_ni),
    .trap          (trap_o),
    // Native simple memory interface
    .mem_valid     (mem_valid),
    .mem_instr     (mem_instr),
    .mem_ready     (mem_ready),
    .mem_addr      (mem_addr),
    .mem_wdata     (mem_wdata),
    .mem_wstrb     (mem_wstrb),
    .mem_rdata     (mem_rdata),
    // Look-ahead variant — unused; let synthesis prune
    .mem_la_read   (),
    .mem_la_write  (),
    .mem_la_addr   (),
    .mem_la_wdata  (),
    .mem_la_wstrb  (),
    // PCPI co-processor — unused
    .pcpi_valid    (),
    .pcpi_insn     (),
    .pcpi_rs1      (),
    .pcpi_rs2      (),
    .pcpi_wr       (1'b0),
    .pcpi_rd       (32'h0),
    .pcpi_wait     (1'b0),
    .pcpi_ready    (1'b0),
    // IRQ
    .irq           (irq_lines),
    .eoi           (eoi_unused),
    // Trace — unused
    .trace_valid   (trace_valid_unused),
    .trace_data    (trace_data_unused)
  );

  // ── Bridge: PicoRV32 mem-if → TL-UL host (single-outstanding) ───────────
  //   IDLE     : drive req_i = mem_valid; on gnt_o, latch as inflight.
  //   INFLIGHT : wait for valid_o; assert mem_ready for one cycle and
  //              forward rdata. PicoRV32 holds mem_valid until mem_ready
  //              is sampled high, so we don't need a separate handshake
  //              register on the master side.
  //
  // tlul_adapter_host with MAX_REQS=1 enforces single outstanding at the
  // TL-UL side. valid_o pulses for both write completion and read
  // response, so the FSM is identical for both directions.
  typedef enum logic { S_IDLE, S_INFLIGHT } bridge_state_e;
  bridge_state_e state_q, state_d;

  logic        adapter_req;
  logic        adapter_gnt;
  logic        adapter_valid;
  logic [31:0] adapter_rdata;
  logic [ 6:0] adapter_rdata_intg;
  logic        adapter_err;

  always_comb begin
    state_d     = state_q;
    adapter_req = 1'b0;
    mem_ready   = 1'b0;
    unique case (state_q)
      S_IDLE: begin
        adapter_req = mem_valid;
        if (mem_valid && adapter_gnt) state_d = S_INFLIGHT;
      end
      S_INFLIGHT: begin
        if (adapter_valid) begin
          mem_ready = 1'b1;
          state_d   = S_IDLE;
        end
      end
      default: state_d = S_IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) state_q <= S_IDLE;
    else         state_q <= state_d;
  end

  assign mem_rdata = adapter_rdata;

  tlul_adapter_host #(
    .MAX_REQS               (1),
    // Must be 1: tlul_adapter_host's data-integrity path runs unconditionally
    // inside slaves like uart_reg_top. With EnableDataIntgGen=1 the wrapper
    // computes correct data_intg from a_data so writes don't trip d_error.
    .EnableDataIntgGen      (1),
    .EnableRspDataIntgCheck (0)
  ) u_tlul_adapter (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .req_i        (adapter_req),
    .gnt_o        (adapter_gnt),
    .addr_i       (mem_addr),
    .we_i         (|mem_wstrb),
    .wdata_i      (mem_wdata),
    .wdata_intg_i (7'h0),
    // For writes, mem_wstrb is the byte-enable; for reads picorv32 ignores
    // be entirely (it shifts/masks rdata internally) so 4'b1111 is fine.
    .be_i         ((|mem_wstrb) ? mem_wstrb : 4'b1111),
    .instr_type_i (mem_instr ? prim_mubi_pkg::MuBi4True
                              : prim_mubi_pkg::MuBi4False),
    .user_rsvd_i  ('0),
    .valid_o      (adapter_valid),
    .rdata_o      (adapter_rdata),
    .rdata_intg_o (adapter_rdata_intg),
    .err_o        (adapter_err),
    .intg_err_o   (),
    .tl_o         (tl_o),
    .tl_i         (tl_i)
  );

  // adapter_err / rdata_intg are observable as TL d_error to the slave but
  // not propagated to picorv32 (no native bus-error input). Bus errors are
  // currently silent at the CPU side; firmware sees stale rdata. If this
  // becomes a problem, route adapter_err to picorv32's irq[2] (the built-in
  // bus-error trap line) and re-enable that bit in MASKED_IRQ.
  logic _unused_err;
  assign _unused_err = adapter_err | (|adapter_rdata_intg)
                       | trace_valid_unused | (|trace_data_unused)
                       | (|eoi_unused);

endmodule
