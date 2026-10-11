# Asynchronous FIFO: Dual-Clock, Gray-Pointer Design in Verilog

A parameterized asynchronous FIFO that moves data between two unrelated clock domains (`wclk` and `rclk`). Only Gray-coded pointers cross the clock boundary, each through a two-flip-flop synchronizer, and the `full` / `empty` flags are generated directly in the Gray domain from look-ahead (`_next`) pointers. The repository also contains a testbench that drives the FIFO from two clocks of different frequency and checks the read data against a manual, array-based scoreboard.

| Property | Value |
|---|---|
| Language | Verilog (`$clog2` is used for the pointer width) |
| Default configuration | `DEPTH = 8`, `DATA_WIDTH = 8`, `PTR_WIDTH = $clog2(DEPTH) = 3` |
| Pointer width | `PTR_WIDTH + 1` bits (`[PTR_WIDTH:0]`); the extra MSB is the wrap bit |
| Capacity | All `DEPTH` entries are usable; the wrap bit, not a sacrificed slot, separates full from empty |
| Reset | Active-low, one per domain (`wrst_n`, `rrst_n`). Pointers and flags use asynchronous-assert resets; the synchronizer flops use a synchronous reset |
| Read data | Combinational read: `data_out = fifo[b_rptr[PTR_WIDTH-1:0]]`. A registered-read variant is present in `fifo_mem.v` as a commented-out block |
| Constraints | `DEPTH` is a power of two and `>= 4`. `fifo_mem` is instantiated without parameter overrides (see the `asynchronous_fifo.v` section) |
| Testbench clocks | `wclk` period 10 ns (100 MHz), `rclk` period 34 ns (about 29.4 MHz) |

---

## 1. Architecture

`N` denotes `PTR_WIDTH`. Every signal that crosses the dotted boundary is a Gray-coded pointer; no binary pointer and no flag ever crosses. Each synchronizer is clocked by the *destination* domain, which is why `sync_rptr` (clocked by `wclk`) sits on the write side of the boundary and `sync_wptr` (clocked by `rclk`) sits on the read side.

```
  WRITE DOMAIN  (wclk, wrst_n)                                READ DOMAIN  (rclk, rrst_n)
        w_en                                             :                                          r_en
          |                                              :                                            |
          |                                              :                                            |
          v                                              :                                            v
  +-------------------+                                  :                                  +-------------------+
  | wptr_handler      |                                  :    +---------------+             | rptr_handler      |
  | (wptr_h)          |             g_wptr               :    | synchronizer  | g_wptr_sync | (rptr_h)          |
  |                   |-------------------------------------->| (sync_wptr)   |------------>|                   |
  | b_wptr  (reg)     |                                  :    | clk = rclk    |             | b_rptr  (reg)     |
  | g_wptr  (reg)     |                                  :    +---------------+             | g_rptr  (reg)     |
  | full    (reg)     |             +---------------+    :                                  | empty   (reg)     |
  |                   |             | synchronizer  |    :                                  |                   |
  | b_wptr_next       | g_rptr_sync | (sync_rptr)   |    :          g_rptr                  | b_rptr_next       |
  | g_wptr_next       |<------------| clk = wclk    |<--------------------------------------| g_rptr_next       |
  | wfull             |             +---------------+    :                                  | rempty            |
  |                   |                                  :                                  |                   |
  +-------------------+                                  :                                  +-------------------+
      |           |                                                                             |       |
      |           |                                                                             |       |
      | b_wptr    | full                                                                  empty |       | b_rptr
      |           |                                                                             |       |
      |           |                                                                             |       |
      v           v                                                                             v       v
  +-------------------------------------------------------------------------------------------------------------+
  |                               fifo_mem  (fifom)     reg [DATA_WIDTH-1:0] fifo [0:DEPTH-1]                   |
  |                                                                                                             |
  | write port (wclk):  if (w_en & !full)  fifo[b_wptr[PTR_WIDTH-1:0]] <= data_in;                              |
  | read  port        :  assign data_out = fifo[b_rptr[PTR_WIDTH-1:0]];   // combinational read                 |
  |                                                                                                             |
  +-------------------------------------------------------------------------------------------------------------+
                        ^                                                       |
                        |                                                       v
                  data_in                                                  data_out
```

### Clock-domain ownership

| Signal | Driven in | Used in | Crosses via |
|---|---|---|---|
| `b_wptr` | `wclk` | `wclk` (RAM write address) | never crosses |
| `g_wptr` | `wclk` | `rclk` (empty logic) | `sync_wptr` -> `g_wptr_sync` |
| `b_rptr` | `rclk` | `rclk` (RAM read address) | never crosses |
| `g_rptr` | `rclk` | `wclk` (full logic) | `sync_rptr` -> `g_rptr_sync` |
| `full` | `wclk` | `wclk` (pointer gating, RAM write guard) | never crosses |
| `empty` | `rclk` | `rclk` (pointer gating) | never crosses |

### Top-level interface

| Port | Dir | Width | Domain | Description |
|---|---|---|---|---|
| `wclk`, `wrst_n` | in | 1 | write | Write clock, active-low reset |
| `rclk`, `rrst_n` | in | 1 | read | Read clock, active-low reset |
| `w_en` | in | 1 | write | Write request; accepted only while `full` is low |
| `r_en` | in | 1 | read | Read request; the read pointer advances only while `empty` is low |
| `data_in` | in | `DATA_WIDTH` | write | Write data |
| `data_out` | out (`wire`) | `DATA_WIDTH` | read | `fifo[b_rptr]`, a combinational read of the word at the current read pointer |
| `full` | out (`wire`) | 1 | write | Registered flag inside `wptr_handler` |
| `empty` | out (`wire`) | 1 | read | Registered flag inside `rptr_handler` |

### Flag timing

| Event | Observable effect | Latency |
|---|---|---|
| Accepted write | `empty` deasserts | 2 `rclk` edges through `sync_wptr` + 1 `rclk` edge into the `empty` register: about 3 `rclk`, plus up to one `rclk` of phase uncertainty |
| Accepted read | `full` deasserts | 2 `wclk` edges through `sync_rptr` + 1 `wclk` edge into the `full` register: about 3 `wclk`, plus up to one `wclk` of phase uncertainty |
| Write that fills the last slot | `full` asserts | On the same `wclk` edge that updates `b_wptr` (look-ahead, see `wptr_handler.v`) |
| Read that drains the last word | `empty` asserts | On the same `rclk` edge that updates `b_rptr` (look-ahead, see `rptr_handler.v`) |

Deassertion is delayed and assertion is immediate. Each side sees a slightly stale copy of the other side's pointer, which can only make the FIFO look fuller (to the writer) or emptier (to the reader) than it really is. The error is always in the safe direction.

### Build and run

All six source files are required; none of the `` `include `` lines in `asynchronous_fifo.v` is active. Icarus Verilog is shown as an example; in Vivado, add the six files as sources and set `async_fifo_TB` as the simulation top. The testbench writes `dump.vcd`.

```bash
iverilog -g2012 -Wall -o async_fifo_sim \
    fifo_mem.v synchronizer.v wptr_handler.v rptr_handler.v \
    asynchronous_fifo.v async_fifo_TB.v
vvp async_fifo_sim
gtkwave dump.vcd
```

---

## 2. Design Files

### `asynchronous_fifo.v`

**Purpose.** Structural top level. It contains no behavioural logic. It derives `PTR_WIDTH`, declares the interconnect buses, and instantiates five blocks: two `synchronizer` instances (one per crossing direction), `wptr_handler`, `rptr_handler` and `fifo_mem`.

```verilog
//`include "synchronizer.v"
//`include "wptr_handler.v"
//`include "rptr_handler.v"
//`include "fifo_mem.v"

module asynchronous_fifo #(parameter DEPTH=8, DATA_WIDTH=8) (
  input wclk, wrst_n,
  input rclk, rrst_n,
  input w_en, r_en,
  input [DATA_WIDTH-1:0] data_in,
  output wire [DATA_WIDTH-1:0] data_out,
  output wire full, empty
);
  
  parameter PTR_WIDTH = $clog2(DEPTH);
 
  wire [PTR_WIDTH:0] g_wptr_sync, g_rptr_sync;
  wire [PTR_WIDTH:0] b_wptr, b_rptr;
  wire [PTR_WIDTH:0] g_wptr, g_rptr;

//  wire [PTR_WIDTH-1:0] waddr, raddr;

  synchronizer #(PTR_WIDTH) sync_wptr (rclk, rrst_n, g_wptr, g_wptr_sync); //write pointer to read clock domain
  synchronizer #(PTR_WIDTH) sync_rptr (wclk, wrst_n, g_rptr, g_rptr_sync); //read pointer to write clock domain 
  
  wptr_handler #(PTR_WIDTH) wptr_h(wclk, wrst_n, w_en,g_rptr_sync,b_wptr,g_wptr,full);
  rptr_handler #(PTR_WIDTH) rptr_h(rclk, rrst_n, r_en,g_wptr_sync,b_rptr,g_rptr,empty);
  fifo_mem fifom(wclk, w_en, rclk, r_en,b_wptr, b_rptr, data_in,full,empty, data_out);

endmodule
```

**Instance map.** All connections are positional, so the order of the actual arguments must match the port order of each submodule exactly.

| Instance | Module | Parameters | Positional connections (actual -> formal port) |
|---|---|---|---|
| `sync_wptr` | `synchronizer` | `#(PTR_WIDTH)` | `rclk` -> `clk`, `rrst_n` -> `rst_n`, `g_wptr` -> `d_in`, `g_wptr_sync` -> `d_out` |
| `sync_rptr` | `synchronizer` | `#(PTR_WIDTH)` | `wclk` -> `clk`, `wrst_n` -> `rst_n`, `g_rptr` -> `d_in`, `g_rptr_sync` -> `d_out` |
| `wptr_h` | `wptr_handler` | `#(PTR_WIDTH)` | `wclk`, `wrst_n`, `w_en`, `g_rptr_sync`, `b_wptr`, `g_wptr`, `full` |
| `rptr_h` | `rptr_handler` | `#(PTR_WIDTH)` | `rclk`, `rrst_n`, `r_en`, `g_wptr_sync`, `b_rptr`, `g_rptr`, `empty` |
| `fifom` | `fifo_mem` | none (defaults) | `wclk`, `w_en`, `rclk`, `r_en`, `b_wptr`, `b_rptr`, `data_in`, `full`, `empty`, `data_out` |

**Structural logic.**

- `parameter PTR_WIDTH = $clog2(DEPTH)` is the number of memory address bits. Every pointer bus is declared `[PTR_WIDTH:0]`, one bit wider than the address, so the pointer sequence has `2 x DEPTH` states and the wrap bit disambiguates full from empty.
- `sync_wptr` is clocked by `rclk` / `rrst_n` and carries `g_wptr` into the read domain. `sync_rptr` is clocked by `wclk` / `wrst_n` and carries `g_rptr` into the write domain. The `#(PTR_WIDTH)` override sets the synchronizer's `WIDTH`, so its bus is `[PTR_WIDTH:0]`, matching the pointers.
- `wptr_h` receives `g_rptr_sync`, the only read-domain information the write side ever sees, and produces `b_wptr`, `g_wptr` and `full`. `rptr_h` is its mirror image and produces `b_rptr`, `g_rptr` and `empty`.
- `fifom` receives the *binary* pointers, which never leave their own domains, plus the flags, which it uses as guards.

**Why `full`, `empty` and `data_out` are `wire`, not `reg`, at this level.** In Verilog, an output port of an instantiated module drives whatever it is connected to through a *continuous* connection. A `reg` is a variable and may only be assigned procedurally from an `always` or `initial` block. If the top level declared `output reg full, empty`, then `wptr_h`'s `full` port and `rptr_h`'s `empty` port would be continuously driving a variable. Vivado rejects that with the error `concurrent assignment to a non-net is not permitted`, because only nets (`wire`) can be the target of a continuous driver. The declaration `output wire full, empty` in this file is therefore not stylistic; it is what makes the port connections legal.

The storage element behind `full` is still a flip-flop. It lives inside `wptr_handler`, where `full` is declared `output reg` and assigned in an `always @(posedge wclk or negedge wrst_n)` block. That is legal because the assignment is procedural and local to the module that owns the register. Seen from the parent, `full` is just the net on which that flop's output appears. The same rule holds for `empty` (a register inside `rptr_handler`) and for `data_out` (driven by the continuous `assign` in `fifo_mem`, which is why it is `output wire` there). It also explains why all the interconnect buses (`g_wptr_sync`, `b_wptr`, `g_wptr`, and so on) are declared `wire`, and why the testbench declares `full`, `empty` and `data_out` as `wire`: in every case a submodule is the driver.

**Parameter constraints.**

- `DEPTH` must be a power of two. The pointer must wrap naturally, and the Gray sequence is cyclic (its last code differs from the first by one bit) only when the number of states, `2 x DEPTH`, is a power of two.
- `DEPTH >= 4`, because the full comparison in `wptr_handler` slices `g_rptr_sync[PTR_WIDTH-2:0]`, which requires `PTR_WIDTH >= 2`.
- `fifom` is instantiated as `fifo_mem fifom(...)` with no `#(...)` override, so `fifo_mem` always runs with its own defaults (`DEPTH=8`, `DATA_WIDTH=8`, `PTR_WIDTH=3`). The synchronizers and handlers follow `PTR_WIDTH` (derived from the top-level `DEPTH`) and the top-level `data_in` / `data_out` follow `DATA_WIDTH`, but the memory does not. For any configuration other than the 8 x 8 default, the instantiation needs `#(DEPTH, DATA_WIDTH, PTR_WIDTH)` (positional order of the `fifo_mem` parameters) or the equivalent named override; otherwise the pointer ports and the array dimensions disagree with the rest of the design.
- The commented-out `` `include `` lines and the commented `waddr, raddr` declaration have no effect; the file list in the build step is what pulls in the other modules.

---

### `synchronizer.v`

**Purpose.** The only place in the design where a signal changes clock domain. It is a two-stage flip-flop synchronizer clocked by the destination clock, instantiated twice (`sync_wptr`, `sync_rptr`), each time carrying one Gray-coded pointer.

```verilog
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.10.2026 19:52:28
// Design Name: 
// Module Name: synchronizer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module synchronizer #(parameter WIDTH=3) (input clk, rst_n, [WIDTH:0] d_in, output reg [WIDTH:0] d_out);
  reg [WIDTH:0] q1;
  always@(posedge clk) begin
    if(!rst_n) begin
      q1 <= 0;
      d_out <= 0;
    end
    else begin
      q1 <= d_in;
      d_out <= q1;
    end
  end
endmodule
```

**Structure.** `q1` is the first stage and `d_out` (declared `output reg`) is the second stage. Both are clocked by `clk`, the destination clock, and both are cleared by `rst_n`. `d_in` inherits the `input` direction from the port list, and its width is `WIDTH+1`, matching the pointer bus. Because `d_out` is itself the second flop, the module can legally declare it `output reg`: the assignment is procedural and inside the module that owns it, and the parent connects it to a `wire`. The reset in this module is synchronous (the sensitivity list contains only `posedge clk`), unlike the handlers, whose pointer and flag registers reset asynchronously.

**Why a raw, unsynchronized signal must never cross a clock boundary.** Two independent failure mechanisms apply, and they compound.

*1. Metastability from setup/hold violation.* A flip-flop produces a clean 0 or 1 only if its data input is stable for the setup time `t_su` before and the hold time `t_h` after the active clock edge. The source pointer changes on `wclk` (or `rclk`), which has no phase relationship to the capturing clock, so sooner or later a pointer bit changes inside the capture window of the destination flop. The flop's internal latch, a pair of cross-coupled inverters, is then left near its unstable equilibrium, with an internal node at an intermediate voltage between `V_IL` and `V_IH`. It eventually resolves to 0 or 1, but after a random time that follows an exponential regeneration law with time constant `tau`. While it hovers:

- The intermediate level propagates into downstream gates. Gates with different input thresholds interpret it differently, so one fan-out branch can see `1` while another sees `0`, and the design is briefly in two states at once.
- Even if the level resolves, it may do so late enough to violate setup time on the *next* register, extending the problem downstream.
- Which value the flop settles to, old or new, is unpredictable.

The reliability of a synchronizer is quantified as `MTBF = e^(t_r / tau) / (T_0 x f_clk x f_data)`, where `t_r` is the time allowed for resolution. Metastability cannot be eliminated, only made exponentially rare by increasing `t_r`.

*2. Multi-bit convergence glitches.* Even ignoring metastability, a multi-bit bus is not sampled atomically. Each bit has its own routing delay, so on a given destination edge each bit independently captures either its old or its new value. A binary counter makes this catastrophic, because one increment can flip many bits. A 4-bit pointer going from `0111` (7) to `1000` (8) flips all four bits, so the capture flops can legally observe any of the 16 combinations, for example `1111`, `0000`, `1011` or `0100`. None of those is 7 or 8. The read side could then compare against a write pointer that is nowhere near the real one, so `empty` deasserts while the RAM holds nothing new (underflow), or the write side sees a corrupted read pointer and overwrites unread data (overflow). The error is transient and pattern dependent, which is why zero-delay RTL simulation never exposes it.

**How Gray coding removes the multi-bit problem.** `g_wptr` and `g_rptr` are Gray codes, in which consecutive values differ in exactly one bit (7 -> 8 is `0100` -> `1100`). A pointer moves by at most one count per source clock, so only one bit of the bus is ever in flight. If the capture flop catches that bit "early" or "late", the sampled bus is either the old pointer or the new pointer, never a third value. If the destination clock is slower and several increments happen between two samples, each sample is still a pointer value that really existed at some instant; it is merely stale, which the flag logic tolerates. Gray coding and the synchronizer are two halves of one solution: the synchronizer guarantees that each captured bit is a clean logic level, and the Gray code guarantees that "old or new" are both legal answers. This is also why the handlers send the *registered* `g_wptr` / `g_rptr` into the synchronizers and never the combinational `_next` values: a glitching combinational signal on a path that must carry exactly one transition would defeat the encoding.

**How the two-flop structure isolates the domains.** `q1` is allowed to go metastable; that is its job. Its output is not used by any logic and feeds only the D input of the second stage, so `q1` has essentially one full destination clock period (minus clock-to-Q and the setup time of `d_out`) to decay to a stable level before the second flop samples it. That makes `t_r` approximately `T_clk`, and because the failure probability falls exponentially with `t_r`, a full period of settling lowers the chance that the second stage ever samples an unresolved level by many orders of magnitude. `d_out` is therefore the first signal that is safe to use in the destination domain, and all destination-domain logic (`rempty`, `wfull`) reads only that second-stage output. The synchronizer does not make the *value* deterministic; it makes it a clean, possibly one-cycle-stale, logic level.

**Implementation notes.**

- Latency: two destination clock edges through the module, plus one more edge into the flag register, which is why flag deassertion is delayed by about three destination clocks (see the flag timing table).
- This source has no `(* ASYNC_REG = "TRUE" *)` attribute on `q1` / `d_out`. In a Vivado flow, adding it to both registers tells the tools to place them in the same slice and to keep them out of retiming or shift-register inference, which preserves the settling time. Timing constraints for these paths (a datapath-only max-delay from `g_wptr` / `g_rptr` into `q1`, and a bus-skew limit on the Gray buses so the one-bit-change property survives routing) are also normally applied in the XDC.

---

### `wptr_handler.v`

**Purpose.** Everything the write domain owns: the binary write pointer that addresses memory, its Gray-coded twin that is sent to the read domain, and the registered `full` flag.

```verilog
module wptr_handler #(parameter PTR_WIDTH=3) (
  input wclk, wrst_n, w_en,
  input [PTR_WIDTH:0] g_rptr_sync,
  output reg [PTR_WIDTH:0] b_wptr, g_wptr,
  output reg full
);

  wire [PTR_WIDTH:0] b_wptr_next;
  wire [PTR_WIDTH:0] g_wptr_next;
   
  reg wrap_around;
  wire wfull;
  
  assign b_wptr_next = b_wptr+(w_en & !full);
  assign g_wptr_next = (b_wptr_next >>1)^b_wptr_next;
  
  always@(posedge wclk or negedge wrst_n) begin
    if(!wrst_n) begin
      b_wptr <= 0; // set default value
      g_wptr <= 0;
    end
    else begin
      b_wptr <= b_wptr_next; // incr binary write pointer
      g_wptr <= g_wptr_next; // incr gray write pointer
    end
  end
  
  always@(posedge wclk or negedge wrst_n) begin
    if(!wrst_n) full <= 0;
    else        full <= wfull;
  end

  assign wfull = (g_wptr_next == {~g_rptr_sync[PTR_WIDTH:PTR_WIDTH-1], g_rptr_sync[PTR_WIDTH-2:0]});

endmodule
```

**Pointer datapath.**

- `b_wptr_next = b_wptr + (w_en & !full)` increments only on an accepted write. Gating with `!full` means the pointer cannot run past a full FIFO even if the producer holds `w_en` high.
- `g_wptr_next = (b_wptr_next >> 1) ^ b_wptr_next` is the binary-to-Gray conversion: one XOR per bit, no carry chain.
- `b_wptr` and `g_wptr` are registered together in the first `always` block, so the Gray pointer is always the Gray encoding of the binary pointer. `g_wptr_next` is the combinational value at the D input of the `g_wptr` register; after the clock edge the register holds that value, so `g_wptr_next` and `g_wptr` are the same quantity one cycle apart.
- `reg wrap_around;` is declared but never used. It has no effect on the design and can be removed.
- Reset clears both pointers and `full` to 0 (asynchronous assert on `wrst_n`).

**Full-flag generation without Gray-to-binary conversion.**

The FIFO is full when the write pointer has advanced exactly `DEPTH` positions beyond the read pointer. With the extra wrap bit, that means the two binary pointers are identical in every bit except the MSB: `b_wptr == b_rptr ^ (1 << N)`.

The direct way to test this from a synchronized *Gray* read pointer is to convert it back to binary first (`bin[i] = ^gray[N:i]`), then compare "MSB differs, all other bits equal". That prefix-XOR network is a serial chain of up to `N` XORs (logic depth `N`), or up to `N(N+1)/2` XORs if flattened to cut the depth, so it costs area or delay and both grow with the pointer width, all on the path into the `full` register.

This design never leaves the Gray domain. Gray encoding `g(b) = b ^ (b >> 1)` is linear over GF(2), so `g(x ^ y) = g(x) ^ g(y)`. Apply it to the binary full condition with the MSB mask `M = 1 << N`:

```
g(b_wptr) = g(b_rptr ^ M)
          = g(b_rptr) ^ g(M)
          = g(b_rptr) ^ (M ^ (M >> 1))
          = g(b_rptr) ^ {1, 1, 0, ..., 0}        <- bits N and N-1 set
```

So **full means the Gray write pointer equals the Gray read pointer with its top two bits inverted and every other bit unchanged.** That is exactly the expression in the RTL:

```
{~g_rptr_sync[PTR_WIDTH:PTR_WIDTH-1], g_rptr_sync[PTR_WIDTH-2:0]}
```

Why two bits and not one: the Gray MSB equals the binary MSB, but the next Gray bit is `b[N] ^ b[N-1]`. Flipping `b[N]` in binary therefore flips *both* Gray bit `N` and Gray bit `N-1`, while bits `N-2:0` are unaffected. Worked example for `DEPTH = 8` (4-bit pointers):

| `b_wptr` | `g_wptr` | `b_rptr` | `g_rptr` | `g_rptr` with top 2 bits inverted | Full? |
|---|---|---|---|---|---|
| 8 (`1000`) | `1100` | 0 (`0000`) | `0000` | `1100` | yes |
| 11 (`1011`) | `1110` | 3 (`0011`) | `0010` | `1110` | yes |
| 9 (`1001`) | `1101` | 0 (`0000`) | `0000` | `1100` | no |

The hardware cost is two inverters and an `(N+1)`-bit equality comparator (one XNOR per bit and an AND reduction). There is no Gray-to-binary network, so the logic is smaller and the longest path (inverter, XNOR, AND-reduce) does not grow with an XOR chain. The comparison uses `g_rptr_sync`, which is slightly stale, so `full` may stay asserted a few cycles after the reader has freed a slot but can never deassert early.

**Why the flag is computed from `g_wptr_next`, not `g_wptr`.**

`full` is a registered flag (`full <= wfull`), and a registered flag always trails the logic that feeds it by one clock. If `wfull` were computed from the *current* `g_wptr`, the flag would describe the FIFO as it was one write ago. Timeline for `DEPTH = 8` holding 7 words, no reads in progress (`g_rptr_sync` = `0000`, so the full pattern is `1100`), `w_en` held high:

| Edge | This design (`wfull` from `g_wptr_next`) | Hypothetical flag from `g_wptr` |
|---|---|---|
| `k` (write #8 accepted, `b_wptr` becomes 8) | `g_wptr_next` = `1100` matches, so `full <= 1` on this very edge | `g_wptr` is still `gray(7)` = `0100`, no match, so `full <= 0` |
| `k+1` (`w_en` still high) | `full` is 1, so `b_wptr_next` holds. Exactly 8 words stored | `full` is still 0, so **write #9 is accepted into `fifo[0]` and overwrites the oldest unread word**; `b_wptr` becomes 9. The flag register now loads 1, because `g_wptr` is `1100` |
| `k+2` | `full` stays 1 | `g_wptr` has moved to `gray(9)` = `1101`, which no longer matches, so `full <= 0`. The flag was high for one cycle and drops while the FIFO is still full |

The look-ahead flag is therefore a correctness requirement, not an optimization. The one-cycle window between "pointer updated" and "flag updated" is long enough for another write to slip through, after which the pointer has moved past the full-detection point and the FIFO keeps overflowing. With look-ahead, the flag register loads `1` on the same edge that accepts the last write, so the next write is already blocked. The same `!full` signal also guards the RAM write in `fifo_mem`, so the memory and the pointer stop on the same edge.

---

### `rptr_handler.v`

**Purpose.** The mirror of `wptr_handler.v` for the read domain: the binary read pointer that addresses memory, the Gray read pointer sent to the write domain, and the registered `empty` flag.

```verilog
module rptr_handler #(parameter PTR_WIDTH=3) (
  input rclk, rrst_n, r_en,
  input [PTR_WIDTH:0] g_wptr_sync,
  output reg [PTR_WIDTH:0] b_rptr, g_rptr,
  output reg empty
);

 
  wire [PTR_WIDTH:0] b_rptr_next;
  wire [PTR_WIDTH:0] g_rptr_next;
  wire               rempty;

  assign b_rptr_next = b_rptr + (r_en & !empty);
  assign g_rptr_next = (b_rptr_next >> 1) ^ b_rptr_next;
  assign rempty      = (g_wptr_sync == g_rptr_next);
  
  // Pointer Registers
  always @(posedge rclk or negedge rrst_n) begin
    if(!rrst_n) begin
      b_rptr <= 0;
      g_rptr <= 0;
    end
    else begin
      b_rptr <= b_rptr_next;
      g_rptr <= g_rptr_next;
    end
  end
  
  // Flag Register
  always @(posedge rclk or negedge rrst_n) begin
    if(!rrst_n) empty <= 1'b1;
    else        empty <= rempty;
  end

endmodule
```

**Pointer datapath.**

- `b_rptr_next = b_rptr + (r_en & !empty)` advances only when a read is requested and the FIFO is not empty. `g_rptr_next` is the same one-XOR-per-bit Gray conversion used on the write side.
- `b_rptr` and `g_rptr` are registered together in the "Pointer Registers" block; `empty` is registered separately in the "Flag Register" block. The registered `g_rptr` is what goes to `sync_rptr` and then to the write domain.

**Empty condition.** The FIFO is empty when the read pointer has caught up with the write pointer, i.e. all `N+1` bits are equal, wrap bit included. A Gray code is a bijection (distinct binary values map to distinct Gray values), so binary equality is exactly Gray equality, and `rempty = (g_wptr_sync == g_rptr_next)` needs neither a conversion nor any bit inversion. This is the asymmetry with `full`: equality is preserved by the encoding, but "differ by exactly `DEPTH`" is not, which is why `full` needs the top-two-bit inversion derived in `wptr_handler.v`.

**Why look-ahead (`g_rptr_next`) is used here too.** `empty <= rempty` is a registered flag, so if `rempty` used the current `g_rptr` the flag would trail the pointer by one clock. Consider the FIFO holding exactly one word, with `g_wptr_sync` already reflecting that write and `r_en` held high:

| Edge | This design (`rempty` from `g_rptr_next`) | Hypothetical flag from `g_rptr` |
|---|---|---|
| `k` (the read that drains the last word) | `g_rptr_next` equals `g_wptr_sync`, so `empty <= 1` on this edge | `g_rptr` is still the old value, no match, so `empty <= 0` |
| `k+1` (`r_en` still high) | `empty` is 1, the pointer is blocked | `empty` is still 0, so a **second read is accepted**, `b_rptr` moves one past `b_wptr` |

After that second read, the read pointer is ahead of the write pointer, which in modulo-`2 x DEPTH` arithmetic looks like a FIFO holding `2 x DEPTH - 1` words. The reader consumes stale RAM contents as valid data (underflow) and the flags no longer describe reality. With look-ahead, `empty` asserts on the very edge that drains the last word and the next pop is already blocked.

**Reset value and staleness.**

- `empty` resets to `1'b1`. After reset the FIFO holds nothing, and `empty` must be asserted from the first cycle so no read can be accepted before the first write has crossed the synchronizer.
- `g_wptr_sync` lags the real write pointer by about three `rclk` cycles, so it is never ahead of the true pointer. `empty` can therefore stay asserted briefly after data has been written (a few lost cycles) but can never deassert before the data is in the RAM.

---

### `fifo_mem.v`

**Purpose.** The shared storage array: a simple dual-port memory with a write port on `wclk` and an address-driven read on the read pointer.

```verilog
module fifo_mem #(parameter DEPTH=8, DATA_WIDTH=8, PTR_WIDTH=3) (
  input wclk, w_en, rclk, r_en,
  input [PTR_WIDTH:0] b_wptr, b_rptr,
  input [DATA_WIDTH-1:0] data_in,
  input full, empty,
  output wire [DATA_WIDTH-1:0] data_out
);
  reg [DATA_WIDTH-1:0] fifo[0:DEPTH-1];
  
  always@(posedge wclk) begin
    if(w_en & !full) begin
      fifo[b_wptr[PTR_WIDTH-1:0]] <= data_in;
    end
  end
  /*
  always@(posedge rclk) begin
    if(r_en & !empty) begin
      data_out <= fifo[b_rptr[PTR_WIDTH-1:0]];
    end
  end
  */
  assign data_out = fifo[b_rptr[PTR_WIDTH-1:0]];
endmodule
```

**Write port.** On each `wclk` edge, `if (w_en & !full)` stores `data_in` at `fifo[b_wptr[PTR_WIDTH-1:0]]`. Only the lower `PTR_WIDTH` bits of the binary pointer address the array; the MSB is the wrap bit, used by the flag logic and ignored by the RAM. The `!full` guard duplicates the pointer gating in `wptr_handler`, so the RAM and the pointer stop accepting on the same edge. The array has no reset.

**Read path in the current source.** The active read is `assign data_out = fifo[b_rptr[PTR_WIDTH-1:0]];`, a *combinational* read, which is why `data_out` is declared `output wire` here. The registered-read `always @(posedge rclk)` block, which would load `data_out` only when `r_en & !empty`, is commented out. Consequences:

- `rclk`, `r_en` and `empty` are present in the port list but unused in the active code; they are kept so the commented-out registered-read variant can be restored without changing the interface.
- `data_out` always shows the word at the current read pointer. While `empty` is low, that is the word the next accepted read will consume (a "show-ahead" behaviour). When the read pointer advances on an accepted read, `data_out` moves to the next word right after that `rclk` edge, so a consumer must capture `data_out` on the same edge on which it asserts `r_en` and sees `empty` low.
- When `empty` is high, `data_out` is whatever happens to be stored at `b_rptr` (possibly never written) and must not be treated as valid.
- An asynchronous-read array maps to distributed (LUT) RAM on an FPGA. The registered-read variant, with a one-clock output register, is the form that maps to block RAM.

**Why sharing the array between two clocks is not a CDC hazard.** The data word and the pointer that publishes it are updated on the same `wclk` edge. The pointer then needs two `rclk` edges to cross `sync_wptr` and one more to reach `empty`, so by the time the read side sees `empty` fall, the RAM word has been stable for several cycles. In the other direction, a slot can be overwritten only after the read pointer that frees it has crossed `sync_rptr` and let `full` deassert. Data never races its own pointer.

---

### `async_fifo_TB.v`

**Purpose.** A testbench that drives the FIFO from two unrelated clocks, records every word it writes in a manual array scoreboard, and compares each word read back against the scoreboard entry, printing a `PASS` or `ERROR` line per compare.

```verilog
`timescale 1ns / 1ps

module async_fifo_TB;

  parameter DATA_WIDTH = 8;
  parameter SCOREBOARD_DEPTH = 128;

  // Output Wires
  wire [DATA_WIDTH-1:0] data_out;
  wire full;
  wire empty;
  
  // Input Regs
  reg [DATA_WIDTH-1:0] data_in;
  reg w_en, wclk, wrst_n;
  reg r_en, rclk, rrst_n;

  // Pure Verilog Scoreboard Array (Replaces SystemVerilog Queue)
  reg [DATA_WIDTH-1:0] scoreboard_mem [0:SCOREBOARD_DEPTH-1];
  reg [7:0] sb_wr_ptr;
  reg [7:0] sb_rd_ptr;
  reg [DATA_WIDTH-1:0] wdata;
  
  // Loop Iteration Variables (Declared globally for Verilog)
  integer i, j;

  // Instantiate Design Under Test (DUT)
  asynchronous_fifo as_fifo (
    .wclk(wclk), .wrst_n(wrst_n),
    .rclk(rclk), .rrst_n(rrst_n),
    .w_en(w_en), .r_en(r_en),
    .data_in(data_in), .data_out(data_out),
    .full(full), .empty(empty)
  );

  // Clock Generation
  always #5   wclk = ~wclk; // 10ns cycle period
  always #17  rclk = ~rclk; // 34ns cycle period
  
  //------------------------------------------------
  // WRITE STIMULUS DOMAIN
  //------------------------------------------------
  initial begin
    wclk = 1'b0; wrst_n = 1'b0;
    w_en = 1'b0;
    data_in = 0;
    sb_wr_ptr = 0;
    
    repeat(10) @(posedge wclk);
    wrst_n = 1'b1;

    for (j=0; j<2; j=j+1) begin
      for (i=0; i<30; i=i+1) begin
        @(posedge wclk);
        if (!full) begin
          w_en <= (i%2 == 0) ? 1'b1 : 1'b0;
          if (i%2 == 0) begin
            // Verilog random data generation
            data_in <= $random; 
            scoreboard_mem[sb_wr_ptr] = $random; // Store directly for matching
            sb_wr_ptr = sb_wr_ptr + 1;
          end
        end else begin
          w_en <= 1'b0;
        end
      end
      #50;
    end
  end

  //------------------------------------------------
  // READ STIMULUS & VERIFICATION DOMAIN
  //------------------------------------------------
  initial begin
    rclk = 1'b0; rrst_n = 1'b0;
    r_en = 1'b0;
    sb_rd_ptr = 0;

    repeat(20) @(posedge rclk);
    rrst_n = 1'b1;

    for (j=0; j<2; j=j+1) begin
      for (i=0; i<30; i=i+1) begin
        @(posedge rclk);
        if (!empty) begin
          r_en <= (i%2 == 0) ? 1'b1 : 1'b0;
          
          if (i%2 == 0) begin
            // Fetch validation target data from memory array
            wdata = scoreboard_mem[sb_rd_ptr];
            sb_rd_ptr = sb_rd_ptr + 1;
            
            // Wait 1 read clock cycle for RAM array lookup latency
            @(posedge rclk); 
            #1; // Delay past hold time window
            
            if (data_out !== wdata) begin
              $display("ERROR at %0t ns: Expected = %h, Got = %h", $time, wdata, data_out);
            end else begin
              $display("PASS at %0t ns: Read data successfully matches written data (%h)", $time, data_out);
            end
          end
        end else begin
          r_en <= 1'b0;
        end
      end
      #50;
    end

    $display("[TEST COMPLETE] Validation sequence ended.");
    $finish;
  end
  
  // Waveform Dump Configuration
  initial begin 
    $dumpfile("dump.vcd"); 
    $dumpvars(0, async_fifo_TB);
  end

endmodule
```

**Structure and clocks.**

- The DUT is instantiated as `as_fifo` with named port connections and default parameters (`DEPTH = 8`, `DATA_WIDTH = 8`).
- `always #5 wclk = ~wclk;` gives a 10 ns write clock (100 MHz). `always #17 rclk = ~rclk;` gives a 34 ns read clock (about 29.4 MHz). The write side is roughly 3.4 times faster, so a continuous writer outruns the reader and exercises `full`.
- `full`, `empty` and `data_out` are declared `wire` because the DUT drives them; `wclk`, `rclk`, `w_en`, `r_en`, `data_in` and the resets are `reg` because the testbench assigns them procedurally. This is the same net-versus-variable rule that makes `full` and `empty` `wire` in `asynchronous_fifo.v`.
- The initial blocks hold each domain in reset for a different number of its own cycles (10 `wclk` cycles, 20 `rclk` cycles) before releasing it, so the two resets are released independently.

**Write stimulus.** Two passes of 30 iterations. On even `i` with `full` low, `w_en <= 1` and `data_in` is updated, and a scoreboard entry is stored at `scoreboard_mem[sb_wr_ptr]` before `sb_wr_ptr` is incremented. On odd `i`, `w_en <= 0`. The result is one write pulse every two `wclk` cycles, 15 per pass. If `full` is high the iteration only drives `w_en <= 0`. `w_en` and `data_in` are driven with non-blocking assignments, so the DUT samples pre-edge values and those inputs cannot race the design.

**Why a manual array scoreboard (`scoreboard_mem`, `sb_wr_ptr`, `sb_rd_ptr`).**

- *Golden reference independent of the DUT.* `scoreboard_mem[0:SCOREBOARD_DEPTH-1]` is indexed by plain counters. It shares no Gray-pointer arithmetic, wrap-bit logic or flag logic with the FIFO, so a bug in the design cannot also corrupt the expected data.
- *Ordering check.* A FIFO's contract is ordering. `sb_wr_ptr` marks the next free scoreboard slot and `sb_rd_ptr` marks the next entry to be consumed, so the Nth word read from the FIFO is checked against the Nth word written. Loss, duplication and reordering all show up as mismatches.
- *Fill-level visibility.* `sb_wr_ptr - sb_rd_ptr` is the true occupancy, which can be placed in the waveform beside `full` and `empty` when debugging flags.
- *Portability.* The comment in the file notes that the array replaces a SystemVerilog queue, so the testbench runs in any plain-Verilog simulator. The cost is that the indices are 8-bit (`reg [7:0]`) and the array has 128 entries, so a run must store fewer than 128 entries (the 8-bit index would otherwise exceed the array). This stimulus stores at most 30.

**Read stimulus and the one-clock delay before `data_out` is read.** Also two passes of 30 iterations. On even `i` with `empty` low, the read side:

1. drives `r_en <= 1`, fetches `wdata = scoreboard_mem[sb_rd_ptr]` and increments `sb_rd_ptr`;
2. executes `@(posedge rclk); #1;`, an explicit one-`rclk` wait followed by 1 ns;
3. compares `data_out` with `wdata` using `!==`, so `X` and `Z` also count as mismatches, and prints `PASS` or `ERROR`.

The wait in step 2 is there to model memory read latency. In a registered-read memory (the commented-out block in `fifo_mem.v`), a read accepted on `rclk` edge `n` loads `data_out` by non-blocking assignment at that edge, so the word is observable only after the edge. Comparing in the same time step as the accepting edge would sample the *previous* read's value (or `X` after reset) and flag a correct DUT as failing. Waiting for the next `rclk` edge and then 1 ns lets all non-blocking updates from that edge settle (the 1 ns is the file's "delay past the hold-time window"), and the sampled value is then stable.

**Alignment between this testbench and the current RTL.** Reading the code as it stands, four points do not line up and will affect what the testbench reports. They are worth fixing before relying on the `PASS` / `ERROR` output:

- *Different random values in `data_in` and the scoreboard.* The write loop calls `$random` twice: `data_in <= $random;` and `scoreboard_mem[sb_wr_ptr] = $random;`. Each call returns the next number in the sequence, so the scoreboard holds a different value than the one written into the FIFO. Using a single `$random` call stored in a temporary (the declared `wdata` is read-side only, so a separate variable is cleaner) and assigning it to both places removes this mismatch.
- *`r_en` stays high for two `rclk` edges per scoreboard entry.* `r_en <= 1` is issued on the even iteration, but the inner `@(posedge rclk)` consumes an extra edge before the odd iteration clears it. `rptr_handler` therefore samples `r_en = 1` on two consecutive edges and can advance `b_rptr` twice for every single `sb_rd_ptr` increment, so the scoreboard and the DUT drift apart.
- *The wait matches a registered read, but the active read is combinational.* With `assign data_out = fifo[b_rptr[...]]`, the accepting edge has already moved `b_rptr` by the time the `#1` expires, so the sampled `data_out` is the word *after* the one just read. Either restore the registered-read block in `fifo_mem.v` (which the one-clock delay was written for) or, for the combinational read, compare `data_out` at the accepting edge, before the pointer moves.
- *The testbench never holds `w_en` high through a sustained full condition.* The write loop checks `!full` before every pulse, so the FIFO's own overflow guard (`w_en & !full` inside `wptr_handler` and `fifo_mem`) is at most touched for a single edge by this stimulus. Holding `w_en` high while `full` is asserted, and likewise `r_en` while `empty` is asserted, is what proves those guards.

I could not run a simulator when preparing this README, so the findings above are from reading the source; confirm them in your own simulation. The run ends with `[TEST COMPLETE] Validation sequence ended.` followed by `$finish`; the testbench has no summary counter and no watchdog, so scan the log for `ERROR` lines.
