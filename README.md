# Asynchronous FIFO: Dual-Clock, Gray-Pointer Design in Verilog-2001

A parameterized asynchronous FIFO that moves data between two unrelated clock domains (`wclk` and `rclk`) without loss, duplication or corruption. Only Gray-coded pointers cross the clock boundary, each through a two-flip-flop synchronizer, and the `full` / `empty` flags are generated directly in the Gray domain from look-ahead pointers. The repository includes a self-checking, scoreboard-based testbench that exercises overflow, underflow, wrap-around and random traffic at unrelated clock ratios.

| Property | Value |
|---|---|
| Language | Verilog-2001 (`$clog2` from IEEE 1364-2005); no SystemVerilog constructs |
| Storage | `DEPTH` x `DATA_WIDTH`, all `DEPTH` entries usable (no sacrificed slot) |
| Pointer width | `PTR_WIDTH + 1` bits, where `PTR_WIDTH = $clog2(DEPTH)`; the extra MSB is the wrap bit |
| Reset | Independent, asynchronous-assert, active-low per domain (`wrst_n`, `rrst_n`) |
| Read latency | `data_out` is valid one `rclk` after an accepted read (registered RAM output) |
| Constraint | `DEPTH` must be a power of two and `>= 4` |
| Verified with | Icarus Verilog 12 (`-g2005 -Wall`), 2567 words compared, 0 errors |

---

## 1. Architecture

`N` below denotes `PTR_WIDTH`. Every signal on a boundary arrow between the two domains is a Gray-coded pointer; no binary pointer and no flag ever crosses.

```
          WRITE DOMAIN (wclk, wrst_n)                                   READ DOMAIN (rclk, rrst_n)
                                                              :
         +---------------+  g_wptr (Gray)                     :    +--------------+g_wptr_sync  +---------------+
 w_en -->| wptr_handler  |------------------------------------:--->| synchronizer |------------>| rptr_handler  |<-- r_en
         | b_wptr g_wptr |                                    :    | (rclk, 2 FF) |             | b_rptr g_rptr |
         | full          |                                    :    +--------------+             | empty         |
         |               | g_rptr_sync  +--------------+      :       g_rptr (Gray)             |               |
         |               |<-------------| synchronizer |<-----:---------------------------------|               |
         |               |              | (wclk, 2 FF) |      :                                 |               |
         +---------------+              +--------------+      :                                 +---------------+
              |                                               :                                         |
              | b_wptr[N-1:0]                                 :                         b_rptr[N-1:0]   |
              |                                                                                         |
              v                                                                                         v
         +------------------------------------------------------------------------------------------------------+
         | fifo_mem  (2^N x DATA_WIDTH dual-clock RAM)                                                          |
data_in->| write port: wclk, guard (w_en & ~full)                   read port: rclk, guard (r_en & ~empty)      |--> data_out
         |                                                                                                      |
         +------------------------------------------------------------------------------------------------------+
```

Clock-domain ownership:

| Signal | Written in | Consumed in | Crosses via |
|---|---|---|---|
| `b_wptr` | `wclk` | `wclk` (RAM write address) | never crosses |
| `g_wptr` | `wclk` | `rclk` (empty logic) | `synchronizer` -> `g_wptr_sync` |
| `b_rptr` | `rclk` | `rclk` (RAM read address) | never crosses |
| `g_rptr` | `rclk` | `wclk` (full logic) | `synchronizer` -> `g_rptr_sync` |
| `full` | `wclk` | `wclk` | never crosses |
| `empty` | `rclk` | `rclk` | never crosses |

### Repository layout

```
.
├── README.md
├── asynchronous_fifo.v     top level, structural only
├── synchronizer.v          2-FF multi-bit synchronizer (single-bit-change buses only)
├── wptr_handler.v          write pointer, Gray encode, full flag
├── rptr_handler.v          read pointer, Gray encode, empty flag
├── fifo_mem.v              dual-clock memory array
└── async_fifo_TB.v         self-checking testbench with scoreboard
```

### Top-level interface

| Port | Dir | Width | Domain | Description |
|---|---|---|---|---|
| `wclk`, `wrst_n` | in | 1 | write | Write clock, active-low async reset |
| `rclk`, `rrst_n` | in | 1 | read | Read clock, active-low async reset |
| `w_en` | in | 1 | write | Write request; accepted only if `full` is low |
| `r_en` | in | 1 | read | Read request; accepted only if `empty` is low |
| `data_in` | in | `DATA_WIDTH` | write | Write data |
| `data_out` | out | `DATA_WIDTH` | read | Read data, valid one `rclk` after an accepted `r_en` |
| `full` | out | 1 | write | Registered flag; no further writes are accepted |
| `empty` | out | 1 | read | Registered flag; no further reads are accepted |

### Flag and data latency

| Event | Observable effect | Latency |
|---|---|---|
| Accepted write | `empty` deasserts | 2 `rclk` synchronizer edges + 1 flag-register edge (about 3 `rclk`, plus up to one `rclk` of phase uncertainty) |
| Accepted read | `full` deasserts | 2 `wclk` synchronizer edges + 1 flag-register edge (about 3 `wclk`, plus up to one `wclk` of phase uncertainty) |
| Accepted read | `data_out` updates | 1 `rclk` |
| Write that fills the last slot | `full` asserts | same `wclk` edge as the pointer update (look-ahead, see `wptr_handler.v`) |
| Read that drains the last word | `empty` asserts | same `rclk` edge as the pointer update (look-ahead, see `rptr_handler.v`) |

The deassertion delays are intentional and safe: each side sees a slightly stale copy of the other side's pointer, which can only make the FIFO look fuller (to the writer) or emptier (to the reader) than it really is. Assertion is exact, deassertion is conservative.

### Build and run

```bash
iverilog -g2005 -Wall -o async_fifo_sim \
    fifo_mem.v synchronizer.v wptr_handler.v rptr_handler.v \
    asynchronous_fifo.v async_fifo_TB.v
vvp async_fifo_sim
gtkwave async_fifo.vcd
```

Expected console summary:

```
 words written : 2567
 words read    : 2567
 words compared: 2567
 errors        : 0
 RESULT: PASS
```

---

## 2. Design Files

### `asynchronous_fifo.v`

**Purpose.** Pure structural top level. It contains no behavioural logic: it computes `PTR_WIDTH`, declares the interconnect, and instantiates five blocks: two `synchronizer` instances (one per crossing direction), `wptr_handler`, `rptr_handler` and `fifo_mem`.

```verilog
module asynchronous_fifo #(
    parameter DEPTH      = 8,                 // power of 2, >= 4
    parameter DATA_WIDTH = 8
) (
    input                   wclk,
    input                   wrst_n,
    input                   rclk,
    input                   rrst_n,
    input                   w_en,
    input                   r_en,
    input  [DATA_WIDTH-1:0] data_in,
    output [DATA_WIDTH-1:0] data_out,         // wire: driven by fifo_mem port
    output                  full,             // wire: driven by wptr_handler port
    output                  empty             // wire: driven by rptr_handler port
);
    localparam PTR_WIDTH = $clog2(DEPTH);

    wire [PTR_WIDTH:0] b_wptr, b_rptr;
    wire [PTR_WIDTH:0] g_wptr, g_rptr;
    wire [PTR_WIDTH:0] g_wptr_sync, g_rptr_sync;

    synchronizer #(.WIDTH(PTR_WIDTH)) sync_wptr (
        .clk(rclk), .rst_n(rrst_n), .d_in(g_wptr), .d_out(g_wptr_sync));

    synchronizer #(.WIDTH(PTR_WIDTH)) sync_rptr (
        .clk(wclk), .rst_n(wrst_n), .d_in(g_rptr), .d_out(g_rptr_sync));

    wptr_handler #(.PTR_WIDTH(PTR_WIDTH)) wptr_h (
        .wclk(wclk), .wrst_n(wrst_n), .w_en(w_en),
        .g_rptr_sync(g_rptr_sync),
        .b_wptr(b_wptr), .g_wptr(g_wptr), .full(full));

    rptr_handler #(.PTR_WIDTH(PTR_WIDTH)) rptr_h (
        .rclk(rclk), .rrst_n(rrst_n), .r_en(r_en),
        .g_wptr_sync(g_wptr_sync),
        .b_rptr(b_rptr), .g_rptr(g_rptr), .empty(empty));

    fifo_mem #(.DEPTH(DEPTH), .DATA_WIDTH(DATA_WIDTH), .PTR_WIDTH(PTR_WIDTH)) fifom (
        .wclk(wclk), .w_en(w_en), .rclk(rclk), .r_en(r_en),
        .b_wptr(b_wptr), .b_rptr(b_rptr),
        .data_in(data_in), .full(full), .empty(empty),
        .data_out(data_out));
endmodule
```

**Structural logic.**

- `localparam PTR_WIDTH = $clog2(DEPTH)` is the number of memory address bits. Every pointer bus is declared `[PTR_WIDTH:0]`, one bit wider than the address, so the pointer sequence has `2 x DEPTH` states and the wrap bit disambiguates full from empty.
- `sync_wptr` is clocked by `rclk`/`rrst_n`: it carries `g_wptr` from the write domain into the read domain. `sync_rptr` is clocked by `wclk`/`wrst_n` and carries `g_rptr` the other way. The synchronizer is always clocked by the *destination* domain.
- `wptr_handler` receives `g_rptr_sync` (the only read-domain information the write side ever sees) and produces `b_wptr`, `g_wptr` and `full`. `rptr_handler` is its mirror image and produces `b_rptr`, `g_rptr` and `empty`.
- `fifo_mem` receives only the binary pointers, which stay inside their own domains, plus the flags used as write/read guards.

**Why `full`, `empty` and `data_out` are `wire` in this file.** In Verilog-2001 an output port of an instantiated module is a *continuous driver* of whatever it is connected to. A `reg` is a *variable*, which may only receive procedural assignments from `always` / `initial` blocks, so connecting a submodule output to a `reg` creates a concurrent (continuous) assignment to a non-net. Tools reject it. Icarus reports `reg full; cannot be driven by primitives or continuous assignment` together with `Output port expression must support continuous assignment`; other tools word it differently (typically as a "concurrent assignment to a non-net" or illegal output-port connection) but all reject it. This was reproduced in this repository by changing the two declarations to `output reg full, empty`:

```
asynchronous_fifo.v:31: error: reg full; cannot be driven by primitives or continuous assignment.
asynchronous_fifo.v:31: error: Output port expression must support continuous assignment.
asynchronous_fifo.v:31:      : Port 7 (full) of wptr_handler is connected to full
```

The actual storage element for `full` is a flip-flop, and it does exist, but it lives *inside* `wptr_handler` (`output reg full`, assigned in an `always @(posedge wclk ...)` block, which is legal because the assignment is procedural and local). In the parent, `full` is simply the net on which that flop's output is observed, so the parent must declare it as a net. The same rule applies to `empty`, to `data_out` (driven by the output register inside `fifo_mem`), and to every interconnect bus (`b_wptr`, `g_wptr`, `g_rptr_sync`, ...), which is why all of them are `wire`. The testbench follows the same rule: `full`, `empty` and `data_out` are `wire` in `async_fifo_TB.v`, because the DUT drives them. (In SystemVerilog, `logic` removes this restriction for single-driver signals, but this code base is deliberately pure Verilog-2001.)

**Parameter constraint.** `DEPTH` must be a power of two for two reasons: the pointer must wrap to zero naturally, and the Gray sequence is cyclic (its last code differs from code 0 by one bit) only when the count of states, `2 x DEPTH`, is a power of two. `DEPTH >= 4` is required because the full comparison slices `g_rptr_sync[PTR_WIDTH-2:0]`, which needs `PTR_WIDTH >= 2`.

---

### `synchronizer.v`

**Purpose.** The only place where a signal is allowed to change clock domain. It is a two-stage flip-flop synchronizer sampled by the destination clock, instantiated twice, once per pointer.

```verilog
module synchronizer #(
    parameter WIDTH = 3                       // address bits; bus is WIDTH+1 wide
) (
    input                  clk,               // DESTINATION-domain clock
    input                  rst_n,             // destination-domain reset (active low)
    input      [WIDTH:0]   d_in,              // Gray-coded pointer from the other domain
    output     [WIDTH:0]   d_out              // pointer, safe to use in clk domain
);
    (* ASYNC_REG = "TRUE" *) reg [WIDTH:0] q1;   // stage 1: may go metastable
    (* ASYNC_REG = "TRUE" *) reg [WIDTH:0] q2;   // stage 2: resolved value

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q1 <= {(WIDTH+1){1'b0}};
            q2 <= {(WIDTH+1){1'b0}};
        end else begin
            q1 <= d_in;
            q2 <= q1;
        end
    end

    assign d_out = q2;
endmodule
```

**Why raw (binary) pointers must never cross a clock boundary.** Two independent failure mechanisms apply, and they compound.

*1. Metastability caused by setup/hold violation.* A flip-flop guarantees a clean 0/1 output only if its data input is stable for `t_su` before and `t_h` after the active clock edge. The source pointer changes on `wclk`, which has no phase relationship to `rclk`, so sooner or later a pointer bit changes inside the capture window of the `rclk` flop. The master latch of the flop, a pair of cross-coupled inverters, is then left balanced near its unstable equilibrium. Its internal node sits at an intermediate voltage between `V_IL` and `V_IH`, and the output resolves to 0 or 1 only after a random time governed by an exponential regeneration, with resolution time constant `tau`. Three things go wrong while it hovers:

- The intermediate voltage propagates into downstream gates. Gates with different input thresholds interpret it differently, so one fan-out branch sees `1` while another sees `0`, and the design is in two states at once.
- Even if the level resolves, it may do so late enough to violate setup time on the *next* register, extending the problem downstream.
- The result of the sample (old value or new value) is unpredictable even after resolution.

The failure probability is quantified by `MTBF = e^(t_r / tau) / (T_0 x f_clk x f_data)`, where `t_r` is the time available for resolution. It cannot be driven to zero, only made exponentially rare by increasing `t_r`.

*2. Multi-bit convergence glitches.* Even without metastability, a multi-bit bus is not sampled atomically. Each bit has a different route delay, so on any given destination edge each bit independently reads either its old or its new value. A binary counter makes this catastrophic, because increments can flip many bits at once. Take a 4-bit pointer going from `0111` (7) to `1000` (8): all four bits flip. The capture flops can legally observe any of the 16 combinations, for example `1111` (15), `0000` (0), `1011` (11) or `0100` (4). None of those is "7" or "8". The read side might then compute `empty = (rptr == sampled_wptr)` against a pointer that is 7 words ahead of reality and pop entries that were never written (underrun), while the write side, given a corrupted `rptr`, might believe space exists and overwrite unread data (overrun). The binary number is effectively warped, and because the error is transient and pattern dependent, simulation with zero-delay gates will never reveal it.

**Why the two-flop structure fixes metastability.** Stage `q1` is allowed to go metastable; that is its job. Its output is not used by any logic. It feeds only the data input of stage `q2`, so `q1` has one full destination clock period (minus clk-to-Q and `t_su` of `q2`) to settle, which gives `t_r` close to `T_clk`. Because the resolution probability improves exponentially with `t_r`, a whole period of settling time lowers the probability that `q2` ever samples an unresolved level by many orders of magnitude, and `q2` becomes the first signal that is safe to use in the destination domain. A synchronizer does not make the *value* deterministic (`q2` may legitimately show the old or the new pointer), it only guarantees that the value is a clean logic level. That is exactly why the data entering it must be a code in which "old or new" are both valid, which is the role of the Gray encoding in `wptr_handler.v` and `rptr_handler.v` (one bit changes per increment, so a skewed sample is always either the previous pointer or the next one, never a third value). Synchronizer and Gray coding are two halves of one solution.

**Implementation notes.**

- Both stages use the destination clock and destination reset; the reset is asynchronous-assert so the pointers are cleared even before the destination clock is running.
- `(* ASYNC_REG = "TRUE" *)` instructs Xilinx tools to place `q1` and `q2` in the same slice and to exclude them from retiming or SRL inference, which preserves the settling time. Other tool flows ignore the attribute.
- Recommended constraints: a `set_max_delay -datapath_only` on `g_wptr -> q1` and `g_rptr -> q1` bounded by one source or destination period, and `set_bus_skew` on the Gray buses so that the single-bit-change property is not destroyed by routing skew.
- Pointer synchronization latency is the reason flag deassertion is delayed by a few cycles; see the latency table above.

---

### `wptr_handler.v`

**Purpose.** Everything the write domain owns: the binary write pointer used to address memory, its Gray-coded twin sent to the read domain, and the registered `full` flag.

```verilog
module wptr_handler #(
    parameter PTR_WIDTH = 3
) (
    input                    wclk,
    input                    wrst_n,
    input                    w_en,
    input  [PTR_WIDTH:0]     g_rptr_sync,     // read pointer, Gray, synchronized into wclk
    output reg [PTR_WIDTH:0] b_wptr,          // binary write pointer (memory address + wrap bit)
    output reg [PTR_WIDTH:0] g_wptr,          // Gray write pointer (sent across domains)
    output reg               full
);
    wire [PTR_WIDTH:0] b_wptr_next;
    wire [PTR_WIDTH:0] g_wptr_next;
    wire               wfull;

    // look-ahead pointer: advances only for an ACCEPTED write
    assign b_wptr_next = b_wptr + (w_en & ~full);
    // binary -> Gray: one XOR per bit
    assign g_wptr_next = (b_wptr_next >> 1) ^ b_wptr_next;

    // FULL: next Gray write pointer == read pointer with its two MSBs inverted
    assign wfull = (g_wptr_next == {~g_rptr_sync[PTR_WIDTH:PTR_WIDTH-1],
                                     g_rptr_sync[PTR_WIDTH-2:0]});

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            b_wptr <= {(PTR_WIDTH+1){1'b0}};
            g_wptr <= {(PTR_WIDTH+1){1'b0}};
        end else begin
            b_wptr <= b_wptr_next;
            g_wptr <= g_wptr_next;
        end
    end

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) full <= 1'b0;
        else         full <= wfull;
    end
endmodule
```

**Pointer datapath.**

- `b_wptr_next = b_wptr + (w_en & ~full)` increments only when a write is *accepted*. Gating by `~full` here means the pointer cannot run past a full FIFO even if the producer keeps `w_en` high.
- `g_wptr_next = (b_wptr_next >> 1) ^ b_wptr_next` is the binary-to-Gray conversion, one XOR per bit, no carry chain.
- `g_wptr` and `g_wptr_next` hold the same quantity at two moments. `g_wptr_next` is the combinational value at the D input of the register; `g_wptr` is the registered value at its Q output. After the clock edge, the register has loaded `g_wptr_next`, so the two names refer to the same pointer one cycle apart. Only the registered `g_wptr` is sent to the synchronizer, because a combinational signal leaving the domain would add glitches to a path that must be glitch-free.

**Full-flag generation: direct Gray comparison (Way 2).**

For a pointer with an extra MSB, the FIFO is full when the write pointer has advanced exactly `DEPTH` positions beyond the read pointer. In binary this means the pointers are identical except for the MSB: `b_wptr == b_rptr ^ (1 << N)`.

*Way 1* converts the synchronized Gray read pointer back to binary (`bin[i] = ^gray[N:i]`, a cascade of up to `N` XORs per bit), compares it to the binary look-ahead write pointer with "MSB differs, other bits equal", and registers the result. This puts a gray-to-binary prefix-XOR network, plus a mixed comparator, on the critical path from `g_rptr_sync` to the `full` register. The network costs either `N` XORs in a serial chain (logic depth `N`) or up to `N(N+1)/2` XORs if it is flattened to cut the depth, so area and delay trade against each other and both grow with pointer width.

*Way 2*, used here, never leaves the Gray domain. The Gray encoding `g(b) = b ^ (b >> 1)` is linear over GF(2), which means `g(x ^ y) = g(x) ^ g(y)`. Applying it to the binary full condition with `M = 1 << N` (the MSB mask):

```
g(b_wptr) = g(b_rptr ^ M)
          = g(b_rptr) ^ g(M)
          = g(b_rptr) ^ (M ^ (M >> 1))
          = g(b_rptr) ^ {1, 1, 0, ..., 0}        <- bits N and N-1 set
```

So **full means the Gray write pointer equals the Gray read pointer with its top two bits inverted and every other bit unchanged.** That is exactly the expression in the RTL:

```verilog
{~g_rptr_sync[PTR_WIDTH:PTR_WIDTH-1], g_rptr_sync[PTR_WIDTH-2:0]}
```

Why two bits and not one: in a Gray code the MSB of a number is the MSB of its binary value, but the second Gray bit is `b[N] ^ b[N-1]`. Flipping `b[N]` in binary therefore flips *both* Gray bit `N` and Gray bit `N-1`. Worked example for `DEPTH = 8` (4-bit pointers):

| `b_wptr` | `g_wptr` | `b_rptr` | `g_rptr` | `g_rptr` with top 2 bits inverted | Full? |
|---|---|---|---|---|---|
| 8 (`1000`) | `1100` | 0 (`0000`) | `0000` | `1100` | yes |
| 11 (`1011`) | `1110` | 3 (`0011`) | `0010` | `1110` | yes |
| 9 (`1001`) | `1101` | 0 (`0000`) | `0000` | `1100` | no |

Hardware cost of Way 2: two inverters and an `(N+1)`-bit equality comparator (XNOR per bit, AND-reduce). There is no gray-to-binary network, so it is both smaller (roughly `N+1` XNORs, versus the comparator plus the prefix-XOR network of Way 1) and faster (the longest path is one inverter, one XNOR and a reduction AND, independent of the XOR chain whose depth grows with pointer width). Because a Gray code is a bijection, plain equality tests also work without conversion, which is why `empty` is a straight equality compare (see `rptr_handler.v`).

**Why the flag is computed from `g_wptr_next`, not `g_wptr`.**

`full` is a registered flag (`full <= wfull`), and a registered flag always lags the logic that feeds it by one clock. If `wfull` were computed from the *current* `g_wptr`, the flag would describe the FIFO as it was one write ago. Timeline for `DEPTH = 8` holding 7 words, no reads, `w_en` held high:

| Edge | Event | Flag from `g_wptr_next` (this design) | Flag from `g_wptr` (broken) |
|---|---|---|---|
| `k` | Write #8 accepted, `b_wptr` becomes 8 | `g_wptr_next = 1100` equals inverted `g_rptr_sync` (`1100`), so `full <= 1` | `g_wptr` is still `gray(7) = 0100`, not equal, so `full <= 0` |
| `k+1` | `w_en` still high | `full = 1`, write is blocked, 8 words stored | `full` is sampled as `0`, so **write #9 is accepted into `mem[0]`, overwriting the oldest unread word**; `b_wptr` becomes 9. On this same edge the flag register finally loads `1` (`g_wptr` is now `1100`) |
| `k+2` | | still full | `g_wptr` has already moved to `gray(9) = 1101`, which no longer matches `1100`, so `full <= 0`: the flag was high for a single cycle and drops while the FIFO is still full |

The look-ahead flag is therefore not an optimization but a correctness requirement: the single-cycle window between "pointer updated" and "flag updated" is exactly long enough for one more write to slip through, after which the pointer has moved beyond the full-detection point, the flag pulses for one cycle and releases, and the FIFO overruns continuously. This was demonstrated by mutation: changing `wfull` and `rempty` to use `g_wptr` / `g_rptr` makes the testbench report `full not asserted` and `overflow protection failed, accepted 23` (instead of 8) in the overfill phase, followed by a stream of data mismatches.

---

### `rptr_handler.v`

**Purpose.** The mirror of `wptr_handler.v` for the read domain: binary read pointer for RAM addressing, Gray read pointer for the write side, and the registered `empty` flag.

```verilog
module rptr_handler #(
    parameter PTR_WIDTH = 3
) (
    input                    rclk,
    input                    rrst_n,
    input                    r_en,
    input  [PTR_WIDTH:0]     g_wptr_sync,     // write pointer, Gray, synchronized into rclk
    output reg [PTR_WIDTH:0] b_rptr,
    output reg [PTR_WIDTH:0] g_rptr,
    output reg               empty
);
    wire [PTR_WIDTH:0] b_rptr_next;
    wire [PTR_WIDTH:0] g_rptr_next;
    wire               rempty;

    assign b_rptr_next = b_rptr + (r_en & ~empty);
    assign g_rptr_next = (b_rptr_next >> 1) ^ b_rptr_next;

    // EMPTY: next Gray read pointer == synchronized Gray write pointer
    assign rempty = (g_rptr_next == g_wptr_sync);

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            b_rptr <= {(PTR_WIDTH+1){1'b0}};
            g_rptr <= {(PTR_WIDTH+1){1'b0}};
        end else begin
            b_rptr <= b_rptr_next;
            g_rptr <= g_rptr_next;
        end
    end

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) empty <= 1'b1;
        else         empty <= rempty;
    end
endmodule
```

**Logic.**

- `b_rptr_next = b_rptr + (r_en & ~empty)` advances only on an accepted read, and `g_rptr_next` is the same one-XOR-per-bit Gray conversion.
- **Empty condition:** the FIFO is empty when the read pointer has caught up with the write pointer, i.e. all `N+1` bits are equal, wrap bit included. Because Gray encoding is a bijection (distinct binary values map to distinct Gray values), binary equality is identical to Gray equality, so `rempty = (g_rptr_next == g_wptr_sync)` needs no conversion and no bit inversion. This is the asymmetry with `full`: equality is invariant under the encoding, the "differ by exactly `DEPTH`" relation is not and requires the two-bit inversion derived above.
- **Look-ahead is mandatory here too.** `empty` is computed from `g_rptr_next` for the same reason as `full`: if the flag were derived from the current `g_rptr`, then after the pop that drains the last word at edge `k`, `empty` would still read `0` at edge `k+1`. A second pop would be accepted, `b_rptr` would move one position past `b_wptr`, and the read pointer would now sit `2 x DEPTH - 1` positions behind the write pointer in modulo arithmetic. The FIFO would present stale RAM contents as valid data (underrun) and would report an inconsistent fill level. With look-ahead, `empty <= 1` is loaded on the very edge that drains the last word, so the next pop is already blocked.
- **Reset value is `1`.** After reset the FIFO holds nothing, and `empty` must be asserted from the first cycle so no spurious read can be accepted before the first write has propagated through the synchronizer.
- **Staleness is safe.** `g_wptr_sync` lags the real write pointer by about three `rclk` cycles. The synchronized pointer is therefore never ahead of the true pointer, so `empty` can remain asserted briefly after data has been written (lost cycles) but can never deassert before data is actually in the RAM (no underrun).

---

### `fifo_mem.v`

**Purpose.** The shared storage array, a simple dual-port RAM with an independent clock on each port.

```verilog
module fifo_mem #(
    parameter DEPTH      = 8,
    parameter DATA_WIDTH = 8,
    parameter PTR_WIDTH  = 3
) (
    input                       wclk,
    input                       w_en,
    input                       rclk,
    input                       r_en,
    input      [PTR_WIDTH:0]    b_wptr,
    input      [PTR_WIDTH:0]    b_rptr,
    input      [DATA_WIDTH-1:0] data_in,
    input                       full,
    input                       empty,
    output reg [DATA_WIDTH-1:0] data_out
);
    reg [DATA_WIDTH-1:0] fifo [0:DEPTH-1];

    // write port: wclk domain
    always @(posedge wclk) begin
        if (w_en & ~full)
            fifo[b_wptr[PTR_WIDTH-1:0]] <= data_in;
    end

    // read port: rclk domain, registered output
    always @(posedge rclk) begin
        if (r_en & ~empty)
            data_out <= fifo[b_rptr[PTR_WIDTH-1:0]];
    end
endmodule
```

**Logic.**

- The write port is clocked by `wclk` and writes `data_in` to `fifo[b_wptr[PTR_WIDTH-1:0]]` when `w_en & ~full`. The read port is clocked by `rclk`, with its output register `data_out` loaded from `fifo[b_rptr[PTR_WIDTH-1:0]]` when `r_en & ~empty`.
- Only the lower `PTR_WIDTH` bits of each binary pointer address the array. The MSB is the wrap bit, used by the flag logic, not by the RAM.
- The read is **registered**: `data_out` changes on the `rclk` edge that accepts the read, so the data is observable one `rclk` later. That one-cycle latency is a deliberate property (it maps to the output register of a block RAM and removes the RAM array access from the downstream timing path) and it is what the testbench scoreboard has to model.
- The `~full` / `~empty` guards duplicate the pointer gating as a second line of defence, and double as clock-enables when mapped to block RAM.
- No reset is applied to the array; `data_out` is `X` until the first accepted read.

**Why sharing the array between two clocks is not a CDC hazard.** The data word and the pointer that "publishes" it are written on the same `wclk` edge. The pointer then needs at least two `rclk` edges to traverse the synchronizer and one more to reach `empty`, so by the time the read domain sees `empty` go low, the RAM word has been stable for several cycles. Symmetrically, a slot can be overwritten only after the read pointer that frees it has crossed back through `sync_rptr`. Data never races its own pointer.

---

### `async_fifo_TB.v`

**Purpose.** A self-checking testbench that drives the FIFO from two unrelated clocks (`wclk` = 100 MHz, `rclk` approximately 71.4 MHz), mirrors every *accepted* transaction in a scoreboard, and compares every word that leaves the DUT against the word that was expected. It ends with a `PASS` / `FAIL` summary.

```verilog
`timescale 1ns/1ps

module async_fifo_TB;

    parameter DEPTH      = 8;
    parameter DATA_WIDTH = 8;
    parameter WCLK_HALF  = 5;       // 100 MHz write clock
    parameter RCLK_HALF  = 7;       // ~71.4 MHz read clock
    parameter SB_SIZE    = 8192;    // scoreboard capacity (>= total accepted writes)

    // ---------------- DUT interface ----------------
    reg                   wclk, rclk;
    reg                   wrst_n, rrst_n;
    reg                   w_en, r_en;
    reg  [DATA_WIDTH-1:0] data_in;
    wire [DATA_WIDTH-1:0] data_out;
    wire                  full, empty;

    asynchronous_fifo #(.DEPTH(DEPTH), .DATA_WIDTH(DATA_WIDTH)) dut (
        .wclk(wclk), .wrst_n(wrst_n),
        .rclk(rclk), .rrst_n(rrst_n),
        .w_en(w_en), .r_en(r_en),
        .data_in(data_in), .data_out(data_out),
        .full(full), .empty(empty));

    // ---------------- clocks ----------------
    initial wclk = 1'b0;
    always #(WCLK_HALF) wclk = ~wclk;
    initial rclk = 1'b0;
    always #(RCLK_HALF) rclk = ~rclk;

    // ---------------- stimulus drivers (non-blocking: race-free) ----------------
    reg [1:0] w_mode, r_mode;       // 0 idle, 1 every cycle, 2 random 50 %, 3 random 75 %

    always @(posedge wclk) begin
        data_in <= $random;
        case (w_mode)
            2'd1:    w_en <= 1'b1;
            2'd2:    w_en <= ({$random} % 4) < 2;
            2'd3:    w_en <= ({$random} % 4) < 3;
            default: w_en <= 1'b0;
        endcase
    end

    always @(posedge rclk) begin
        case (r_mode)
            2'd1:    r_en <= 1'b1;
            2'd2:    r_en <= ({$random} % 4) < 2;
            2'd3:    r_en <= ({$random} % 4) < 3;
            default: r_en <= 1'b0;
        endcase
    end

    // ---------------- scoreboard ----------------
    reg [DATA_WIDTH-1:0] scoreboard_mem [0:SB_SIZE-1];
    integer              sb_wptr, sb_rptr;
    integer              errors, checked;
    reg                  rd_check_pending;
    reg [DATA_WIDTH-1:0] rd_expected;

    // record every ACCEPTED write
    always @(posedge wclk) begin
        if (wrst_n && w_en && !full) begin
            scoreboard_mem[sb_wptr] <= data_in;
            sb_wptr                 <= sb_wptr + 1;
        end
    end

    // accepted read: stage 1 captures expectation, stage 2 (next rclk) compares
    always @(posedge rclk) begin
        if (rd_check_pending) begin
            checked = checked + 1;
            if (data_out !== rd_expected) begin
                errors = errors + 1;
                $display("[%0t] ERROR: mismatch expected=%h got=%h", $time, rd_expected, data_out);
            end
        end

        rd_check_pending <= 1'b0;
        if (rrst_n && r_en && !empty) begin
            if (sb_rptr >= sb_wptr) begin
                errors = errors + 1;
                $display("[%0t] ERROR: underrun, read accepted but scoreboard is empty", $time);
            end else begin
                rd_expected      <= scoreboard_mem[sb_rptr];
                sb_rptr          <= sb_rptr + 1;
                rd_check_pending <= 1'b1;
            end
        end
    end

    // ---------------- helpers ----------------
    task drain;
        begin
            @(negedge rclk); r_mode = 2'd1;
            while (sb_rptr != sb_wptr) @(posedge rclk);
            repeat (6) @(posedge rclk);          // keep r_en high on empty, flush last compare
            @(negedge rclk); r_mode = 2'd0;
            repeat (3) @(posedge rclk);
        end
    endtask

    // ---------------- sequencer ----------------
    initial begin
        $dumpfile("async_fifo.vcd");
        $dumpvars(0, async_fifo_TB);

        sb_wptr = 0; sb_rptr = 0; errors = 0; checked = 0;
        rd_check_pending = 1'b0; rd_expected = 0;
        w_mode = 0; r_mode = 0; w_en = 0; r_en = 0; data_in = 0;
        wrst_n = 1'b0; rrst_n = 1'b0;

        repeat (4) @(negedge wclk);  wrst_n = 1'b1;
        repeat (4) @(negedge rclk);  rrst_n = 1'b1;

        // PHASE 1: overfill (3x capacity attempted, DEPTH must be accepted)
        @(negedge wclk); w_mode = 2'd1;
        repeat (3*DEPTH) @(posedge wclk);
        @(negedge wclk); w_mode = 2'd0;
        repeat (4) @(posedge wclk);
        if (full !== 1'b1)      begin errors = errors + 1; $display("ERROR: full not asserted"); end
        if (sb_wptr != DEPTH)   begin errors = errors + 1; $display("ERROR: overflow protection failed, accepted %0d", sb_wptr); end

        // PHASE 2: drain, with underflow attempts while empty
        drain;
        if (empty !== 1'b1)     begin errors = errors + 1; $display("ERROR: empty not asserted"); end
        if (sb_rptr != DEPTH)   begin errors = errors + 1; $display("ERROR: underflow protection failed"); end

        // PHASE 3: write-heavy random traffic (75 % write @100 MHz vs 50 % read @71 MHz)
        @(negedge wclk); w_mode = 2'd3;
        @(negedge rclk); r_mode = 2'd2;
        repeat (3000) @(posedge wclk);
        @(negedge wclk); w_mode = 2'd0;
        drain;

        // PHASE 4: read-heavy random traffic (50 % write vs continuous read)
        @(negedge wclk); w_mode = 2'd2;
        @(negedge rclk); r_mode = 2'd1;
        repeat (3000) @(posedge wclk);
        @(negedge wclk); w_mode = 2'd0;
        drain;

        $display("--------------------------------------------------");
        $display(" words written : %0d", sb_wptr);
        $display(" words read    : %0d", sb_rptr);
        $display(" words compared: %0d", checked);
        $display(" errors        : %0d", errors);
        if (errors == 0 && checked == sb_wptr && sb_wptr == sb_rptr)
            $display(" RESULT: PASS");
        else
            $display(" RESULT: FAIL");
        $display("--------------------------------------------------");
        $finish;
    end

    initial begin
        #5_000_000;
        $display("[%0t] ERROR: watchdog expired", $time);
        $finish;
    end

endmodule
```

**Stimulus and race avoidance.**

- Clock generators use half-period parameters (`WCLK_HALF`, `RCLK_HALF`), so any ratio can be tested by editing two numbers.
- `w_mode` / `r_mode` select idle, every-cycle, 50 % random or 75 % random enables. The driver blocks use **non-blocking assignments** at `posedge`, so every DUT or monitor process that samples at the same edge sees the *pre-edge* values; the testbench therefore cannot race the design. The sequencer changes modes on the falling edge for the same reason, and resets are released on a falling edge so reset recovery is never violated.
- Enables are driven irrespective of `full` / `empty` on purpose. The testbench keeps `w_en` high while the FIFO is full and `r_en` high while it is empty, to prove that the RTL itself rejects those requests.

**Why a scoreboard array with manual pointers.** The scoreboard is an independent golden model: `scoreboard_mem[0:SB_SIZE-1]` with a manual write index `sb_wptr` and read index `sb_rptr`. This matters for several reasons:

- *Independence.* The model shares no code, pointer format or flag logic with the DUT. If the expected data came from the same Gray-pointer arithmetic that is being verified, a bug could cancel out; an array indexed by plain integers cannot be fooled by the same defect.
- *Only accepted transactions are recorded.* A write enters the scoreboard only when `w_en && !full` is true at the edge, and a read consumes an entry only when `r_en && !empty`. Requests that the DUT is *supposed* to reject therefore never enter the model, so any overflow that corrupts the FIFO shows up as a data mismatch, and any underrun triggers the explicit check `sb_rptr >= sb_wptr`.
- *Order checking.* A FIFO's contract is ordering. The scoreboard is an append-and-consume structure, so word `n` out must equal word `n` in, which also detects loss, duplication and reordering.
- *Occupancy visibility.* `sb_wptr - sb_rptr` is the true fill level and is visible in the waveform next to `full` and `empty`, which makes flag debugging direct.
- *Portability.* A manual array works in any Verilog-2001 simulator, with no dependency on SystemVerilog queues.
- *Capacity.* `SB_SIZE` must exceed the total number of accepted writes in a run (about 2570 here, against 8192 entries), since the indices are not wrapped.

**Why the compare is delayed by one clock.** `fifo_mem` has a registered read: when a read is accepted on `rclk` edge `n`, `data_out` is loaded by non-blocking assignment at that edge and holds the new word only after it. Comparing `data_out` in the same edge that accepts the read would sample the *previous* read's value (or `X` after reset), which produces false mismatches even though the DUT is correct. The monitor therefore uses two stages:

1. **Stage 1 (edge `n`, accepted read):** fetch `scoreboard_mem[sb_rptr]` into `rd_expected`, increment `sb_rptr`, and set `rd_check_pending`.
2. **Stage 2 (edge `n+1`):** if `rd_check_pending` is set, compare `data_out` with `rd_expected` using `!==` (so `X` and `Z` also fail).

During back-to-back reads the pending flag is re-armed every cycle, so the compare pipeline sustains one word per `rclk` with the expected data lined up behind the RAM output. The drain task keeps `r_en` high for six extra cycles after the last accepted read so that the final compare completes and the underflow-protection path is exercised. This was verified by mutation: collapsing the two stages into one compare at the accepting edge turns a clean run into 2559 false errors out of 2567 words.

**Test phases.**

| Phase | Stimulus | What it proves |
|---|---|---|
| Reset | Domains released on different falling edges | Reset values (`full = 0`, `empty = 1`) and independent reset release |
| 1: Overfill | `w_en` held high for `3 x DEPTH` cycles, no reads | Exactly `DEPTH` writes accepted (`sb_wptr == DEPTH`), `full` asserted, overflow rejected |
| 2: Drain | `r_en` held high until the scoreboard is empty, plus 6 extra cycles | All `DEPTH` words in order, `empty` asserted, underflow rejected (`sb_rptr == DEPTH`) |
| 3: Write-heavy random | 75 % write @ 100 MHz vs 50 % read @ ~71 MHz, 3000 `wclk` cycles | Sustained `full` back-pressure, pointer wrap-around, `g_rptr_sync` staleness handling |
| 4: Read-heavy random | 50 % write vs continuous read, 3000 `wclk` cycles | Sustained `empty` operation, `g_wptr_sync` staleness handling, simultaneous push and pop |

A watchdog (`#5_000_000`) terminates a hung simulation. The pass criterion is `errors == 0`, every word written was read (`sb_wptr == sb_rptr`) and every word read was compared (`checked == sb_wptr`).

---

## 3. Verification Evidence

Clean run (Icarus Verilog, `-g2005 -Wall`):

| Metric | Result |
|---|---|
| Words written / read / compared | 2567 / 2567 / 2567 |
| Errors | 0 |
| Clock ratio | 100 MHz write, ~71.4 MHz read (swap `WCLK_HALF` / `RCLK_HALF` to test the reverse) |

Mutation checks that confirm each design decision is load-bearing:

| Mutation | Observed result |
|---|---|
| `full` / `empty` declared `output reg` in `asynchronous_fifo.v` | Elaboration fails: `reg full; cannot be driven by primitives or continuous assignment` |
| Flags computed from `g_wptr` / `g_rptr` instead of `_next` | `full` never asserts correctly, 23 writes accepted into an 8-deep FIFO, repeated data mismatches |
| Scoreboard compare at the accepting edge (no 1-clock delay) | 2559 false mismatches in 2567 reads |

Note that zero-delay RTL simulation cannot expose metastability or bus skew, so the synchronizer and Gray-code argument in `synchronizer.v` is established by construction and by constraints (`ASYNC_REG`, max-delay and bus-skew limits), not by this testbench. Gate-level simulation with SDF, or a CDC lint tool, should complement it before tape-out or release.
