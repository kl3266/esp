// TILES*PLANES inputs to 1 output router arbiter
//
// There is no delay from request to grant.
// The abriter assumes that the request remains stable while the entire
// packet is forwarded. Hence, priority is updated whenever a tail flit
// is forwarded. Grant is locked between a head flit and the corresponding
// tail flit.
//
// Interface
//
// * Inputs
// - clk: clock.
// - rst: active-high reset.
// - request: each bit should be set to 1 if there is a valid flit coming from the corresponding
//   input port that needs to be routed to the output port arbitrated by this module.
// - forwarding_head: set to 1 to indicate the head flit of a new packet is being routed this cycle.
//   The current grant gets locked until the tail flit is forwarded (wormhole routing)
// - forwarding_tail: set to 1 to indicate the tail flit of a packet is being routed this cycle.
//   Priority is updated and grant is unlocked.
//
// * Outputs
// - grant: one-hot or zero. When grant[i] is set, request[i] is granted and the packet from the
//   corresponding input port i can be routed.
//   and the packet from the input
// - grant_valid: this flag indicates whether the current grant output is valid. When at least one
//   request bit is set, the arbiter grants the next higher priority request with zero-cycles delay,
//   unless grant is locked.
//

module d2d_arbiter #(
  parameter int CHANNELS = 2
  )
  (
  input  logic clk,
  input  logic rstn,
  input  logic [CHANNELS-1:0] request,
  input  logic grant_valid_other_arbiter,
  output logic [$clog2(CHANNELS)-1:0] grant_index,
  output logic grant_valid
);
  localparam int N = CHANNELS;
  localparam int logN = $clog2(N);

  // Update priority
  typedef logic [N-1:0][N-1:0] priority_t;
  priority_t priority_mask, priority_mask_next;
  priority_t grant_stage1;
  logic [N-1:0][logN-1:0] grant_stage2;
  logic priority_shift;
  logic [N-1:0] grant;

  assign priority_shift = grant_valid_other_arbiter & grant_valid;

  // Higher priority is given to request[0] at reset
  function automatic priority_t gen_priority_matrix();
    priority_t matrix;
    for (int i = 0; i < N; i++) begin
      for (int j = 0; j < N; j++) begin
        matrix[i][N-1-j] = (j < (N - 1 - i)) ? 1'b1 : 1'b0;
      end
    end
    return matrix;
  endfunction

  always_ff @(rstn, posedge clk) begin
    if (!rstn) begin
      priority_mask <= gen_priority_matrix();
    end else if (posedge clk) begin
      if (priority_shift) begin
        priority_mask <= priority_mask_next;
      end
    end
  end

  always_comb begin
    priority_mask_next = priority_mask;
    for (int g = 0; g < N; g++) begin
      if (grant[g]) begin
        priority_mask_next[g] = '0;
        for (int i = 0; i < N; i++) begin
          if (i != g) begin
            priority_mask_next[i][g] = 1'b1;
          end
        end
      end
    end
  end

  genvar g_i, g_j;
  for (g_i = 0; g_i < N; g_i++) begin : gen_grant

      for (g_j = 0; g_j < N; g_j++) begin : gen_grant_stage1
          assign grant_stage1[g_i][g_j] = request[g_j] & priority_mask[g_j][g_i];
      end

      for (g_j = 0; g_j < logN; g_j++) begin : gen_grant_stage2
        if (2*g_j + 1 < N) begin
          assign grant_stage2[g_i][g_j] = ~(grant_stage1[g_i][2*g_j] | grant_stage1[g_i][2*g_j + 1]);
        end else begin
          assign grant_stage2[g_i][g_j] = ~(grant_stage1[g_i][2*g_j]);
        end
      end

      assign grant[g_i] = &grant_stage2[g_i] & request[g_i];

  end  // gen_grant

  always_comb begin
    grant_index = '0;
    grant_valid = 1'b0;
    for (int i = 0; i < N; i++) begin
      if (grant[i]) begin
        grant_index = i;
        grant_valid = 1'b1;
      end
    end
  end

    //
    // Assertions
    //

`ifndef SYNTHESIS
    // pragma coverage off
    //VCS coverage off

    a_grant_onehot :
    assert property (@(posedge clk) disable iff (!rstn) $onehot0(grant))
    else $error("Fail: a_grant_onehot");

    // pragma coverage on
    //VCS coverage on
`endif  // ~SYNTHESIS

endmodule
