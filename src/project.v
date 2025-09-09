/*
 * Bus Transaction FSM in TinyTapeout template
 * Author: Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none
`timescale 1ns/1ps

module tt_um_bus_fsm (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // Map inputs
    wire req = ui_in[0];   // request
    wire rw  = ui_in[1];   // 1 = READ, 0 = WRITE

    // FSM outputs
    reg ack, busy, done, data_valid;

    // state encoding
    localparam [2:0]
        S_IDLE      = 3'b000,
        S_ADDR_ACK  = 3'b001,
        S_DATA      = 3'b010,
        S_RESP      = 3'b011;

    reg [2:0] state, next_state;

    // internal reg for demo
    reg [7:0] internal_reg;

    // Active-low reset handling
    wire rst = ~rst_n;

    // State register
    always @(posedge clk) begin
        if (rst) state <= S_IDLE;
        else     state <= next_state;
    end

    // Next-state logic
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:     if (req) next_state = S_ADDR_ACK;
            S_ADDR_ACK: next_state = S_DATA;
            S_DATA:     next_state = S_RESP;
            S_RESP:     next_state = S_IDLE;
            default:    next_state = S_IDLE;
        endcase
    end

    // Output & datapath
    always @(posedge clk) begin
        if (rst) begin
            ack         <= 0;
            busy        <= 0;
            done        <= 0;
            data_valid  <= 0;
            internal_reg <= 8'h00;
        end else begin
            done       <= 0;
            data_valid <= 0;

            case (state)
                S_IDLE: begin
                    ack  <= 0;
                    busy <= 0;
                end

                S_ADDR_ACK: begin
                    ack  <= 1;
                    busy <= 1;
                end

                S_DATA: begin
                    ack  <= 1;
                    busy <= 1;
                    if (rw == 1'b0)
                        internal_reg <= internal_reg + 8'h11;  // WRITE
                    else
                        internal_reg <= internal_reg ^ 8'hAA;  // READ
                end

                S_RESP: begin
                    ack  <= 0;
                    busy <= 0;
                    done <= 1;
                    if (rw == 1'b1) data_valid <= 1;
                end

                default: begin
                    ack  <= 0;
                    busy <= 0;
                end
            endcase
        end
    end

    // Map outputs to uo_out (use lower bits)
    assign uo_out[0] = ack;
    assign uo_out[1] = busy;
    assign uo_out[2] = done;
    assign uo_out[3] = data_valid;
    assign uo_out[7:4] = internal_reg[3:0]; // expose part of internal reg for debug

    // IOs not used
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // Tie off unused signals to prevent warnings
    wire _unused = &{ena, uio_in, ui_in[7:2], 1'b0};

endmodule
