`timescale 1ns / 1ps

// Drum Selector Module
// Combines zone, pitch, and gyro to select final drum/cymbal code
// Matches C code logic exactly

module drum_selector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [2:0]  zone_id,        // 0=snare, 1=high_tom, 2=mid_tom, 3=floor_tom
    input  logic signed [31:0] pitch,   // Q16.15 format, degrees
    input  logic signed [15:0] gyro_z,   // For left hand hi-hat detection
    input  logic        is_left_hand,    // 0=right hand, 1=left hand
    output logic        valid_out,
    output logic [3:0]  drum_code       // 0-7 (matches C code exactly)
);

    // Drum codes (EXACT from C code)
    localparam [3:0] CODE_SNARE    = 4'd0;
    localparam [3:0] CODE_HI_HAT   = 4'd1;
    localparam [3:0] CODE_KICK     = 4'd2;
    localparam [3:0] CODE_HIGH_TOM = 4'd3;
    localparam [3:0] CODE_MID_TOM  = 4'd4;
    localparam [3:0] CODE_CRASH    = 4'd5;
    localparam [3:0] CODE_RIDE     = 4'd6;
    localparam [3:0] CODE_FLOOR_TOM = 4'd7;
    
    // Zone IDs
    localparam [2:0] ZONE_SNARE    = 3'd0;
    localparam [2:0] ZONE_HIGH_TOM = 3'd1;
    localparam [2:0] ZONE_MID_TOM  = 3'd2;
    localparam [2:0] ZONE_FLOOR_TOM = 3'd3;
    
    // Pitch thresholds in Q16.15 format (degrees * 32768)
    // 30° = 30 * 32768 = 983,040
    // 50° = 50 * 32768 = 1,638,400
    localparam [31:0] PITCH_30 = 32'd983040;
    localparam [31:0] PITCH_50 = 32'd1638400;
    
    // Gyro threshold for hi-hat detection
    localparam signed [15:0] GYRO_Z_THRESHOLD = -16'sd2000;
    
    // Drum selection logic (EXACT from C code)
    logic [3:0] drum_code_comb;
    
    always_comb begin
        if (is_left_hand) begin
            // Left hand logic (EXACT from C code)
            case (zone_id)
                ZONE_SNARE: begin
                    // if (pitch2 > 30 && gyro2_z > -2000) -> hi-hat
                    // else -> snare
                    if ((pitch > PITCH_30) && (gyro_z > GYRO_Z_THRESHOLD)) begin
                        drum_code_comb = CODE_HI_HAT;
                    end else begin
                        drum_code_comb = CODE_SNARE;
                    end
                end
                ZONE_HIGH_TOM: begin
                    // if (pitch2 > 50) -> crash
                    // else -> high tom
                    if (pitch > PITCH_50) begin
                        drum_code_comb = CODE_CRASH;
                    end else begin
                        drum_code_comb = CODE_HIGH_TOM;
                    end
                end
                ZONE_MID_TOM: begin
                    // if (pitch2 > 50) -> ride
                    // else -> mid tom
                    if (pitch > PITCH_50) begin
                        drum_code_comb = CODE_RIDE;
                    end else begin
                        drum_code_comb = CODE_MID_TOM;
                    end
                end
                ZONE_FLOOR_TOM: begin
                    // if (pitch2 > 30) -> ride
                    // else -> floor tom
                    if (pitch > PITCH_30) begin
                        drum_code_comb = CODE_RIDE;
                    end else begin
                        drum_code_comb = CODE_FLOOR_TOM;
                    end
                end
                default: drum_code_comb = CODE_SNARE;
            endcase
        end else begin
            // Right hand logic (EXACT from C code)
            case (zone_id)
                ZONE_SNARE: begin
                    // Always snare
                    drum_code_comb = CODE_SNARE;
                end
                ZONE_HIGH_TOM: begin
                    // if (pitch1 > 50) -> crash
                    // else -> high tom
                    if (pitch > PITCH_50) begin
                        drum_code_comb = CODE_CRASH;
                    end else begin
                        drum_code_comb = CODE_HIGH_TOM;
                    end
                end
                ZONE_MID_TOM: begin
                    // if (pitch1 > 50) -> ride
                    // else -> mid tom
                    if (pitch > PITCH_50) begin
                        drum_code_comb = CODE_RIDE;
                    end else begin
                        drum_code_comb = CODE_MID_TOM;
                    end
                end
                ZONE_FLOOR_TOM: begin
                    // if (pitch1 > 30) -> ride
                    // else -> floor tom
                    if (pitch > PITCH_30) begin
                        drum_code_comb = CODE_RIDE;
                    end else begin
                        drum_code_comb = CODE_FLOOR_TOM;
                    end
                end
                default: drum_code_comb = CODE_SNARE;
            endcase
        end
    end
    
    // Pipeline register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drum_code <= CODE_SNARE;
            valid_out <= 1'b0;
        end else begin
            drum_code <= drum_code_comb;
            valid_out <= valid_in;
        end
    end
    
endmodule

