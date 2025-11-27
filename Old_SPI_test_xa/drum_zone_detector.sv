`timescale 1ns / 1ps

// Drum Zone Detector Module
// Determines drum zone from yaw angle based on exact C code ranges
// Pure combinational logic for zone detection

module drum_zone_detector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [31:0] yaw,           // 0-360 degrees, Q16.15 format
    input  logic        is_left_hand,  // 0=right hand, 1=left hand
    output logic        valid_out,
    output logic [2:0]  zone_id        // 0=snare, 1=high_tom, 2=mid_tom, 3=floor_tom
);

    // Zone IDs (matches C code zone logic)
    localparam [2:0] ZONE_SNARE    = 3'd0;
    localparam [2:0] ZONE_HIGH_TOM = 3'd1;
    localparam [2:0] ZONE_MID_TOM  = 3'd2;
    localparam [2:0] ZONE_FLOOR_TOM = 3'd3;
    
    // Constants for comparisons (in Q16.15 format: degrees * 32768)
    // 20° = 20 * 32768 = 655,360
    // 100° = 100 * 32768 = 3,276,800
    // 120° = 120 * 32768 = 3,932,160
    // 200° = 200 * 32768 = 6,553,600
    // 300° = 300 * 32768 = 9,830,400
    // 305° = 305 * 32768 = 9,994,240
    // 325° = 325 * 32768 = 10,649,600
    // 340° = 340 * 32768 = 11,141,120
    // 350° = 350 * 32768 = 11,468,800
    // 360° = 360 * 32768 = 11,796,480
    
    localparam [31:0] DEG_20  = 32'd655360;
    localparam [31:0] DEG_100 = 32'd3276800;
    localparam [31:0] DEG_120 = 32'd3932160;
    localparam [31:0] DEG_200 = 32'd6553600;
    localparam [31:0] DEG_300 = 32'd9830400;
    localparam [31:0] DEG_305 = 32'd9994240;
    localparam [31:0] DEG_325 = 32'd10649600;
    localparam [31:0] DEG_340 = 32'd11141120;
    localparam [31:0] DEG_350 = 32'd11468800;
    localparam [31:0] DEG_360 = 32'd11796480;
    
    // Zone detection logic (EXACT from C code)
    logic [2:0] zone_id_comb;
    
    always_comb begin
        if (is_left_hand) begin
            // Left hand zones (EXACT from C code - if-else priority)
            // if (yaw2 >= 350 || yaw2 <= 100) -> snare/hi-hat zone
            if ((yaw >= DEG_350) || (yaw <= DEG_100)) begin
                zone_id_comb = ZONE_SNARE;
            end
            // else if (yaw2 >= 325 && yaw2 <= 350) -> high tom
            else if ((yaw >= DEG_325) && (yaw <= DEG_350)) begin
                zone_id_comb = ZONE_HIGH_TOM;
            end
            // else if (yaw2 >= 300 && yaw2 <= 325) -> mid tom
            else if ((yaw >= DEG_300) && (yaw <= DEG_325)) begin
                zone_id_comb = ZONE_MID_TOM;
            end
            // else if (yaw2 >= 200 && yaw2 <= 300) -> floor tom
            else if ((yaw >= DEG_200) && (yaw <= DEG_300)) begin
                zone_id_comb = ZONE_FLOOR_TOM;
            end
            else begin
                zone_id_comb = ZONE_SNARE;  // Default
            end
        end else begin
            // Right hand zones (EXACT from C code - if-else priority)
            // if (yaw1 >= 20 && yaw1 <= 120) -> snare (takes priority at yaw=20)
            if ((yaw >= DEG_20) && (yaw <= DEG_120)) begin
                zone_id_comb = ZONE_SNARE;
            end
            // else if (yaw1 >= 340 || yaw1 <= 20) -> high tom (wraps around, but yaw=20 excluded by above if)
            else if ((yaw >= DEG_340) || (yaw < DEG_20)) begin  // Changed <= to < to exclude 20
                zone_id_comb = ZONE_HIGH_TOM;
            end
            // else if (yaw1 >= 305 && yaw1 <= 340) -> mid tom
            else if ((yaw >= DEG_305) && (yaw <= DEG_340)) begin
                zone_id_comb = ZONE_MID_TOM;
            end
            // else if (yaw1 >= 200 && yaw1 <= 305) -> floor tom
            else if ((yaw >= DEG_200) && (yaw <= DEG_305)) begin
                zone_id_comb = ZONE_FLOOR_TOM;
            end
            else begin
                zone_id_comb = ZONE_SNARE;  // Default
            end
        end
    end
    
    // Pipeline register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            zone_id <= ZONE_SNARE;
            valid_out <= 1'b0;
        end else begin
            zone_id <= zone_id_comb;
            valid_out <= valid_in;
        end
    end
    
endmodule

