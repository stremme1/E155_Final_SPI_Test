`timescale 1ns / 1ps

// Button Debouncer Module
// Debounces button inputs to prevent false triggers from mechanical bounce
// Matches C code debounce logic: 50ms delay @ 3MHz = 150,000 cycles

module button_debouncer #(
    parameter DEBOUNCE_CYCLES = 150000  // 50ms @ 3MHz (matches C code DEBOUNCE_DELAY1/2)
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        btn_n,          // Active low button input (with pull-up)
    output logic        btn_pressed,    // Single-cycle pulse on button press
    output logic        btn_released,   // Single-cycle pulse on button release
    output logic        btn_state       // Current debounced state (1=pressed, 0=released)
);

    // Internal signals
    logic btn_sync1, btn_sync2;  // Double synchronizer
    logic [17:0] debounce_counter;  // 18 bits for 150,000 cycles
    logic btn_stable;
    logic btn_prev;
    
    // Double synchronizer for metastability protection
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_sync1 <= 1'b1;  // Default high (button not pressed)
            btn_sync2 <= 1'b1;
        end else begin
            btn_sync1 <= btn_n;  // Invert: btn_n=0 means pressed
            btn_sync2 <= btn_sync1;
        end
    end
    
    // Debounce counter logic (matches C code: millis() - lastDebounceTime > DEBOUNCE_DELAY)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debounce_counter <= 18'd0;
            btn_stable <= 1'b1;  // Default: button not pressed
            btn_prev <= 1'b1;
        end else begin
            btn_prev <= btn_stable;
            
            if (btn_sync2 != btn_stable) begin
                // Button state changed, reset counter
                debounce_counter <= 18'd0;
            end else if (debounce_counter < DEBOUNCE_CYCLES) begin
                // Count up while button state is stable
                debounce_counter <= debounce_counter + 1;
            end else begin
                // Counter reached threshold, update stable state
                btn_stable <= btn_sync2;
            end
        end
    end
    
    // Edge detection for pressed/released pulses
    assign btn_pressed = btn_stable && !btn_prev;   // Rising edge (0->1, button pressed)
    assign btn_released = !btn_stable && btn_prev;  // Falling edge (1->0, button released)
    assign btn_state = btn_stable;  // Current debounced state
    
endmodule

