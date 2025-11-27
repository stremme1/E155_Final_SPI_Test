`timescale 1ns / 1ps

// Button Debouncer Module
// Debounces button inputs to prevent false triggers from mechanical bounce
// Uses FSM pattern for reliable debouncing
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
    
    // FSM states
    typedef enum logic [1:0] {
        IDLE,           // Button not pressed
        DEBOUNCING,     // Button state changing, debouncing
        BUTTON_HELD     // Button confirmed pressed
    } state_t;
    
    state_t current_state;
    logic btn_sync2_prev;
    
    // Double synchronizer for metastability protection
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_sync1 <= 1'b1;  // Default high (button not pressed)
            btn_sync2 <= 1'b1;
            btn_sync2_prev <= 1'b1;
        end else begin
            btn_sync1 <= btn_n;  // btn_n=0 means pressed, so btn_sync1=0 when pressed
            btn_sync2 <= btn_sync1;
            btn_sync2_prev <= btn_sync2;
        end
    end
    
    // Debouncer FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            debounce_counter <= 18'd0;
        end else begin
            case (current_state)
                IDLE: begin
                    // Check for button press (btn_sync2 goes from 1->0)
                    if (btn_sync2_prev && !btn_sync2) begin
                        // Button just pressed, start debouncing
                        current_state <= DEBOUNCING;
                        debounce_counter <= 18'd0;
                    end
                end
                
                DEBOUNCING: begin
                    if (btn_sync2_prev && !btn_sync2) begin
                        // Button released during debounce, go back to IDLE
                        current_state <= IDLE;
                    end else if (debounce_counter >= DEBOUNCE_CYCLES) begin
                        // Debounce complete, button is confirmed pressed
                        current_state <= BUTTON_HELD;
                    end else begin
                        // Continue debouncing
                        debounce_counter <= debounce_counter + 1;
                    end
                end
                
                BUTTON_HELD: begin
                    // Check for button release (btn_sync2 goes from 0->1)
                    if (!btn_sync2_prev && btn_sync2) begin
                        // Button released, go back to IDLE
                        current_state <= IDLE;
                    end
                    // Stay in BUTTON_HELD while button is pressed
                end
                
                default: begin
                    current_state <= IDLE;
                end
            endcase
        end
    end
    
    // Output assignments
    always_comb begin
        case (current_state)
            IDLE: begin
                btn_pressed = 1'b0;
                btn_released = 1'b0;
                btn_state = 1'b0;  // Not pressed
            end
            
            DEBOUNCING: begin
                if (debounce_counter >= DEBOUNCE_CYCLES) begin
                    // Transitioning to BUTTON_HELD, pulse btn_pressed
                    btn_pressed = 1'b1;
                end else begin
                    btn_pressed = 1'b0;
                end
                btn_released = 1'b0;
                btn_state = 1'b0;  // Not yet confirmed pressed
            end
            
            BUTTON_HELD: begin
                btn_pressed = 1'b0;  // No new pulse while held
                if (!btn_sync2_prev && btn_sync2) begin
                    // Transitioning to IDLE, pulse btn_released
                    btn_released = 1'b1;
                end else begin
                    btn_released = 1'b0;
                end
                btn_state = 1'b1;  // Confirmed pressed
            end
            
            default: begin
                btn_pressed = 1'b0;
                btn_released = 1'b0;
                btn_state = 1'b0;
            end
        endcase
    end
    
endmodule
