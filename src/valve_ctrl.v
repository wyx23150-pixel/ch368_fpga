module valve_ctrl (
    input  wire        clk,
    input  wire        rst_n,
    
    input  wire        trigger_pulse,  // 接收大脑发来的瞬间脉冲
    input  wire [7:0]  blow_ms,        // 接收大脑分配的吹气时长
    
    output wire        valve_pin       // 物理气阀引脚 (低电平有效)
);

    wire [23:0] blow_cycles = blow_ms * 24'd24_000;
    reg  [23:0] timer;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) timer <= 0;
        else begin
            if (trigger_pulse) timer <= blow_cycles;
            else if (timer > 0) timer <= timer - 1'b1;
        end
    end

    // 常态输出 1，触发倒计时内输出 0
    assign valve_pin = (timer > 0) ? 1'b0 : 1'b1;

endmodule