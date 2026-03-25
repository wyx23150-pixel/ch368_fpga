module cam_light_channel (
    input  wire        clk,
    input  wire        rst_n,
    
    input  wire        trigger_pulse,  // 接收大脑发来的瞬间脉冲
    input  wire [7:0]  delay_ms,       // 接收大脑分配的配置时间
    
    output wire        light_pin,      // 物理输出引脚 (低电平有效)
    output wire        cam_pin         // 物理输出引脚 (低电平有效)
);

    wire [23:0] delay_cycles = delay_ms * 24'd24_000;
    
    reg [23:0] light_timer;
    reg [23:0] cam_delay_timer;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            light_timer <= 0;
            cam_delay_timer <= 0;
        end else begin
            if (trigger_pulse) begin
                // 光源亮起时间 = 提前量 + 相机10ms曝光时间
                light_timer     <= delay_cycles + 24'd240_000; 
                cam_delay_timer <= delay_cycles;
            end else begin
                if (light_timer > 0)     light_timer     <= light_timer - 1'b1;
                if (cam_delay_timer > 0) cam_delay_timer <= cam_delay_timer - 1'b1;
            end
        end
    end

    // 常态输出 1，触发时输出 0 (完美兼容工业光耦和继电器)
    assign light_pin = (light_timer > 0) ? 1'b0 : 1'b1;
    assign cam_pin   = (light_timer > 0 && cam_delay_timer == 0) ? 1'b0 : 1'b1;

endmodule