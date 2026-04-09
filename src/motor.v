module motor(
    input  wire       clk,        // 系统时钟 24MHz
    input  wire       rst_n,      // 复位信号

    // 电脑下发的控制信号 (来自 CH368 内部总线)
    input  wire       en_val,     // 1=使能(转), 0=失能(停)
    input  wire       dir_val,    // 1=正转, 0=反转
    input  wire [7:0] speed_val,  // 速度调节 (0~255)

    // 输出给外部电机驱动器 (如 A4988, TB6600 等) 的物理引脚
    output wire       motor_pwm,  // 步进脉冲引脚 (PUL / STEP)
    output wire       motor_dir,  // 方向控制引脚 (DIR)
    output wire       motor_en    // 使能控制引脚 (ENA)
);

    // 使用 20位 的相位累加器来控制频率
    reg [19:0] freq_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq_cnt <= 20'd0;
        end else if (en_val) begin
            // 核心调频逻辑：每次时钟周期加上 speed_val。
            // speed_val 越大，累加器溢出翻转得越快，产生的脉冲频率越高
            freq_cnt <= freq_cnt + speed_val;
        end else begin
            freq_cnt <= 20'd0; // 失能时清零，防止意外发脉冲
        end
    end

    // 提取累加器的最高位作为脉冲输出。
    // 这能自动产生一个 50% 占空比的方波22-6k，且无缝衔接。
    // 如果 speed_val 为 0，freq_cnt 不变，自然也就没有脉冲输出了。
    assign motor_pwm = en_val ? freq_cnt[19] : 1'b0;

    // 方向和使能信号直接透传给物理引脚
    assign motor_dir = dir_val;
    assign motor_en  = en_val;

endmodule