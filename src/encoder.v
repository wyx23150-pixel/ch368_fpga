module encoder(
    input  wire        clk,      // 系统时钟 (24MHz)
    input  wire        rst_n,    // 系统复位 (低电平有效)
    
    // 编码器物理引脚输入
    input  wire        enc_a,    // 接 P2 的 21 脚
    input  wire        enc_b,    // 接 P2 的 23 脚
    input  wire        enc_z,    // 接 P2 的 25 脚
    
    // 32位计数值输出，送给 CH368 模块去读
    output reg  [31:0] count     
);

    // =======================================================
    // 1. 消除亚稳态：用 24M 时钟连续打两拍，把外部毛刺滤掉
    // =======================================================
    reg a_d1, a_d2;
    reg b_d1, b_d2;
    reg z_d1, z_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_d1 <= 0; a_d2 <= 0;
            b_d1 <= 0; b_d2 <= 0;
            z_d1 <= 0; z_d2 <= 0;
        end else begin
            a_d1 <= enc_a; a_d2 <= a_d1;
            b_d1 <= enc_b; b_d2 <= b_d1;
            z_d1 <= enc_z; z_d2 <= z_d1;
        end
    end

    // =======================================================
    // 2. 边沿检测：找出 A、B、Z 发生变化的那一瞬间
    // =======================================================
    wire a_pos = (a_d1 == 1'b1 && a_d2 == 1'b0); // A 上升沿
    wire a_neg = (a_d1 == 1'b0 && a_d2 == 1'b1); // A 下降沿
    wire b_pos = (b_d1 == 1'b1 && b_d2 == 1'b0); // B 上升沿
    wire b_neg = (b_d1 == 1'b0 && b_d2 == 1'b1); // B 下降沿
    wire z_pos = (z_d1 == 1'b1 && z_d2 == 1'b0); // Z 上升沿 (一圈一次)

    // =======================================================
    // 3. 核心 4倍频 鉴相与计数逻辑
    // =======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= 32'd0;
        end 
        else if (z_pos) begin
            // 收到 Z 脉冲，说明转完了一圈，位置清零归位！
            count <= 32'd0;
        end 
        // 任何一个边沿到来，都根据另一相的当前电平来判断正反转
        else if (a_pos) begin
            count <= (b_d2 == 1'b0) ? (count + 1'b1) : (count - 1'b1);
        end 
        else if (a_neg) begin
            count <= (b_d2 == 1'b1) ? (count + 1'b1) : (count - 1'b1);
        end 
        else if (b_pos) begin
            count <= (a_d2 == 1'b1) ? (count + 1'b1) : (count - 1'b1);
        end 
        else if (b_neg) begin
            count <= (a_d2 == 1'b0) ? (count + 1'b1) : (count - 1'b1);
        end
    end

endmodule