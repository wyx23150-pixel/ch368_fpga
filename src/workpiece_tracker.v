module workpiece_tracker(
    input  wire        clk,          // 系统时钟 (24MHz)
    input  wire        rst_n,        // 复位信号
    
    // 物理世界信号
    input  wire [15:0] current_enc,  // 实时编码器数值 (取绝对值的低16位即可)
    input  wire        sensor_in,    // 光电传感器输入 (高电平代表来料了)
    output wire        camera_trig,  // 输出给相机的硬件触发脉冲
    output wire        reject_trig,  // 输出给气缸的剔除硬件脉冲

    // CH368 总线通信接口
    input  wire [7:0]  addr,         // 8位地址线 (0x00 ~ 0xFF)
    input  wire        wr_en,        // 写使能脉冲 (高电平有效的一个时钟周期)
    input  wire [7:0]  data_in,      // 电脑发来的数据
    input  wire        rd_en,        // 读使能信号 (低电平有效)
    output reg  [7:0]  data_out      // 发给电脑的数据
);

    // =========================================================
    // 1. 物理信号处理：传感器边缘检测 & 编码器相对位移计算
    // =========================================================
    reg sensor_d1, sensor_d2;
    reg [15:0] prev_enc;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_d1 <= 0; sensor_d2 <= 0;
            prev_enc <= 0;
        end else begin
            sensor_d1 <= sensor_in; 
            sensor_d2 <= sensor_d1;
            prev_enc  <= current_enc; // 记录上一拍的编码器值
        end
    end
    
    // 传感器上升沿检测：新工件到达！
    wire new_workpiece_pulse = (sensor_d1 == 1'b1 && sensor_d2 == 1'b0);
    // 相对位移计算：这一拍编码器走了多少？(巧妙利用 16位溢出特性，无视正反转和归零)
    wire [15:0] enc_diff = current_enc - prev_enc; 

    // =========================================================
    // 2. 核心硬件队列 (4个槽位) 及其影子锁存器
    // =========================================================
    reg        slot_valid [0:3];     // 槽位是否有效 (0空闲，1有货)
    reg [7:0]  slot_state [0:3];     // 工件状态 (1=已上料, 2=良品, 3=不良品)
    reg [15:0] slot_pos   [0:3];     // 工件的相对计步器

    reg [7:0]  shadow_pos_h [0:3];   // 影子寄存器：专为防数据撕裂准备的高8位缓存

    // =========================================================
    // 3. 队列维护与自动移位逻辑 
    // =========================================================
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<4; i=i+1) begin
                slot_valid[i] <= 0;
                slot_state[i] <= 0;
                slot_pos[i]   <= 0;
            end
        end 
        else begin
            // 【并行维护】：所有有效槽位的计步器，同时累加相对位移！
            for (i=0; i<4; i=i+1) begin
                if (slot_valid[i]) begin
                    slot_pos[i] <= slot_pos[i] + enc_diff;
                end
            end

            // 【入料分配】：如果有新料来，瞬间寻找第一个空槽位装进去
            if (new_workpiece_pulse) begin
                if      (!slot_valid[0]) begin slot_valid[0] <= 1; slot_pos[0] <= 0; slot_state[0] <= 8'h01; end
                else if (!slot_valid[1]) begin slot_valid[1] <= 1; slot_pos[1] <= 0; slot_state[1] <= 8'h01; end
                else if (!slot_valid[2]) begin slot_valid[2] <= 1; slot_pos[2] <= 0; slot_state[2] <= 8'h01; end
                else if (!slot_valid[3]) begin slot_valid[3] <= 1; slot_pos[3] <= 0; slot_state[3] <= 8'h01; end
            end
            
            // 【出料销毁】：如果走到 3000 脉冲 (已经过了剔除站)，自动销毁档案，释放槽位
            for (i=0; i<4; i=i+1) begin
                if (slot_valid[i] && slot_pos[i] > 16'd3000) begin
                    slot_valid[i] <= 0; // 槽位回收！
						  slot_pos[i] <= 0;
						  slot_state[i] <=0;
                end
            end

            // 【电脑下发命令处理】：如果电脑发来了写操作，更新对应槽位的状态
            if (wr_en) begin
                case (addr)
                    8'h13: slot_state[0] <= data_in; // 电脑向 0 号槽位写结果
                    8'h23: slot_state[1] <= data_in; // 电脑向 1 号槽位写结果
                    8'h33: slot_state[2] <= data_in;
                    8'h43: slot_state[3] <= data_in;
                endcase
            end
        end
    end

// =========================================================
    // 4. 读操作与影子寄存器锁存 (完美匹配 CH368 高速时序)
    // =========================================================
    
    // 4.1 读信号的上升沿检测 (用于触发锁存动作)
    reg rd_en_d1, rd_en_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_en_d1 <= 0;
            rd_en_d2 <= 0;
        end else begin
            rd_en_d1 <= rd_en;
            rd_en_d2 <= rd_en_d1;
        end
    end
    wire rd_rising_edge = (rd_en_d1 == 1'b1 && rd_en_d2 == 1'b0);

    // 4.2 锁存影子寄存器 (仅在被读取的瞬间触发一次)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shadow_pos_h[0] <= 8'h00;
            shadow_pos_h[1] <= 8'h00;
            shadow_pos_h[2] <= 8'h00;
            shadow_pos_h[3] <= 8'h00;
        end else if (rd_rising_edge) begin
            if (addr == 8'h11) shadow_pos_h[0] <= slot_pos[0][15:8];
            if (addr == 8'h21) shadow_pos_h[1] <= slot_pos[1][15:8];
            if (addr == 8'h31) shadow_pos_h[2] <= slot_pos[2][15:8];
            if (addr == 8'h41) shadow_pos_h[3] <= slot_pos[3][15:8];
        end
    end

    // 4.3 组合逻辑极速输出 (绝对零延迟，像你最早的代码一样！)
    always @(*) begin
        case (addr)
            // 🌟 魔法测试位：只要能读出 5A，说明通信完全没问题！
            8'h00: data_out = 8'h5A; 

            // --- 槽位 0 ---
            8'h10: data_out = slot_valid[0] ? slot_state[0] : 8'h00; 
            8'h11: data_out = slot_pos[0][7:0];         
            8'h12: data_out = shadow_pos_h[0];       

            // --- 槽位 1 ---
            8'h20: data_out = slot_valid[1] ? slot_state[1] : 8'h00;
            8'h21: data_out = slot_pos[1][7:0]; 
            8'h22: data_out = shadow_pos_h[1];
            
            // --- 槽位 2 ---
            8'h30: data_out = slot_valid[2] ? slot_state[2] : 8'h00;
            8'h31: data_out = slot_pos[2][7:0]; 
            8'h32: data_out = shadow_pos_h[2];

            // --- 槽位 3 ---
            8'h40: data_out = slot_valid[3] ? slot_state[3] : 8'h00;
            8'h41: data_out = slot_pos[3][7:0]; 
            8'h42: data_out = shadow_pos_h[3];

            // --- 兜底防错 ---
            default: data_out = 8'hFF; // 读没映射的地址，应该返回 FF
        endcase
    end

    // =========================================================
    // 5. 硬件极速触发电路 (绝对零延迟对比)
    // =========================================================
    // 设定常数：假设走 500 个脉冲拍照，走 2000 个脉冲剔除
    localparam TARGET_CAM    = 16'd500;
    localparam TARGET_REJECT = 16'd2000;

    // 并行对比所有有效槽位，只要有一个刚好等于目标值，立刻输出 1！
    assign camera_trig = (slot_valid[0] && slot_pos[0] == TARGET_CAM) |
                         (slot_valid[1] && slot_pos[1] == TARGET_CAM) |
                         (slot_valid[2] && slot_pos[2] == TARGET_CAM) |
                         (slot_valid[3] && slot_pos[3] == TARGET_CAM) ;

    // 剔除触发：不仅要位置到，而且状态必须是不良品 (比如我们定 0xEE 代表不良品)
    assign reject_trig = (slot_valid[0] && slot_pos[0] == TARGET_REJECT && slot_state[0] == 8'hEE) |
                         (slot_valid[1] && slot_pos[1] == TARGET_REJECT && slot_state[1] == 8'hEE) |
                         (slot_valid[2] && slot_pos[2] == TARGET_REJECT && slot_state[2] == 8'hEE) |
                         (slot_valid[3] && slot_pos[3] == TARGET_REJECT && slot_state[3] == 8'hEE) ;

endmodule