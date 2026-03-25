module workpiece_tracker(
    input  wire        clk,          // 系统时钟 (24MHz)
    input  wire        rst_n,        // 复位信号
    
    // 1. 物理世界信号
    input  wire [31:0] current_enc,  
    input  wire        sensor_in,    
    
    // 2. 输出给外设通道的“开火脉冲”和配置
    output wire [7:0]  cam_hit_pulse, // 【升级】：8个相机的独立触发脉冲
    output wire        qual_hit_pulse, 
    output wire        rej_hit_pulse,  
    
    output wire [7:0]  light_delay_out,
    output wire [7:0]  blow_time_out,  

    // 3. CH368 总线通信接口
    input  wire [7:0]  addr,         
    input  wire        wr_en,        
    input  wire [7:0]  data_in,      
    input  wire        rd_en,        
    output reg  [7:0]  data_out,

    // 4. 电机驱动
    output reg         motor_en,     
    output reg         motor_dir,    
    output reg  [7:0]  motor_speed   
);

    // =========================================================
    // 模块 A：输入滤波 (20ms 硬件消抖)
    // =========================================================
    reg sensor_d1, sensor_d2;
    reg sensor_stable;               
    reg [18:0] debounce_cnt;         

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_d1 <= 0; sensor_d2 <= 0;
            sensor_stable <= 0; debounce_cnt <= 0;
        end else begin
            sensor_d1 <= sensor_in; sensor_d2 <= sensor_d1;
            if (sensor_d2 != sensor_stable) begin
                debounce_cnt <= debounce_cnt + 1'b1;
                if (debounce_cnt >= 19'd480_00) begin 
                    sensor_stable <= sensor_d2;
                    debounce_cnt  <= 0;
                end
            end else debounce_cnt <= 0; 
        end
    end
    reg sensor_stable_d1;
    always @(posedge clk) sensor_stable_d1 <= sensor_stable;
    wire new_workpiece_pulse = (sensor_stable == 1'b1 && sensor_stable_d1 == 1'b0);

    // 编码器相对位移计算 (修复后的32位减法)
    reg [31:0] prev_enc;
    always @(posedge clk) prev_enc <= current_enc;
    wire [15:0] diff_16 = current_enc[15:0] - prev_enc[15:0];
    wire [31:0] enc_diff = {{16{diff_16[15]}}, diff_16}; 

    // =========================================================
    // 模块 B：全局动态寄存器
    // =========================================================
    // 【升级】：相机目标位置变成了 8 个 32位的数组！
    reg [31:0] target_cam [0:7];     
    
    reg [31:0] target_reject  = 32'd20000;    
    reg [7:0]  light_delay_ms = 8'd2;        
    reg [7:0]  blow_time_ms   = 8'd50;       
    
    assign light_delay_out = light_delay_ms;
    assign blow_time_out   = blow_time_ms;

    // =========================================================
    // 模块 C：核心硬件队列 
    // =========================================================
    reg        slot_valid [0:7];     
    reg [7:0]  slot_state [0:7];     
    reg [31:0] slot_pos   [0:7];     
    reg [23:0] shadow_pos [0:7];    

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            motor_en <= 0; motor_dir <= 1; motor_speed <= 8'd0;
            target_reject  <= 32'd20000; 
            light_delay_ms <= 8'd2;      
            blow_time_ms   <= 8'd50;     
            for (i=0; i<8; i=i+1) begin
                // 初始化时，给 8 个相机稍微不同的错开位置（可被上位机覆盖）
                target_cam[i] <= 32'd5000 + (i * 1000); 
                slot_valid[i] <= 0; slot_state[i] <= 0; slot_pos[i] <= 0;
            end
        end 
        else begin
            // 1. 位置累加
            for (i=0; i<8; i=i+1) begin
                if (slot_valid[i]) slot_pos[i] <= slot_pos[i] + enc_diff;
            end

            // 2. 入料分配
            if (new_workpiece_pulse) begin
                if      (!slot_valid[0]) begin slot_valid[0] <= 1; slot_pos[0] <= 0; slot_state[0] <= 8'h01; end
                else if (!slot_valid[1]) begin slot_valid[1] <= 1; slot_pos[1] <= 0; slot_state[1] <= 8'h01; end
                else if (!slot_valid[2]) begin slot_valid[2] <= 1; slot_pos[2] <= 0; slot_state[2] <= 8'h01; end
                else if (!slot_valid[3]) begin slot_valid[3] <= 1; slot_pos[3] <= 0; slot_state[3] <= 8'h01; end
                else if (!slot_valid[4]) begin slot_valid[4] <= 1; slot_pos[4] <= 0; slot_state[4] <= 8'h01; end
                else if (!slot_valid[5]) begin slot_valid[5] <= 1; slot_pos[5] <= 0; slot_state[5] <= 8'h01; end
                else if (!slot_valid[6]) begin slot_valid[6] <= 1; slot_pos[6] <= 0; slot_state[6] <= 8'h01; end
                else if (!slot_valid[7]) begin slot_valid[7] <= 1; slot_pos[7] <= 0; slot_state[7] <= 8'h01; end
            end
            
            // 3. 档案销毁
            for (i=0; i<8; i=i+1) begin
                if (slot_valid[i] && slot_pos[i] > (target_reject + 32'd1000)) begin
                    slot_valid[i] <= 0; slot_pos[i] <= 0; slot_state[i] <= 0;
                end
            end

            // 4. 【高级切片写操作】
            if (wr_en) begin
                if (addr >= 8'h20 && addr <= 8'h3F) begin
                    // 写入 8 个相机位置 (利用位运算提取相机索引 addr[4:2])
                    case (addr[1:0])
                        2'd0: target_cam[addr[4:2]][7:0]   <= data_in;
                        2'd1: target_cam[addr[4:2]][15:8]  <= data_in;
                        2'd2: target_cam[addr[4:2]][23:16] <= data_in;
                        2'd3: target_cam[addr[4:2]][31:24] <= data_in;
                    endcase
                end
                else if (addr >= 8'h80 && addr <= 8'hBF) begin
                    // 写入 8 个工件状态 (只有尾号为0的地址才写状态)
                    if (addr[2:0] == 3'd0) slot_state[addr[5:3]] <= data_in;
                end
                else begin
                    // 全局参数写入
                    case (addr)
                        8'h02: begin motor_en <= data_in[0]; motor_dir <= data_in[1]; end
                        8'h03: motor_speed <= data_in;
                        
                        8'h08: target_reject[7:0]   <= data_in;
                        8'h09: target_reject[15:8]  <= data_in;
                        8'h0A: target_reject[23:16] <= data_in;
                        8'h0B: target_reject[31:24] <= data_in;
                        
                        8'h0E: light_delay_ms <= data_in; 
                        8'h0F: blow_time_ms   <= data_in; 
                    endcase
                end
            end
        end
    end

    // =========================================================
    // 模块 D：读操作与 32位影子锁存器 (高级切片化)
    // =========================================================
    reg rd_en_d1, rd_en_d2;
    always @(posedge clk) begin rd_en_d1 <= rd_en; rd_en_d2 <= rd_en_d1; end
    wire rd_rising_edge = (rd_en_d1 == 1'b1 && rd_en_d2 == 1'b0);

    always @(posedge clk) begin
        if (rd_rising_edge) begin
            // 只要读取 0x81, 0x89... 等最低位位置，瞬间触发高位锁存
            if (addr >= 8'h80 && addr <= 8'hBF && addr[2:0] == 3'd1) begin
                shadow_pos[addr[5:3]] <= slot_pos[addr[5:3]][31:8];
            end
        end
    end

    always @(*) begin
        if (addr >= 8'h20 && addr <= 8'h3F) begin
            // 动态读出 8 个相机位置
            case (addr[1:0])
                2'd0: data_out = target_cam[addr[4:2]][7:0];
                2'd1: data_out = target_cam[addr[4:2]][15:8];
                2'd2: data_out = target_cam[addr[4:2]][23:16];
                2'd3: data_out = target_cam[addr[4:2]][31:24];
            endcase
        end
        else if (addr >= 8'h80 && addr <= 8'hBF) begin
            // 动态读出 8 个工件参数
            case (addr[2:0])
                3'd0: data_out = slot_valid[addr[5:3]] ? slot_state[addr[5:3]] : 8'h00; 
                3'd1: data_out = slot_pos[addr[5:3]][7:0];     
                3'd2: data_out = shadow_pos[addr[5:3]][7:0];   
                3'd3: data_out = shadow_pos[addr[5:3]][15:8];  
                3'd4: data_out = shadow_pos[addr[5:3]][23:16]; 
                default: data_out = 8'h00;
            endcase
        end
        else begin
            case (addr)
                8'h00: data_out = 8'h5A; 
                8'h02: data_out = {6'b0, motor_dir, motor_en};
                8'h03: data_out = motor_speed;
                
                8'h08: data_out = target_reject[7:0];
                8'h09: data_out = target_reject[15:8];
                8'h0A: data_out = target_reject[23:16];
                8'h0B: data_out = target_reject[31:24];
                
                8'h0E: data_out = light_delay_ms;
                8'h0F: data_out = blow_time_ms;
                default: data_out = 8'hFF; 
            endcase
        end
    end

    // =========================================================
    // 模块 E：硬件生成树 (极其优美的 8相机独立碰撞检测)
    // =========================================================
    wire [7:0] cam_hit_cond;
    wire [7:0] qual_matches;
    wire [7:0] rej_matches;

    genvar c, w;
    generate
        // 1. 生成 8 个相机的独立触发检测
        for (c = 0; c < 8; c = c + 1) begin : CAM_HIT_GEN
            wire [7:0] wp_cam_matches;
            for (w = 0; w < 8; w = w + 1) begin : WP_CAM_MATCH
                // 任何一个有效的工件，到达了 c号相机 的目标位置，就触发匹配
                assign wp_cam_matches[w] = (slot_valid[w] && (slot_pos[w] == target_cam[c]));
            end
            assign cam_hit_cond[c] = |wp_cam_matches; // 把 8个工件的匹配结果按位或
        end
        
        // 2. 生成气阀的触发检测
        for (w = 0; w < 8; w = w + 1) begin : VALVE_MATCH_GEN
            assign qual_matches[w] = (slot_valid[w] && (slot_pos[w] == target_reject) && (slot_state[w] == 8'hAA));
            assign rej_matches[w]  = (slot_valid[w] && (slot_pos[w] == target_reject) && (slot_state[w] == 8'hEE));
        end
    endgenerate

    wire qual_hit_cond = |qual_matches;
    wire rej_hit_cond  = |rej_matches;

    // 硬件边沿检测：生成严格的 1 拍脉冲
    reg [7:0] cam_hit_d1;
    reg       qual_hit_d1, rej_hit_d1;
    always @(posedge clk) begin
        cam_hit_d1  <= cam_hit_cond;
        qual_hit_d1 <= qual_hit_cond;
        rej_hit_d1  <= rej_hit_cond;
    end
    
    // 最终输出：8根相机线，2根气阀线
    assign cam_hit_pulse = cam_hit_cond  & ~cam_hit_d1;
    assign qual_hit_pulse = qual_hit_cond & ~qual_hit_d1;
    assign rej_hit_pulse  = rej_hit_cond  & ~rej_hit_d1;

endmodule