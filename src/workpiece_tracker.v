module workpiece_tracker(
    input  wire        clk,          // 系统时钟 (24MHz)
    input  wire        rst_n,        // 复位信号
    
    // 1. 物理世界信号
    input  wire [15:0] current_enc,  // 实时编码器数值
    input  wire        sensor_in,    // 光电传感器输入
    
    // 【修改/新增】：输出统一高电平有效
    output wire        light_trig,   // 输出给光源控制器
    output wire        camera_trig,  // 输出给相机的硬件触发脉冲
    output wire        qual_trig,    // 输出给良品气阀 (新增)
    output wire        reject_trig,  // 输出给次品气阀

    // 2. CH368 总线通信接口
    input  wire [7:0]  addr,         
    input  wire        wr_en,        
    input  wire [7:0]  data_in,      
    input  wire        rd_en,        
    output reg  [7:0]  data_out,

    // 3. 输出给电机驱动模块的控制信号
    output reg         motor_en,     // 电机使能
    output reg         motor_dir,    // 电机方向
    output reg  [7:0]  motor_speed   // 电机速度
);

    // =========================================================
    // 模块 A：输入滤波 (20ms 完美硬件消抖 - 修复了缺少0的bug)
    // =========================================================
    reg sensor_d1, sensor_d2;
    reg sensor_stable;               
    reg [18:0] debounce_cnt;         

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_d1 <= 0; sensor_d2 <= 0;
            sensor_stable <= 0;
            debounce_cnt <= 0;
        end else begin
            sensor_d1 <= sensor_in; 
            sensor_d2 <= sensor_d1;
            if (sensor_d2 != sensor_stable) begin
                debounce_cnt <= debounce_cnt + 1'b1;
                if (debounce_cnt >= 19'd480_00) begin // 修复：2ms
                    sensor_stable <= sensor_d2;
                    debounce_cnt  <= 0;
                end
            end else begin
                debounce_cnt <= 0; 
            end
        end
    end
    
    reg sensor_stable_d1;
    always @(posedge clk) sensor_stable_d1 <= sensor_stable;
    wire new_workpiece_pulse = (sensor_stable == 1'b1 && sensor_stable_d1 == 1'b0);

    // 编码器相对位移计算
    reg [15:0] prev_enc;
    always @(posedge clk) prev_enc <= current_enc;
    wire [15:0] enc_diff = current_enc - prev_enc; 

    // =========================================================
    // 模块 B：全局动态寄存器 (含新增的延时和脉宽)
    // =========================================================
// =========================================================
    // 模块 B：全局动态寄存器 (含新增的延时和脉宽)
    // =========================================================
    // 【终极双重保险】：直接在声明时赋初值！
    // 这样无论板子有没有上电自动复位电路，只要 FPGA 配置完成，瞬间就是这些值！
    reg [15:0] target_cam     = 16'd500;     // 动态拍照位置，默认 500
    reg [15:0] target_reject  = 16'd2000;    // 动态吹气位置，默认 2000
    reg [7:0]  light_delay_ms = 8'd2;        // 光源提前时间，默认 2ms
    reg [7:0]  blow_time_ms   = 8'd50;       // 气阀吹气时间，默认 50ms

    // =========================================================
    // 模块 C：核心硬件队列 (5 个槽位)
    // =========================================================
    reg        slot_valid [0:4];     
    reg [7:0]  slot_state [0:4];     
    reg [15:0] slot_pos   [0:4];     
    reg [7:0]  shadow_pos_h [0:4];   

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            motor_en <= 0; motor_dir <= 1; motor_speed <= 8'd0;
            target_cam    <= 16'd500;    
            target_reject <= 16'd2000; 
            light_delay_ms <= 8'd2;      // 默认光源提前 2ms
            blow_time_ms   <= 8'd50;     // 默认吹气 50ms
            
            for (i=0; i<5; i=i+1) begin
                slot_valid[i] <= 0; slot_state[i] <= 0; slot_pos[i] <= 0;
            end
        end 
        else begin
            // 1. 相对计步器累加
            for (i=0; i<5; i=i+1) begin
                if (slot_valid[i]) slot_pos[i] <= slot_pos[i] + enc_diff;
            end

            // 2. 入料分配 (5路级联)
            if (new_workpiece_pulse) begin
                if      (!slot_valid[0]) begin slot_valid[0] <= 1; slot_pos[0] <= 0; slot_state[0] <= 8'h01; end
                else if (!slot_valid[1]) begin slot_valid[1] <= 1; slot_pos[1] <= 0; slot_state[1] <= 8'h01; end
                else if (!slot_valid[2]) begin slot_valid[2] <= 1; slot_pos[2] <= 0; slot_state[2] <= 8'h01; end
                else if (!slot_valid[3]) begin slot_valid[3] <= 1; slot_pos[3] <= 0; slot_state[3] <= 8'h01; end
                else if (!slot_valid[4]) begin slot_valid[4] <= 1; slot_pos[4] <= 0; slot_state[4] <= 8'h01; end
            end
            
            // 3. 档案销毁
            for (i=0; i<5; i=i+1) begin
                if (slot_valid[i] && slot_pos[i] > (target_reject + 16'd1000)) begin
                    slot_valid[i] <= 0; slot_pos[i] <= 0; slot_state[i] <= 0;
                end
            end

            // 4. 电脑写操作 (增加 0x10 和 0x11)
            if (wr_en) begin
                case (addr)
                    8'h02: begin motor_en <= data_in[0]; motor_dir <= data_in[1]; end
                    8'h03: motor_speed <= data_in;
                    8'h06: target_cam[7:0] <= data_in;
                    8'h07: target_cam[15:8] <= data_in;
                    8'h09: target_reject[7:0] <= data_in;
                    8'h0A: target_reject[15:8] <= data_in;
                    
                    8'h10: light_delay_ms <= data_in; // 设延时
                    8'h11: blow_time_ms   <= data_in; // 设吹气时长
                    
                    8'h23: slot_state[0] <= data_in; 
                    8'h2B: slot_state[1] <= data_in; 
                    8'h33: slot_state[2] <= data_in;
                    8'h3B: slot_state[3] <= data_in;
                    8'h43: slot_state[4] <= data_in;
                endcase
            end
        end
    end

    // =========================================================
    // 模块 D：读操作与影子锁存器 
    // =========================================================
    reg rd_en_d1, rd_en_d2;
    always @(posedge clk) begin rd_en_d1 <= rd_en; rd_en_d2 <= rd_en_d1; end
    wire rd_rising_edge = (rd_en_d1 == 1'b1 && rd_en_d2 == 1'b0);

    always @(posedge clk) begin
        if (rd_rising_edge) begin
            if (addr == 8'h21) shadow_pos_h[0] <= slot_pos[0][15:8];
            if (addr == 8'h29) shadow_pos_h[1] <= slot_pos[1][15:8];
            if (addr == 8'h31) shadow_pos_h[2] <= slot_pos[2][15:8];
            if (addr == 8'h39) shadow_pos_h[3] <= slot_pos[3][15:8];
            if (addr == 8'h41) shadow_pos_h[4] <= slot_pos[4][15:8];
        end
    end

    always @(*) begin
        case (addr)
            8'h00: data_out = 8'h5A; 
            8'h02: data_out = {6'b0, motor_dir, motor_en};
            8'h03: data_out = motor_speed;
            8'h06: data_out = target_cam[7:0];
            8'h07: data_out = target_cam[15:8];
            8'h09: data_out = target_reject[7:0];
            8'h0A: data_out = target_reject[15:8];
            
            8'h10: data_out = light_delay_ms;
            8'h11: data_out = blow_time_ms;

            8'h20: data_out = slot_valid[0] ? slot_state[0] : 8'h00; 8'h21: data_out = slot_pos[0][7:0]; 8'h22: data_out = shadow_pos_h[0];
            8'h28: data_out = slot_valid[1] ? slot_state[1] : 8'h00; 8'h29: data_out = slot_pos[1][7:0]; 8'h2A: data_out = shadow_pos_h[1];
            8'h30: data_out = slot_valid[2] ? slot_state[2] : 8'h00; 8'h31: data_out = slot_pos[2][7:0]; 8'h32: data_out = shadow_pos_h[2];
            8'h38: data_out = slot_valid[3] ? slot_state[3] : 8'h00; 8'h39: data_out = slot_pos[3][7:0]; 8'h3A: data_out = shadow_pos_h[3];
            8'h40: data_out = slot_valid[4] ? slot_state[4] : 8'h00; 8'h41: data_out = slot_pos[4][7:0]; 8'h42: data_out = shadow_pos_h[4];
            default: data_out = 8'hFF; 
        endcase
    end

    // =========================================================
    // 模块 E：双气阀分拣 与 光源/相机智能联动时序控制
    // =========================================================
    
    // 命中信号检测
    wire cam_hit = (slot_valid[0] && slot_pos[0] == target_cam) |
                   (slot_valid[1] && slot_pos[1] == target_cam) |
                   (slot_valid[2] && slot_pos[2] == target_cam) |
                   (slot_valid[3] && slot_pos[3] == target_cam) |
                   (slot_valid[4] && slot_pos[4] == target_cam) ;

    // 良品命中 (0xAA)
    wire qual_hit = (slot_valid[0] && slot_pos[0] == target_reject && slot_state[0] == 8'hAA) |
                    (slot_valid[1] && slot_pos[1] == target_reject && slot_state[1] == 8'hAA) |
                    (slot_valid[2] && slot_pos[2] == target_reject && slot_state[2] == 8'hAA) |
                    (slot_valid[3] && slot_pos[3] == target_reject && slot_state[3] == 8'hAA) |
                    (slot_valid[4] && slot_pos[4] == target_reject && slot_state[4] == 8'hAA) ;

    // 次品命中 (0xEE)
    wire rej_hit  = (slot_valid[0] && slot_pos[0] == target_reject && slot_state[0] == 8'hEE) |
                    (slot_valid[1] && slot_pos[1] == target_reject && slot_state[1] == 8'hEE) |
                    (slot_valid[2] && slot_pos[2] == target_reject && slot_state[2] == 8'hEE) |
                    (slot_valid[3] && slot_pos[3] == target_reject && slot_state[3] == 8'hEE) |
                    (slot_valid[4] && slot_pos[4] == target_reject && slot_state[4] == 8'hEE) ;

    // 使用硬件乘法器，将毫秒(ms)转换为24MHz下的时钟周期数
    wire [23:0] delay_cycles = light_delay_ms * 24'd24_000;
    wire [23:0] blow_cycles  = blow_time_ms   * 24'd24_000;

    // 独立计时器
    reg [23:0] light_timer;      // 光源总时长
    reg [23:0] cam_delay_timer;  // 约束相机的“不许拍”倒计时
    reg [23:0] qual_timer;       // 良品气缸秒表
    reg [23:0] rej_timer;        // 次品气缸秒表

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            light_timer <= 0; cam_delay_timer <= 0;
            qual_timer <= 0;  rej_timer <= 0;
        end else begin
            // 【相机与光源时序逻辑】
            if (cam_hit) begin
                // 光源立刻亮起，并且一直亮到相机曝光结束(延迟时间 + 相机固定的10ms脉冲)
                light_timer     <= delay_cycles + 24'd240_000; 
                // 相机先憋着不拍，倒计时 delay_cycles
                cam_delay_timer <= delay_cycles;
            end else begin
                if (light_timer > 0)     light_timer     <= light_timer - 1'b1;
                if (cam_delay_timer > 0) cam_delay_timer <= cam_delay_timer - 1'b1;
            end

            // 【双气阀逻辑】
            if (qual_hit) qual_timer <= blow_cycles;
            else if (qual_timer > 0) qual_timer <= qual_timer - 1'b1;

            if (rej_hit)  rej_timer <= blow_cycles;
            else if (rej_timer > 0)  rej_timer <= rej_timer - 1'b1;
        end
    end

    // 输出映射：全部高电平有效
    assign light_trig  = (light_timer > 0) ? 1'b1 : 1'b0;
    // 相机触发条件：光源开启，并且“不许拍倒计时”结束了
    assign camera_trig = (light_timer > 0 && cam_delay_timer == 0) ? 1'b1 : 1'b0;
    
    assign qual_trig   = (qual_timer > 0) ? 1'b1 : 1'b0;
    assign reject_trig = (rej_timer > 0)  ? 1'b1 : 1'b0;

endmodule