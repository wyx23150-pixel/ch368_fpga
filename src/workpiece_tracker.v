module workpiece_tracker(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] current_enc,  
    input  wire        sensor_in,
    input  wire        hw_estop,
    
    output wire [7:0]  cam_hit_pulse, 
    output wire [3:0]  valve_hit_pulse,
    output wire [7:0]  light_delay_out,
    output wire [7:0]  blow_time_out,  
    input  wire [7:0]  addr,         
    input  wire        wr_en,        
    input  wire [7:0]  data_in,      
    input  wire        rd_en,        
    output reg  [7:0]  data_out,
    output wire        motor_en,
    output reg         motor_dir,    
    output reg  [7:0]  motor_speed   
);

    reg [7:0] cam_hit_internal;
    reg [3:0] valve_hit_internal;
    reg       motor_en_internal;

    // =======================================================
    // 传感器去抖动处理
    // =======================================================
    reg sensor_d1, sensor_d2, sensor_stable;               
    reg [18:0] debounce_cnt;         
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_d1 <= 0; sensor_d2 <= 0; sensor_stable <= 0; debounce_cnt <= 0;
        end else begin
            sensor_d1 <= sensor_in; sensor_d2 <= sensor_d1;
            if (sensor_d2 != sensor_stable) begin
                debounce_cnt <= debounce_cnt + 1'b1;
                if (debounce_cnt >= 19'd200) begin sensor_stable <= sensor_d2; debounce_cnt <= 0; end
            end else debounce_cnt <= 0; 
        end
    end
    reg sensor_stable_d1;
    always @(posedge clk) sensor_stable_d1 <= sensor_stable;
    wire new_workpiece_pulse = (sensor_stable == 1'b1 && sensor_stable_d1 == 1'b0);

    // =======================================================
    // 入料信号缓存机制（解决扫描期间入料丢失问题）
    // =======================================================
    reg        pending_new_workpiece;  // 缓存入料请求标志
    reg [31:0] pending_enc;            // 缓存入料时的编码器位置

    // 在任何状态下检测入料并缓存
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_new_workpiece <= 1'b0;
            pending_enc <= 32'h0;
        end else begin
            if (new_workpiece_pulse) begin
                pending_new_workpiece <= 1'b1;
                pending_enc <= global_enc;
            end
            // 写入成功后清除缓存
            else if (state == S_WRITE_NEW && ram_wren_b) begin
                pending_new_workpiece <= 1'b0;
            end
        end
    end

    // =======================================================
    // 目标位置参数
    // =======================================================
    reg [31:0] target_cam [0:7];     
    reg [31:0] target_valve [0:3];
    reg [7:0]  light_delay_ms = 8'd2;        
    reg [7:0]  blow_time_ms   = 8'd50;       
    assign light_delay_out = light_delay_ms;
    assign blow_time_out   = blow_time_ms;

    reg [31:0] min_spacing;
    reg [7:0]  sys_ctrl_reg;
    reg [7:0]  valve_select_reg;
    reg [31:0] last_accepted_enc;

    reg [31:0] global_enc;
    always @(posedge clk) global_enc <= current_enc;

    reg [15:0] wp_index; 

    wire [12:0] ram_addr_a; 
    wire [7:0]  ram_data_a_out;
    reg         ram_wren_a;
    
    assign ram_addr_a = {wp_index[9:0], addr[2:0]}; 

    reg  [9:0]  ram_addr_b; 
    reg  [63:0] ram_data_b_in;
    wire [63:0] ram_data_b_out;
    reg         ram_wren_b;

    workpieces_ram u_ram (
        .clock      (clk),
        .data_a     (data_in), .address_a  (ram_addr_a), .wren_a (ram_wren_a), .q_a (ram_data_a_out),
        .data_b     (ram_data_b_in), .address_b  (ram_addr_b), .wren_b (ram_wren_b), .q_b (ram_data_b_out)
    );

    reg [9:0]  wp_head; 

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            motor_en_internal <= 0; motor_dir <= 1; motor_speed <= 0;
            light_delay_ms <= 8'd2; blow_time_ms <= 8'd50;
            for(i=0; i<8; i=i+1) target_cam[i] <= 32'd5000 + (i*1000);
            for(i=0; i<4; i=i+1) target_valve[i] <= 32'd20000 + (i*1000);
            ram_wren_a <= 1'b0; wp_index <= 0;
            min_spacing <= 32'd50;
            sys_ctrl_reg <= 8'h00;
            valve_select_reg <= 8'h00;
            last_accepted_enc <= 32'h8000_0000;
        end else begin
            ram_wren_a <= 1'b0; 
            if (wr_en) begin
                if (addr >= 8'h20 && addr <= 8'h3F) begin
                    case (addr[1:0])
                        2'd0: target_cam[addr[4:2]][7:0]   <= data_in;
                        2'd1: target_cam[addr[4:2]][15:8]  <= data_in;
                        2'd2: target_cam[addr[4:2]][23:16] <= data_in;
                        2'd3: target_cam[addr[4:2]][31:24] <= data_in;
                    endcase
                end
                else if (addr >= 8'h40 && addr <= 8'h4F) begin
                    case (addr[1:0])
                        2'd0: target_valve[addr[3:2]][7:0]   <= data_in;
                        2'd1: target_valve[addr[3:2]][15:8]  <= data_in;
                        2'd2: target_valve[addr[3:2]][23:16] <= data_in;
                        2'd3: target_valve[addr[3:2]][31:24] <= data_in;
                    endcase
                end
                else if (addr == 8'h7E) wp_index[15:8] <= data_in; 
                else if (addr == 8'h7F) wp_index[7:0]  <= data_in; 
                else if (addr >= 8'h80 && addr <= 8'h87) ram_wren_a <= 1'b1;       
                else begin
                    case (addr)
                        8'h02: begin motor_en_internal <= data_in[0]; motor_dir <= data_in[1]; end
                        8'h03: motor_speed <= data_in;
                        8'h0E: light_delay_ms <= data_in; 
                        8'h0F: blow_time_ms   <= data_in; 
                        8'h10: min_spacing[7:0]   <= data_in;
                        8'h11: min_spacing[15:8]  <= data_in;
                        8'h12: min_spacing[23:16] <= data_in;
                        8'h13: min_spacing[31:24] <= data_in;
                        8'h14: sys_ctrl_reg       <= data_in;
                        8'h15: valve_select_reg   <= data_in;
                    endcase
                end
            end
            
            if (state == S_WRITE_NEW) last_accepted_enc <= global_enc;
        end
    end

    always @(*) begin
        if (addr >= 8'h20 && addr <= 8'h3F) begin
            case (addr[1:0])
                2'd0: data_out = target_cam[addr[4:2]][7:0];
                2'd1: data_out = target_cam[addr[4:2]][15:8];
                2'd2: data_out = target_cam[addr[4:2]][23:16];
                2'd3: data_out = target_cam[addr[4:2]][31:24];
            endcase
        end
        else if (addr >= 8'h40 && addr <= 8'h4F) begin
            case (addr[1:0])
                2'd0: data_out = target_valve[addr[3:2]][7:0];
                2'd1: data_out = target_valve[addr[3:2]][15:8];
                2'd2: data_out = target_valve[addr[3:2]][23:16];
                2'd3: data_out = target_valve[addr[3:2]][31:24];
            endcase
        end
        else if (addr == 8'h7E) data_out = wp_index[15:8];
        else if (addr == 8'h7F) data_out = wp_index[7:0];
        else if (addr >= 8'h80 && addr <= 8'h87) data_out = ram_data_a_out; 
        else begin
            case (addr)
                8'h00: data_out = 8'h5A; 
                8'h01: data_out = wp_head[7:0];                 
                8'h0C: data_out = {6'b0, wp_head[9:8]};         
                8'h02: data_out = {6'b0, motor_dir, motor_en_internal};
                8'h03: data_out = motor_speed;
                8'h04: data_out = global_enc[7:0];       
                8'h05: data_out = global_enc[15:8];      
                8'h06: data_out = global_enc[23:16];     
                8'h07: data_out = global_enc[31:24];
                8'h0E: data_out = light_delay_ms;
                8'h0F: data_out = blow_time_ms;
                8'h10: data_out = min_spacing[7:0];
                8'h11: data_out = min_spacing[15:8];
                8'h12: data_out = min_spacing[23:16];
                8'h13: data_out = min_spacing[31:24];
                8'h14: data_out = sys_ctrl_reg;
                8'h15: data_out = valve_select_reg;
                default: data_out = 8'hFF;
            endcase
        end
    end

    localparam S_IDLE       = 3'd0;
    localparam S_WRITE_NEW  = 3'd1;
    localparam S_SWEEP_RD   = 3'd2;
    localparam S_SWEEP_WAIT = 3'd3;
    localparam S_SWEEP_CALC = 3'd4;
    localparam S_SWEEP_WR   = 3'd5;

    reg [2:0]  state;
    reg [31:0] last_checked_enc;  
    reg [9:0]  sweep_idx;         

    wire [7:0]  wp_status  = ram_data_b_out[7:0];    
    wire [31:0] wp_abs_pos = ram_data_b_out[39:8];   
    wire [31:0] rel_pos    = global_enc - wp_abs_pos; 

    // =======================================================
    // 主状态机（带入料缓存优先处理）
    // =======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; last_checked_enc <= 0; wp_head <= 0; sweep_idx <= 0;
            ram_wren_b <= 0; 
            cam_hit_internal <= 0; valve_hit_internal <= 0;
        end else begin
            ram_wren_b <= 0; 
            cam_hit_internal <= 0; valve_hit_internal <= 0; 

            case (state)
                // =======================================================
                // S_IDLE: 优先处理缓存的入料，再处理新入料，最后启动扫描
                // =======================================================
                S_IDLE: begin
                    // 1. 优先处理缓存的入料请求
                    if (pending_new_workpiece) begin
                        if (pending_enc - last_accepted_enc >= min_spacing) begin
                            ram_addr_b    <= wp_head; 
                            ram_data_b_in <= {24'd0, pending_enc, 8'h01}; 
                            ram_wren_b    <= 1'b1;
                            state         <= S_WRITE_NEW;
                        end
                    end
                    // 2. 处理实时新入料（无缓存时）
                    else if (new_workpiece_pulse) begin
                        if (global_enc - last_accepted_enc >= min_spacing) begin
                            ram_addr_b    <= wp_head; 
                            ram_data_b_in <= {24'd0, global_enc, 8'h01}; 
                            ram_wren_b    <= 1'b1;
                            state         <= S_WRITE_NEW;
                        end
                    end 
                    // 3. 无入料时启动扫描
                    else if (global_enc != last_checked_enc) begin
                        last_checked_enc <= global_enc; sweep_idx <= 0; state <= S_SWEEP_RD;
                    end
                end

                S_WRITE_NEW: begin
                    wp_head <= wp_head + 1'b1; 
                    state   <= S_IDLE;
                end

                S_SWEEP_RD: begin
                    ram_addr_b <= sweep_idx; state <= S_SWEEP_WAIT;
                end
                S_SWEEP_WAIT: begin state <= S_SWEEP_CALC; end

                S_SWEEP_CALC: begin
                    if (wp_status != 0) begin
                        if (rel_pos == target_cam[0]) cam_hit_internal[0] <= 1;
                        if (rel_pos == target_cam[1]) cam_hit_internal[1] <= 1;
                        if (rel_pos == target_cam[2]) cam_hit_internal[2] <= 1;
                        if (rel_pos == target_cam[3]) cam_hit_internal[3] <= 1;
                        if (rel_pos == target_cam[4]) cam_hit_internal[4] <= 1;
                        if (rel_pos == target_cam[5]) cam_hit_internal[5] <= 1;
                        if (rel_pos == target_cam[6]) cam_hit_internal[6] <= 1;
                        if (rel_pos == target_cam[7]) cam_hit_internal[7] <= 1;

                        if (wp_status == 8'd2 && rel_pos == target_valve[0]) valve_hit_internal[0] <= 1;
                        if (wp_status == 8'd3 && rel_pos == target_valve[1]) valve_hit_internal[1] <= 1;
                        if (wp_status == 8'd4 && rel_pos == target_valve[2]) valve_hit_internal[2] <= 1;
                        if (wp_status == 8'd5 && rel_pos == target_valve[3]) valve_hit_internal[3] <= 1;

                        if (rel_pos > target_valve[3] + 32'd1000) begin
                            ram_addr_b    <= sweep_idx;
                            ram_data_b_in <= {24'd0, 32'd0, 8'h00}; 
                            ram_wren_b    <= 1'b1;
                            state         <= S_SWEEP_WR;
                        end else begin
                            if (sweep_idx == 10'd1023) state <= S_IDLE; else begin sweep_idx <= sweep_idx + 1'b1; state <= S_SWEEP_RD; end
                        end
                    end else begin
                        if (sweep_idx == 10'd1023) state <= S_IDLE; else begin sweep_idx <= sweep_idx + 1'b1; state <= S_SWEEP_RD; end
                    end
                end

                S_SWEEP_WR: begin
                    if (sweep_idx == 10'd1023) state <= S_IDLE; else begin sweep_idx <= sweep_idx + 1'b1; state <= S_SWEEP_RD; end
                end
            endcase
        end
    end

    // =======================================================
    // 频闪测试脉冲发生器
    // =======================================================
    reg [23:0] strobe_cnt;
    reg        strobe_pulse;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            strobe_cnt <= 0; strobe_pulse <= 0;
        end else begin
            if (sys_ctrl_reg[0] == 1'b1) begin
                if (strobe_cnt >= 24'd1_200_000) begin
                    strobe_cnt <= 0;
                    strobe_pulse <= 1'b1;
                end else begin
                    strobe_cnt <= strobe_cnt + 1'b1;
                    strobe_pulse <= 1'b0;
                end
            end else begin
                strobe_cnt <= 0;
                strobe_pulse <= 1'b0;
            end
        end
    end

    wire estop_active = hw_estop | sys_ctrl_reg[1];
    
    assign cam_hit_pulse = (estop_active) ? 8'h00 : 
                           (sys_ctrl_reg[0]) ? {8{strobe_pulse}} : 
                           cam_hit_internal;

    assign valve_hit_pulse = (estop_active) ? 4'h0 : 
                             (sys_ctrl_reg[2] && valve_select_reg >= 8'd1 && valve_select_reg <= 8'd4) ? 
                             (1 << (valve_select_reg - 1)) : 
                             valve_hit_internal;

    assign motor_en = (estop_active) ? 1'b0 : motor_en_internal;

endmodule