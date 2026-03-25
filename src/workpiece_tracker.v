module workpiece_tracker(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] current_enc,  
    input  wire        sensor_in,    
    output reg  [7:0]  cam_hit_pulse, 
    output reg         qual_hit_pulse, 
    output reg         rej_hit_pulse,  
    output wire [7:0]  light_delay_out,
    output wire [7:0]  blow_time_out,  
    input  wire [7:0]  addr,         
    input  wire        wr_en,        
    input  wire [7:0]  data_in,      
    input  wire        rd_en,        
    output reg  [7:0]  data_out,
    output reg         motor_en,     
    output reg         motor_dir,    
    output reg  [7:0]  motor_speed   
);

    reg sensor_d1, sensor_d2, sensor_stable;               
    reg [18:0] debounce_cnt;         
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_d1 <= 0; sensor_d2 <= 0; sensor_stable <= 0; debounce_cnt <= 0;
        end else begin
            sensor_d1 <= sensor_in; sensor_d2 <= sensor_d1;
            if (sensor_d2 != sensor_stable) begin
                debounce_cnt <= debounce_cnt + 1'b1;
                if (debounce_cnt >= 19'd480_00) begin 
                    sensor_stable <= sensor_d2; debounce_cnt <= 0;
                end
            end else debounce_cnt <= 0; 
        end
    end
    reg sensor_stable_d1;
    always @(posedge clk) sensor_stable_d1 <= sensor_stable;
    wire new_workpiece_pulse = (sensor_stable == 1'b1 && sensor_stable_d1 == 1'b0);

    reg [31:0] target_cam [0:7];     
    reg [31:0] target_reject  = 32'd20000;    
    reg [7:0]  light_delay_ms = 8'd2;        
    reg [7:0]  blow_time_ms   = 8'd50;       
    assign light_delay_out = light_delay_ms;
    assign blow_time_out   = blow_time_ms;

    reg [31:0] global_enc;
    always @(posedge clk) global_enc <= current_enc;

    wire [10:0] ram_addr_a;
    wire [7:0]  ram_data_a_out;
    reg         ram_wren_a;
    assign ram_addr_a = {4'd0, addr[6:0]}; 

    reg  [7:0]  ram_addr_b;
    reg  [63:0] ram_data_b_in;
    wire [63:0] ram_data_b_out;
    reg         ram_wren_b;

    workpieces_ram u_ram (
        .clock      (clk),
        .data_a     (data_in), .address_a  (ram_addr_a), .wren_a (ram_wren_a), .q_a (ram_data_a_out),
        .data_b     (ram_data_b_in), .address_b  (ram_addr_b), .wren_b (ram_wren_b), .q_b (ram_data_b_out)
    );

    // 【修改点1】：暴露 wp_head (最新槽位) 和 global_enc
    reg [3:0]  wp_head; 

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            motor_en <= 0; motor_dir <= 1; motor_speed <= 0;
            target_reject <= 32'd20000; light_delay_ms <= 8'd2; blow_time_ms <= 8'd50;
            for(i=0; i<8; i=i+1) target_cam[i] <= 32'd5000 + (i*1000);
            ram_wren_a <= 1'b0;
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
                else if (addr >= 8'h80 && addr <= 8'hFF) ram_wren_a <= 1'b1;       
                else begin
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

    always @(*) begin
        if (addr >= 8'h20 && addr <= 8'h3F) begin
            case (addr[1:0])
                2'd0: data_out = target_cam[addr[4:2]][7:0];
                2'd1: data_out = target_cam[addr[4:2]][15:8];
                2'd2: data_out = target_cam[addr[4:2]][23:16];
                2'd3: data_out = target_cam[addr[4:2]][31:24];
            endcase
        end
        else if (addr >= 8'h80 && addr <= 8'hFF) data_out = ram_data_a_out; 
        else begin
            case (addr)
                8'h00: data_out = 8'h5A; 
                8'h01: data_out = {4'b0, wp_head};       // 开放：让 C# 知道哪个槽位是最新的！
                8'h02: data_out = {6'b0, motor_dir, motor_en};
                8'h03: data_out = motor_speed;
                8'h04: data_out = global_enc[7:0];       // 开放：全局编码器 0
                8'h05: data_out = global_enc[15:8];      // 开放：全局编码器 1
                8'h06: data_out = global_enc[23:16];     // 开放：全局编码器 2
                8'h07: data_out = global_enc[31:24];     // 开放：全局编码器 3
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

    localparam S_IDLE       = 3'd0;
    localparam S_WRITE_NEW  = 3'd1;
    localparam S_SWEEP_RD   = 3'd2;
    localparam S_SWEEP_WAIT = 3'd3;
    localparam S_SWEEP_CALC = 3'd4;
    localparam S_SWEEP_WR   = 3'd5;

    reg [2:0]  state;
    reg [31:0] last_checked_enc;  
    reg [3:0]  sweep_idx;         

    // 【修改点2】：完美字节对齐 C#
    wire [7:0]  wp_status  = ram_data_b_out[7:0];    // 状态对齐到最低字节 (0x80)
    wire [31:0] wp_abs_pos = ram_data_b_out[39:8];   // 绝对坐标对齐到 0x81~0x84
    wire [31:0] rel_pos    = global_enc - wp_abs_pos; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; last_checked_enc <= 0; wp_head <= 0; sweep_idx <= 0;
            ram_wren_b <= 0; cam_hit_pulse <= 0; qual_hit_pulse <= 0; rej_hit_pulse <= 0;
        end else begin
            ram_wren_b <= 0; cam_hit_pulse <= 0; qual_hit_pulse <= 0; rej_hit_pulse <= 0; 

            case (state)
                S_IDLE: begin
                    if (new_workpiece_pulse) begin
                        ram_addr_b    <= {4'd0, wp_head}; 
                        // 【修改点3】：数据打包顺序调整！先压入 global_enc，最后压入状态字 0x01
                        ram_data_b_in <= {24'd0, global_enc, 8'h01}; 
                        ram_wren_b    <= 1'b1;
                        state         <= S_WRITE_NEW;
                    end 
                    else if (global_enc != last_checked_enc) begin
                        last_checked_enc <= global_enc; sweep_idx <= 0; state <= S_SWEEP_RD;
                    end
                end

                S_WRITE_NEW: begin
                    wp_head <= wp_head + 1'b1; 
                    state   <= S_IDLE;
                end

                S_SWEEP_RD: begin
                    ram_addr_b <= {4'd0, sweep_idx}; state <= S_SWEEP_WAIT;
                end
                S_SWEEP_WAIT: begin state <= S_SWEEP_CALC; end

                S_SWEEP_CALC: begin
                    if (wp_status != 0) begin
                        if (rel_pos == target_cam[0]) cam_hit_pulse[0] <= 1;
                        if (rel_pos == target_cam[1]) cam_hit_pulse[1] <= 1;
                        if (rel_pos == target_cam[2]) cam_hit_pulse[2] <= 1;
                        if (rel_pos == target_cam[3]) cam_hit_pulse[3] <= 1;
                        if (rel_pos == target_cam[4]) cam_hit_pulse[4] <= 1;
                        if (rel_pos == target_cam[5]) cam_hit_pulse[5] <= 1;
                        if (rel_pos == target_cam[6]) cam_hit_pulse[6] <= 1;
                        if (rel_pos == target_cam[7]) cam_hit_pulse[7] <= 1;

                        if (rel_pos == target_reject) begin
                            if (wp_status == 8'hAA) qual_hit_pulse <= 1;
                            if (wp_status == 8'hEE) rej_hit_pulse <= 1;
                        end

                        if (rel_pos > target_reject + 32'd1000) begin
                            ram_addr_b    <= {4'd0, sweep_idx};
                            // 销毁时也要保持格式：状态位清零
                            ram_data_b_in <= {24'd0, 32'd0, 8'h00}; 
                            ram_wren_b    <= 1'b1;
                            state         <= S_SWEEP_WR;
                        end else begin
                            if (sweep_idx == 4'd15) state <= S_IDLE; else begin sweep_idx <= sweep_idx + 1'b1; state <= S_SWEEP_RD; end
                        end
                    end else begin
                        if (sweep_idx == 4'd15) state <= S_IDLE; else begin sweep_idx <= sweep_idx + 1'b1; state <= S_SWEEP_RD; end
                    end
                end

                S_SWEEP_WR: begin
                    if (sweep_idx == 4'd15) state <= S_IDLE; else begin sweep_idx <= sweep_idx + 1'b1; state <= S_SWEEP_RD; end
                end
            endcase
        end
    end
endmodule