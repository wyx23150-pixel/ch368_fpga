// 文件名：top.v (工程的 Absolute Top-Level)
module top(
    // 1. 全局时钟与复位
    input  wire        clk_50m,    
    input  wire        rst_n,      

    // 2. CH368 物理总线引脚
    input  wire        ch368_cs_n, 
    input  wire        ch368_rd_n, 
    input  wire        ch368_wr_n, 
    input  wire [7:0]  ch368_addr, // 注意：这里要确保你在 Pin Planner 里把 A4~A7 也分配了物理引脚
    inout  wire [7:0]  ch368_data, 

    // 3. 编码器引脚
    input  wire        enc_a,      
    input  wire        enc_b,      
    input  wire        enc_z,      

    // 4. 【新增】：外设引脚
    input  wire        sensor_in,  // 接光电传感器 (你需要分配一个物理引脚)
    output wire        camera_out, // 接相机触发 (你需要分配一个物理引脚)
    output wire        reject_out  // 接剔除气缸继电器 (你需要分配一个物理引脚)
);

    // =======================================================
    // 内部总线连线 (主板上的铜线)
    // =======================================================
    wire [31:0] w_encoder_val; // 编码器送出来的绝对数值
    
    // 桥接器送出来的内部总线信号
    wire [7:0]  w_sys_addr;
    wire [7:0]  w_sys_data_out;
    wire        w_sys_wr_en;
    wire        w_sys_rd_en;
    wire [7:0]  w_sys_data_in;

    // =======================================================
    // 模块 1：编码器采集
    // =======================================================
    encoder u_encoder (
        .clk   (clk_50m),    
        .rst_n (1'b1),        // 强制不复位
        .enc_a (enc_a),      
        .enc_b (enc_b),      
        .enc_z (1'b0),        // 【极度重要】：强制接 0！屏蔽 Z 相清零，利用补码自然溢出机制
        .count (w_encoder_val) 
    );

    // =======================================================
    // 模块 2：CH368 总线桥接器
    // =======================================================
    ch368_interface u_ch368 (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
        .ch368_cs_n   (ch368_cs_n),
        .ch368_rd_n   (ch368_rd_n),
        .ch368_wr_n   (ch368_wr_n),
        .ch368_addr   (ch368_addr),
        .ch368_data   (ch368_data),
        // 连到内部总线上
        .sys_addr     (w_sys_addr),
        .sys_data_out (w_sys_data_out),
        .sys_wr_en    (w_sys_wr_en),
        .sys_rd_en    (w_sys_rd_en),
        .sys_data_in  (w_sys_data_in)
    );

    // =======================================================
    // 模块 3：【核心】工件追踪与触发引擎
    // =======================================================
    workpiece_tracker u_tracker (
        .clk          (clk_50m),
        .rst_n        (1'b1),            // 强制不复位
        .current_enc  (w_encoder_val[15:0]), // 只要低 16 位
        
        .sensor_in    (sensor_in),       // 传感器输入
        .camera_trig  (camera_out),      // 相机触发输出
        .reject_trig  (reject_out),      // 气缸触发输出
        
        // 接内部通信总线
        .addr         (w_sys_addr),
        .wr_en        (w_sys_wr_en),
        .data_in      (w_sys_data_out),
        .rd_en        (w_sys_rd_en),
        .data_out     (w_sys_data_in)
    );

endmodule