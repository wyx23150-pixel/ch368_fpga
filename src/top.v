// 文件名称：top.v (工程的顶层，全局架构)
module top(
    // 1. 全局时钟与复位
    input  wire        clk_50m,    
    input  wire        rst_n,      

    // 2. CH368 总线并行接口
    input  wire        ch368_cs_n, 
    input  wire        ch368_rd_n, 
    input  wire        ch368_wr_n, 
    input  wire [7:0]  ch368_addr, 
    inout  wire [7:0]  ch368_data, 

    // 3. 编码器输入
    input  wire        enc_a,      
    input  wire        enc_b,      
    input  wire        enc_z,      

    // 4. 传感器输入与输出信号/统一高电平/低电平有效
    input  wire        sensor_in,  //传感器（如光电）
    output wire [5:0]       light_out,  //光源1（对应相机前光源曝光）
    output wire [5:0]       camera_out, //相机1
    output wire [3:0]       valve_out,  //修改：4个气阀输出

    // 5. 电机等实际执行器驱动信号
    output wire        motor_pwm_out, // 接到电机驱动 PWM 输入
    output wire        motor_dir_out, // 接到电机驱动的方向端 (DIR/IN1)
    output wire        motor_en_out,   // 接到电机驱动使能端 (EN)

    // 6. 新增：杂项控制

    output wire        misc_led_r,
    output wire        misc_led_y,
    output wire        misc_led_g,
    output wire        misc_buzzer,
    output wire        misc_vibrator,  // 震动盘控制
    output wire        misc_feeder     // 下料控制

   
);

    // =======================================================
    // 内部连线与参数 (流水线间的铜线)
    // =======================================================
    wire [31:0] w_encoder_val; 
    
    // CH368 与各个模块之间的通信信号线
    wire [7:0]  w_sys_addr;
    wire [7:0]  w_sys_data_out;
    wire        w_sys_wr_en;
    wire        w_sys_rd_en;
    wire [7:0]  w_sys_data_in;
    wire [7:0]  w_misc_data_out;
    // 各个功能模块之间的通信信号
    wire        w_motor_en;
    wire        w_motor_dir;
    wire [7:0]  w_motor_speed;
 
    wire [7:0]  w_cam_hit_pulse;
    wire [7:0]  w_light_delay_ms;
    wire [7:0]  w_blow_time_out;

    wire [3:0]  w_valve_hit_pulse;  //修改：4个气阀信号


    // =======================================================
    // 模块 1：编码器采集 (正交，甚至有 Z 相)
    // =======================================================
    encoder u_encoder (
        .clk   (clk_50m),    
        .rst_n (1'b1),        
        .enc_a (enc_a),      
        .enc_b (enc_b),      
        .enc_z (1'b0),        // 清零位暂不用，上电16位依然计存   
        .count (w_encoder_val) 
    );

    // =======================================================
    // 模块 2：CH368 总线解析接口 (核心)
    // =======================================================
    ch368_interface u_ch368 (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
        .misc_data_in (w_misc_data_out),
        .ch368_cs_n   (ch368_cs_n),
        .ch368_rd_n   (ch368_rd_n),
        .ch368_wr_n   (ch368_wr_n),
        .ch368_addr   (ch368_addr),
        .ch368_data   (ch368_data),

        .sys_addr     (w_sys_addr),
        .sys_data_out (w_sys_data_out),
        .sys_wr_en    (w_sys_wr_en),
        .sys_rd_en    (w_sys_rd_en),
        .sys_data_in  (w_sys_data_in)
    );

    // =======================================================
    // 模块 3：工件的核心与触发调度
    // =======================================================
    workpiece_tracker u_tracker (
        .clk          (clk_50m),
        .rst_n        (rst_n),            
        .current_enc  (w_encoder_val), 
        
        .sensor_in    (sensor_in),       
        
        // 【重要】这里的参数都是可以动态配置的
        .cam_hit_pulse   (w_cam_hit_pulse),            
        .valve_hit_pulse  (w_valve_hit_pulse),  //修改：4个气阀   

        .light_delay_out (w_light_delay_ms),
        .blow_time_out  (w_blow_time_out),

        
        
        
        // 配置接口
        .addr         (w_sys_addr),
        .wr_en        (w_sys_wr_en),
        .data_in      (w_sys_data_out),
        .rd_en        (w_sys_rd_en),
        .data_out     (w_sys_data_in),

        // 获取上位机发来的电机信号，通过内部总线传过去
        .motor_en     (w_motor_en),
        .motor_dir    (w_motor_dir),
        .motor_speed  (w_motor_speed)
    );

    // =======================================================
    // 模块 4：PWM 电机驱动模块
    // =======================================================
    motor u_motor (
        .clk          (clk_50m),
        .rst_n        (rst_n),
        
        // 从追踪模块或上位机获取的参数
        .en_val       (w_motor_en),
        .dir_val      (w_motor_dir),
        .speed_val    (w_motor_speed),
        
        // 直接连到外部电机驱动器
        .motor_pwm    (motor_pwm_out),
        .motor_dir    (motor_dir_out),
        .motor_en     (motor_en_out)
    );

    // =======================================================
    // 模块 5：气阀模块
    // =======================================================

    // 第 1 组相机通道
    cam_light_channel u_cam_ch0 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[0]), // 详细请见 0 号通道解释
        .delay_ms      (w_light_delay_ms),    // 光源延时时间
        .light_pin     (light_out[0]),         // 接 1# 光源控制输出
        .cam_pin       (camera_out[0])         // 接 1# 相机触发输出
    );

        // 第 2 组相机通道
    cam_light_channel u_cam_ch1 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[1]), // 详细请见 0 号通道解释
        .delay_ms      (w_light_delay_ms),    // 光源延时时间
        .light_pin     (light_out[1]),         // 接 1# 光源控制输出
        .cam_pin       (camera_out[1])         // 接 1# 相机触发输出
    );
        // 第 3 组相机通道
    cam_light_channel u_cam_ch2 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[2]), // 详细请见 0 号通道解释
        .delay_ms      (w_light_delay_ms),    // 光源延时时间
        .light_pin     (light_out[2]),         // 接 1# 光源控制输出
        .cam_pin       (camera_out[2])         // 接 1# 相机触发输出
    );
        // 第 4 组相机通道
    cam_light_channel u_cam_ch3 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[3]), // 详细请见 0 号通道解释
        .delay_ms      (w_light_delay_ms),    // 光源延时时间
        .light_pin     (light_out[3]),         // 接 1# 光源控制输出
        .cam_pin       (camera_out[3])         // 接 1# 相机触发输出
    );
        // 第 5 组相机通道
    cam_light_channel u_cam_ch4 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[4]), // 详细请见 0 号通道解释
        .delay_ms      (w_light_delay_ms),    // 光源延时时间
        .light_pin     (light_out[4]),         // 接 1# 光源控制输出
        .cam_pin       (camera_out[4])         // 接 1# 相机触发输出
    );
        // 第 6 组相机通道
    cam_light_channel u_cam_ch5 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[5]), // 详细请见 0 号通道解释
        .delay_ms      (w_light_delay_ms),    // 光源延时时间
        .light_pin     (light_out[5]),         // 接 1# 光源控制输出
        .cam_pin       (camera_out[5])         // 接 1# 相机触发输出
    );
        // 第 7 组相机通道


    // 修改：4个气阀实例化
    valve_ctrl u_valve0(
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .trigger_pulse  (w_valve_hit_pulse[0]),
        .blow_ms        (w_blow_time_out),
        .valve_pin      (valve_out[0])
    );

    valve_ctrl u_valve1(
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .trigger_pulse  (w_valve_hit_pulse[1]),
        .blow_ms        (w_blow_time_out),
        .valve_pin      (valve_out[1])
    );

    valve_ctrl u_valve2(
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .trigger_pulse  (w_valve_hit_pulse[2]),
        .blow_ms        (w_blow_time_out),
        .valve_pin      (valve_out[2])
    );

    valve_ctrl u_valve3(
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .trigger_pulse  (w_valve_hit_pulse[3]),
        .blow_ms        (w_blow_time_out),
        .valve_pin      (valve_out[3])
    );

    // =======================================================
    // 模块 6：杂项 IO 控制（三色灯、蜂鸣器）
    // =======================================================
    misc_io_ctrl u_misc_io (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .addr       (w_sys_addr),
        .wr_en      (w_sys_wr_en),
        .rd_en      (w_sys_rd_en),
        .data_in    (w_sys_data_out),
        .data_out   (w_misc_data_out),
        .led_r      (misc_led_r),
        .led_y      (misc_led_y),
        .led_g      (misc_led_g),
        .buzzer     (misc_buzzer),
        .vibrator   (misc_vibrator),  // 震动盘控制
        .feeder     (misc_feeder)     // 下料控制
    );

    // =======================================================
    // 模块 7：输出端口连接
    // =======================================================



endmodule