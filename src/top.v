// 文件名：top.v (工程的最顶层，全局大管家)
module top(
    // 1. 全局时钟与复位
    input  wire        clk_50m,    
    input  wire        rst_n,      

    // 2. CH368 本地总线接口
    input  wire        ch368_cs_n, 
    input  wire        ch368_rd_n, 
    input  wire        ch368_wr_n, 
    input  wire [7:0]  ch368_addr, 
    inout  wire [7:0]  ch368_data, 

    // 3. 编码器引脚
    input  wire        enc_a,      
    input  wire        enc_b,      
    input  wire        enc_z,      

    // 4. 传感器与气缸/相机引脚/输出统一高电平/上升沿有效
    input  wire        sensor_in,  //光纤（进料）
    output wire [7:0]       light_out,  //光源1 （相机拍照前短暂曝光）
    output wire [7:0]       camera_out, //相机1
    output wire        qual_out,   //良料气阀
    output wire        rej_out, //次品气阀

    // 5. 输出给真实电机驱动器的物理引脚
    output wire        motor_pwm_out, // 接电机驱动板的 PWM 输入
    output wire        motor_dir_out, // 接电机驱动板的方向输入 (DIR/IN1)
    output wire        motor_en_out   // 接电机驱动板的使能输入 (EN)
);

    // =======================================================
    // 内部总线连线 (主板上的铜线)
    // =======================================================
    wire [31:0] w_encoder_val; 
    
    // CH368 与追踪引擎之间的通信桥梁
    wire [7:0]  w_sys_addr;
    wire [7:0]  w_sys_data_out;
    wire        w_sys_wr_en;
    wire        w_sys_rd_en;
    wire [7:0]  w_sys_data_in;

    // 追踪引擎与电机模块之间的通信线
    wire        w_motor_en;
    wire        w_motor_dir;
    wire [7:0]  w_motor_speed;
 
    wire [7:0]  w_cam_hit_pulse;
    wire [7:0]  w_light_delay_ms;
    wire [7:0]  w_blow_time_out;

    wire        w_qual_hit_pulse;
    wire        w_rej_hit_pulse;
    // =======================================================
    // 模块 1：编码器采集 (不变，仍屏蔽 Z 相)
    // =======================================================
    encoder u_encoder (
        .clk   (clk_50m),    
        .rst_n (1'b1),        
        .enc_a (enc_a),      
        .enc_b (enc_b),      
        .enc_z (1'b0),        // 屏蔽机械清零，依靠16位自然溢出    
        .count (w_encoder_val) 
    );

    // =======================================================
    // 模块 2：CH368 总线桥接器 (不变)
    // =======================================================
    ch368_interface u_ch368 (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
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
    // 模块 3：【核心】工件追踪与触发引擎
    // =======================================================
    workpiece_tracker u_tracker (
        .clk          (clk_50m),
        .rst_n        (rst_n),            
        .current_enc  (w_encoder_val), 
        
        .sensor_in    (sensor_in),       
        
        // 【修改/补齐】：连接所有的外设控制引脚
        .cam_hit_pulse   (w_cam_hit_pulse),            
        .qual_hit_pulse    (w_qual_hit_pulse),        
        .rej_hit_pulse  (w_rej_hit_pulse),   

        .light_delay_out (w_light_delay_ms),
        .blow_time_out  (w_blow_time_out),

        
        
        // 接总线
        .addr         (w_sys_addr),
        .wr_en        (w_sys_wr_en),
        .data_in      (w_sys_data_out),
        .rd_en        (w_sys_rd_en),
        .data_out     (w_sys_data_in),

        // 把解码出来的电机控制信号，通过内部连线送出去
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
        
        // 接收追踪引擎送过来的命令
        .en_val       (w_motor_en),
        .dir_val      (w_motor_dir),
        .speed_val    (w_motor_speed),
        
        // 直接输出到外部物理引脚
        .motor_pwm    (motor_pwm_out),
        .motor_dir    (motor_dir_out),
        .motor_en     (motor_en_out)
    );

    // =======================================================
    // 模块 5：相机模块
    // =======================================================

    // 第 1 号相机通道
    cam_light_channel u_cam_ch0 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[0]), // 严格接入第 0 根脉冲线
        .delay_ms      (w_light_delay_ms),    // 共享延时配置
        .light_pin     (light_out[0]),         // 接 1# 光源物理引脚
        .cam_pin       (camera_out[0])         // 接 1# 相机物理引脚
    );

        // 第 2 号相机通道
    cam_light_channel u_cam_ch1 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[1]), // 严格接入第 0 根脉冲线
        .delay_ms      (w_light_delay_ms),    // 共享延时配置
        .light_pin     (light_out[1]),         // 接 1# 光源物理引脚
        .cam_pin       (camera_out[1])         // 接 1# 相机物理引脚
    );
        // 第 3 号相机通道
    cam_light_channel u_cam_ch2 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[2]), // 严格接入第 0 根脉冲线
        .delay_ms      (w_light_delay_ms),    // 共享延时配置
        .light_pin     (light_out[2]),         // 接 1# 光源物理引脚
        .cam_pin       (camera_out[2])         // 接 1# 相机物理引脚
    );
        // 第 4 号相机通道
    cam_light_channel u_cam_ch3 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[3]), // 严格接入第 0 根脉冲线
        .delay_ms      (w_light_delay_ms),    // 共享延时配置
        .light_pin     (light_out[3]),         // 接 1# 光源物理引脚
        .cam_pin       (camera_out[3])         // 接 1# 相机物理引脚
    );
        // 第 5 号相机通道
    cam_light_channel u_cam_ch4 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[4]), // 严格接入第 0 根脉冲线
        .delay_ms      (w_light_delay_ms),    // 共享延时配置
        .light_pin     (light_out[4]),         // 接 1# 光源物理引脚
        .cam_pin       (camera_out[4])         // 接 1# 相机物理引脚
    );
        // 第 6 号相机通道
    cam_light_channel u_cam_ch5 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[5]), // 严格接入第 0 根脉冲线
        .delay_ms      (w_light_delay_ms),    // 共享延时配置
        .light_pin     (light_out[5]),         // 接 1# 光源物理引脚
        .cam_pin       (camera_out[5])         // 接 1# 相机物理引脚
    );
        // 第 7 号相机通道
    cam_light_channel u_cam_ch6 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[6]), // 严格接入第 0 根脉冲线
        .delay_ms      (w_light_delay_ms),    // 共享延时配置
        .light_pin     (light_out[6]),         // 接 1# 光源物理引脚
        .cam_pin       (camera_out[6])         // 接 1# 相机物理引脚
    );
        // 第 8 号相机通道
    cam_light_channel u_cam_ch7 (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .trigger_pulse (w_cam_hit_pulse[7]), // 严格接入第 0 根脉冲线
        .delay_ms      (w_light_delay_ms),    // 共享延时配置
        .light_pin     (light_out[7]),         // 接 1# 光源物理引脚
        .cam_pin       (camera_out[7])         // 接 1# 相机物理引脚
    );

    valve_ctrl u_valve_qual0(
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .trigger_pulse  (w_qual_hit_pulse),
        .blow_ms        (w_blow_time_out),
        .valve_pin      (qual_out)
    );

    valve_ctrl u_valve_rej0(
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .trigger_pulse  (w_rej_hit_pulse),
        .blow_ms        (w_blow_time_out),
        .valve_pin      (rej_out)
    );





endmodule
