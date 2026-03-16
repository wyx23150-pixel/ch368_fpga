module CH368_Test(
    // 1. FPGA 核心板自带的基础引脚
    input  wire       clk_50m,    // 核心板时钟 (记得绑到你的 24MHz 晶振引脚 M16)
    input  wire       rst_n,      // 复位按键

    // 2. CH368 本地总线接口
    input  wire       ch368_cs_n, // 片选信号
    input  wire       ch368_rd_n, // 读使能
    input  wire       ch368_wr_n, // 写使能
    input  wire [3:0] ch368_addr, // 4根地址线
    inout  wire [7:0] ch368_data, // 8根双向数据线

    // 3. 【这里是补充的！】编码器物理引脚输入
    input  wire       enc_a,      // 接到 P2 的 21 脚 (PIN_R8)
    input  wire       enc_b,      // 接到 P2 的 23 脚 (PIN_T9)
    input  wire       enc_z       // 接到 P2 的 25 脚 (PIN_T10)
);

    // =======================================================
    // 模块 1：编码器例化 (调用子模块)
    // =======================================================
    wire [31:0] encoder_val; 

    encoder u_my_encoder (
        .clk   (clk_50m),    
        .rst_n (1'b1),        
        .enc_a (enc_a),      
        .enc_b (enc_b),      
        .enc_z (enc_z),      
        .count (encoder_val) 
    );

    // =======================================================
    // 模块 2：内部测试寄存器 (小抽屉) 与写操作
    // =======================================================
    reg [7:0] reg_file [0:15];
    integer i;

    // 写信号跨时钟域处理
    reg wr_n_d1, wr_n_d2;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            wr_n_d1 <= 1'b1;
            wr_n_d2 <= 1'b1;
        end else begin
            wr_n_d1 <= ch368_wr_n;
            wr_n_d2 <= wr_n_d1;
        end
    end
    wire wr_rising_edge = (wr_n_d2 == 1'b0 && wr_n_d1 == 1'b1);

    // 写逻辑
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 16; i = i + 1) begin
                reg_file[i] <= 8'h00;
            end
            reg_file[0] <= 8'h5A; // 0号地址固定为 5A
        end else if (wr_rising_edge) begin
            // 收到写脉冲，把电脑发来的数据存入对应的抽屉
            reg_file[ch368_addr] <= ch368_data;
        end
    end

    // =======================================================
    // 模块 3：【核心修改区】读操作地址分配网络
    // =======================================================
    reg [7:0] read_data;
    
    // 用 case 语句实现多路选择：根据不同的地址，返回不同的数据
    always @(*) begin
        case (ch368_addr)
            4'h0: read_data = reg_file[0];             // 地址 0：读出测试值 (默认5A)
            4'h1: read_data = encoder_val[7:0];        // 地址 1：读出编码器 最低 8位
            4'h2: read_data = encoder_val[15:8];       // 地址 2：读出编码器 次低 8位
            4'h3: read_data = encoder_val[23:16];      // 地址 3：读出编码器 次高 8位
            4'h4: read_data = encoder_val[31:24];      // 地址 4：读出编码器 最高 8位
            default: read_data = reg_file[ch368_addr]; // 其他地址：读出小抽屉里的值
        endcase
    end

    // =======================================================
    // 模块 4：三态门总线控制输出
    // =======================================================
    // 只要读信号为低，就立刻输出分配好的 read_data
    assign ch368_data = (!ch368_rd_n) ? read_data : 8'hZZ;

endmodule