module ch368_interface(
    input  wire        clk_50m,
    input  wire        rst_n,

    // 沿用老接口：接收外设模块的数据
    input  wire [7:0]  misc_data_in,

    input  wire        ch368_cs_n,
    input  wire        ch368_rd_n,
    input  wire        ch368_wr_n,
    input  wire [7:0]  ch368_addr,
    inout  wire [7:0]  ch368_data,

    output wire [7:0]  sys_addr,
    output wire [7:0]  sys_data_out,
    output wire        sys_wr_en,
    output wire        sys_rd_en,
    input  wire [7:0]  sys_data_in
);

    // 1. 写信号跨时钟域处理 (防亚稳态)
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

    // 2. 数据与地址快照锁存器 (仅在写操作时抓取)
    reg [7:0] addr_latch;
    reg [7:0] data_latch;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            addr_latch <= 8'h00;
            data_latch <= 8'h00;
        end else if (wr_n_d1 == 1'b0) begin 
            addr_latch <= ch368_addr;
            data_latch <= ch368_data;
        end
    end

    // 3. 产生单拍写脉冲 (上升沿触发)
    assign sys_wr_en = (wr_n_d2 == 1'b0 && wr_n_d1 == 1'b1);

    // =======================================================
    // 4. 【完美修复】：读写地址智能分流
    // 如果是读操作(RD低电平)，地址0延迟透传；如果是写操作，使用锁存地址
    // =======================================================
    assign sys_addr = (ch368_rd_n == 1'b0) ? ch368_addr : addr_latch;
    
    assign sys_data_out = data_latch;
    assign sys_rd_en = (~ch368_rd_n);

    // 5. 沿用老架构：内部数据路由
    localparam ADDR_MISC = 8'h40;
    wire use_misc = (sys_rd_en && ch368_addr == ADDR_MISC);

    // 6. 三态门输出 (防止总线短路)
    assign ch368_data = sys_rd_en ? (use_misc ? misc_data_in : sys_data_in) : 8'hZZ;

endmodule