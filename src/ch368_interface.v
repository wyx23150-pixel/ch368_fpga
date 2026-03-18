// 文件名：ch368_interface.v
module ch368_interface(
    input  wire        clk_50m,
    input  wire        rst_n,

    // 1. 物理总线引脚
    input  wire        ch368_cs_n,   // 物理引脚保留，防止 Quartus 报错，但内部不再使用它
    input  wire        ch368_rd_n,
    input  wire        ch368_wr_n,
    input  wire [7:0]  ch368_addr,   // 8根地址线 (A0~A7)
    inout  wire [7:0]  ch368_data,

    // 2. 内部标准总线接口 (输出给 workpiece_tracker)
    output wire [7:0]  sys_addr,     
    output wire [7:0]  sys_data_out, 
    output wire        sys_wr_en,    
    output wire        sys_rd_en,    
    input  wire [7:0]  sys_data_in   
);

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

    // 【修改点 1】：去掉 CS 限制，只看 WR 的上升沿！
    assign sys_wr_en = (wr_n_d2 == 1'b0 && wr_n_d1 == 1'b1);

    // 地址和数据透传
    assign sys_addr = ch368_addr;
    assign sys_data_out = ch368_data; 
    
    // 【修改点 2】：去掉 CS 限制，只要 RD 为低电平，就触发读！
    assign sys_rd_en = (~ch368_rd_n);

    // 三态门控制输出
    assign ch368_data = (sys_rd_en) ? sys_data_in : 8'hZZ;

endmodule