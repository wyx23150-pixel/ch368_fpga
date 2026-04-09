module ch368_interface(
    input  wire        clk_50m,
    input  wire        rst_n,

    input  wire [7:0]  misc_data_in,

    input  wire        ch368_cs_n,
    input  wire        ch368_rd_n,
    input  wire        ch368_wr_n,
    input  wire [7:0] ch368_addr,
    inout  wire [7:0]  ch368_data,

    output wire [7:0]  sys_addr,
    output wire [7:0]  sys_data_out,
    output wire        sys_wr_en,
    output wire        sys_rd_en,
    input  wire [7:0]  sys_data_in
);

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

    assign sys_wr_en = (wr_n_d2 == 1'b0 && wr_n_d1 == 1'b1);

    assign sys_addr     = ch368_addr;
    assign sys_data_out = ch368_data;

    assign sys_rd_en = (~ch368_rd_n);

    localparam ADDR_MISC = 8'h40;
    wire use_misc = (sys_rd_en && ch368_addr == ADDR_MISC);

    assign ch368_data = use_misc ? misc_data_in : sys_data_in;

endmodule
