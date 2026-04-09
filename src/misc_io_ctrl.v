module misc_io_ctrl (
    input wire clk,
    input wire rst_n,
    input wire [7:0] addr,
    input wire wr_en,
    input wire rd_en,
    input wire [7:0] data_in,
    output reg [7:0] data_out,

    output wire led_r,
    output wire led_y,
    output wire led_g,
    output wire buzzer,
    output wire vibrator,   // 震动盘控制
    output wire feeder      // 下料控制
);

    localparam ADDR_CTRL = 8'h40;

    reg [7:0] misc_out_reg;

    assign led_r     = misc_out_reg[0];
    assign led_y     = misc_out_reg[1];
    assign led_g     = misc_out_reg[2];
    assign buzzer    = misc_out_reg[3];
    assign vibrator  = misc_out_reg[4];  // Bit4: 震动盘控制
    assign feeder    = misc_out_reg[5];  // Bit5: 下料控制

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            misc_out_reg <= 8'h0;
        end else begin
            if (wr_en && addr == ADDR_CTRL) begin
                misc_out_reg <= data_in;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out <= 8'h0;
        end else begin
            if (rd_en && addr == ADDR_CTRL) begin
                data_out <= misc_out_reg;
            end else begin
                data_out <= 8'h0;
            end
        end
    end

endmodule
