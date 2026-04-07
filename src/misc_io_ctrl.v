module misc_io_ctrl (
input wire clk,
input wire rst_n,
input wire [7:0] addr,
input wire wr_en,
input wire [7:0] data_in,
output reg [7:0] data_out,

output wire led_r,
output wire leg_y,
output wire led_g,
output wire buzzer

)

reg [7:0] misc_out_reg;

assign led_r = misc_out_reg[0];
assign led_y = misc_out_reg[1];
assign led_g = misc_out_reg[2];
assign buzzer = misc_out_reg[3];

always @(posedge clk or negedege rst_n) begin
    if(!rst_n)begin
        misc_io_ctrl <= 0;
    end else begin
        if (wr_en && addr == 8'h40) begin
            misc_out_reg = data_in;
        end
    end

always @(posedge clk)begin
    if (addr )
    
end





endmodule