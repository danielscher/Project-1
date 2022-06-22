module Division(
    input         clock,
    input         start,
    input  [31:0] a,
    input  [31:0] b,
    output [31:0] q,
    output [31:0] r
);
    parameter n = 31;
	reg [31:0] r_curr;
    reg [31:0] rx;
    reg [31:0] qx;
    reg [31:0] i;
    reg enable;
    initial begin
		i = n;
        rx = 0;
        qx = 0;
        enable = 0;
    end

    always @(posedge start) enable = 1;

    always @(posedge clock && enable) begin
		if (!b || i == -1) begin
			enable = 0;
			$finish; 
		end
        r_curr = rx << 1;
		r_curr = r_curr + a[i];
        if (r_curr < b) begin
            qx[i] = 0;
            rx = r_curr;
			//$display("q = 0");
        end else begin
            qx[i] = 1;
            rx = r_curr - b;
			//$display("q = 1");

        end
        i = i - 1;
    end
	assign q = qx;
    assign r = rx;

endmodule
