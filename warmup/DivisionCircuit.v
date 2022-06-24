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
    reg [31:0] i;
    reg [31:0] sharedReg;
    reg [31:0] B;
    
    initial begin
		i = n;
        r_curr = 0;
        sharedReg = 0;
        B = b;
    end

    always @(posedge clock && start) begin
        sharedReg = a;
        i = 31;
        B = b;
    end

    always @(posedge clock && (i < 32)) begin

        r_curr = r_curr << 1;
		r_curr = r_curr + sharedReg[i];

        if (r_curr < B) begin
            sharedReg[i] = 0;
        end else begin
            sharedReg[i] = 1;
            r_curr = r_curr - B;

        end
        i = i - 1;
    end
    assign q = sharedReg;
    assign r = r_curr;





endmodule
