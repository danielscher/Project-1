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
    reg [31:0] sharedReg;
    reg [31:0] R;
    reg [31:0] B;
    
    initial begin
		i = n;
//        i = -1;
        rx = 0;
        qx = 0;
        sharedReg = 0;
        R = 0;
        B = 0;
        enable = 0;
    end

    always @(posedge start) begin 
        enable = 1;
        sharedReg = a;
    end

    always @(posedge clock && enable) begin
        if (!b || i == 0) begin
			enable = 0;
			// $finish; 
		end

        
        r_curr = rx << 1;
//		r_curr = r_curr + a[i];
		r_curr = r_curr + sharedReg[i];

        if (r_curr < b) begin
//            qx[i] = 0;
            sharedReg[i] = 0;
            rx = r_curr;
			//$display("q = 0");
        end else begin
//            qx[i] = 1;
            sharedReg[i] = 1;
            rx = r_curr - b;
			//$display("q = 1");

        end
        i = i - 1;
    end
//	assign q = qx;
    assign q = sharedReg;
    assign r = rx;



/*
    // initialize all regs
    always @(posedge clock && start) begin
        i <= 31;
        sharedReg <= a;
        B <= b;
        R <= 0;
    end

    // actual algo
    always @(posedge clock && (i < 32)) begin
        $display("i:%d - R:%b, sharedReg:%b",i,R,sharedReg);
        
        R = (R << 1) + sharedReg[i];
        if (R < B) begin
//            sharedReg[(i << 1) + 1] = 0;
            sharedReg[i] = 0;
        end
        else begin
            sharedReg[i] = 0;
            R = R - B;
        end
        i = i - 1;
    end

    assign q = sharedReg;
    assign r = R;
*/


endmodule
