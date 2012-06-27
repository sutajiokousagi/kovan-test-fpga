//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Andrew "bunnie" Huang
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice, 
//    this list of conditions and the following disclaimer in the documentation and/or 
//    other materials provided with the distribution.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
//    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
//    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
//    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
//    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
//    POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////////
// Notes
//  1280 x 720p60  = 74.25 MHz pclk; 45 kHz horiz rate, 750 vertical
//  1920 x 1080p24 = 74.25 MHz pclk; 27k Hz horiz rate, 1125 vertical
//////////////////////////////////////////////////////////////////////////////

`timescale 1 ns / 1 ps

module zerobytes (
	input wire CLK26,      			// 26 mhz oscillator

	input wire RX0_HPD_N,			// Input HPD line is ignored
	output wire TX0_HPD_override,	// Allow us to override HPD if desired
	output wire[3:0] TX0_TMDS_P,
	output wire[3:0] TX0_TMDS_N,

	output wire LED_blue_N,
	
		      // LCD output to display
	      output wire [7:3] LCDO_B,  // note truncation of blue channel
	      output wire [7:2] LCDO_G,
	      output wire [7:2] LCDO_R,
	      output wire       LCDO_DEN,
	      output wire       LCDO_DOTCLK,
	      output wire       LCDO_HSYNC,
	      output wire       LCDO_RESET_N,
	      output wire       LCDO_VSYNC,

	      // LCD input from CPU
	      input wire [5:0]  LCD_B,  // note no truncation of data in
	      input wire [5:0]  LCD_G,
	      input wire [5:0]  LCD_R,
	      input wire        LCD_DEN,
	      input wire        LCD_HS,
	      input wire [5:0]  LCD_SUPP,
	      input wire        LCD_VS,
	      output wire       LCD_CLK_T,  // clock is sourced from the FPGA
	      // for forward compatibility with HDMI-synced streams

	input wire CPU_sw
);
   wire        clk26buf;
   wire LED_blue;
   assign LED_blue_N = ~LED_blue;

   // extend sync pulses and detect edge
   reg 	      vsync_v;
   reg 	      hsync_v;
   reg 	      hsync_v2;
   reg 	      vsync_v2;
   wire       hsync_rising;
   wire       vsync_rising;

   // lock state of PLLs and channels
   wire tx0_plllckd;
   wire m720p_locked;


	assign TX0_HPD_override = 1'b0;
      
	////////////////////////////////////////////////////
	// utility clock buffer 
	////////////////////////////////////////////////////
	wire clk26_ibuf;
	wire m720p_clk;
	wire tx0_pclk;  // tx clock got promoted above input port because 
	IBUFG clk26buf_ibuf(.I(CLK26), .O(clk26ibuf));
	BUFG clk26buf_buf (.I(clk26ibuf), .O(clk26buf));
   
	// generate internal 720p clock for self-timed mode operation
	BUFG clk26buf_bufx(.I(clk26ibuf), .O(clk26bufpll));
	clkgendcm_720p60hz clkgen_720p (
		 .CLK_IN1(clk26bufpll)
		,.CLK_OUT1(tx0_pclk)
		,.RESET(1'b0)
		,.LOCKED(m720p_locked)
	);
	
	reg[25:0] cntr;
	always @(posedge clk26buf)
		cntr <= cntr+1;
	assign LED_blue = cntr[22];
	
	
	
	
	   ////////// loop-throughs
   // lcd runs at a target of 6.41 MHz (156 ns cycle time)
   // i.e., 408 x 262 x 60 Hz (408 is total H width, 320 active, etc.)
   wire            qvga_clkgen_locked;
   
   clk_wiz_v3_2_qvga qvga_clkgen( .CLK_IN1(clk26buf),
				  .clk_out6p4(clk_qvga),
				  .clk_out13(clk13buf),
				  .clk_out3p25(clk3p2M), // note: a slight overclock (about 2%)
				  .clk_out208(clk208M),
				  .RESET(glbl_reset),
				  .LOCKED(qvga_clkgen_locked) );
   
   reg 	[5:0]	   lcd_pipe_b;
   reg 	[5:0]	   lcd_pipe_r;
   reg 	[5:0]	   lcd_pipe_g;
   reg 		   lcd_pipe_den;
   reg 		   lcd_hsync;
   reg 		   lcd_vsync;
   reg 		   lcd_reset_n;
   reg 		   lcd_pipe_hsync;
   reg 		   lcd_pipe_vsync;
   wire 	   lcd_reset;

   sync_reset  qvga_reset(
			  .clk(clk_qvga),
			  .glbl_reset(glbl_reset || !qvga_clkgen_locked),
			  .reset(lcd_reset) );
   always @(posedge clk_qvga) begin
      // TODO: assign timing constraints to ensure hold times met for LCD
      lcd_pipe_b[5:0] <= LCD_B[5:0];
      lcd_pipe_g[5:0] <= LCD_G[5:0];
      lcd_pipe_r[5:0] <= LCD_R[5:0];
      lcd_pipe_den <= LCD_DEN;
      lcd_pipe_hsync <= LCD_HS;
      lcd_pipe_vsync <= LCD_VS;
      lcd_reset_n <= !lcd_reset;
   end

   assign LCDO_B[7:3] = lcd_pipe_b[5:1];
   assign LCDO_G[7:2] = lcd_pipe_g[5:0];
   assign LCDO_R[7:2] = lcd_pipe_r[5:0];
   assign LCDO_DEN = lcd_pipe_den;
   assign LCDO_HSYNC = lcd_pipe_hsync;
   assign LCDO_VSYNC = lcd_pipe_vsync;
   assign LCDO_RESET_N = lcd_reset_n;

   // low-skew clock mirroring to an output pin requires this hack
   ODDR2 qvga_clk_to_lcd (.D0(1'b1), .D1(1'b0), 
			  .C0(clk_qvga), .C1(!clk_qvga), 
			  .Q(LCDO_DOTCLK), .CE(1'b1), .R(1'b0), .S(1'b0) );

   ODDR2 qvga_clk_to_cpu (.D0(1'b1), .D1(1'b0), 
			 .C0(clk_qvga), .C1(!clk_qvga), 
			 .Q(LCD_CLK_T), .CE(1'b1), .R(1'b0), .S(1'b0) );


	//////////////////////////////////////////////////////////////////
	// Instantiate a dedicate PLL for output port
	//////////////////////////////////////////////////////////////////
	wire tx0_clkfbout, tx0_clkfbin;
	wire tx0_pllclk0, tx0_pllclk2;

	PLL_BASE # (
//    .CLKIN_PERIOD(10.526315), // 95 MHz
//    .CLKIN_PERIOD(35.34), // 28.29 MHz 480p/60
		.CLKIN_PERIOD(13.481449525), // 74.176 MHz
		.CLKFBOUT_MULT(10), //set VCO to 10x of CLKIN
		.CLKOUT0_DIVIDE(1),
		.CLKOUT1_DIVIDE(10),
		.CLKOUT2_DIVIDE(5),
//	      .BANDWIDTH("LOW"), // normally not here
		.COMPENSATION("SOURCE_SYNCHRONOUS")
	) PLL_OSERDES_0 (
		.CLKFBOUT(tx0_clkfbout),
		.CLKOUT0(tx0_pllclk0),
		.CLKOUT1(),
		.CLKOUT2(tx0_pllclk2),
		.CLKOUT3(),
		.CLKOUT4(),
		.CLKOUT5(),
		.LOCKED(tx0_plllckd),
		.CLKFBIN(tx0_clkfbin),
		.CLKIN(tx0_pclk),
		.RST(~m720p_locked)  
	);
   
	//
	// This BUFG is needed in order to deskew between PLL clkin and clkout
	// So the tx0 pclkx2 and pclkx10 will have the same phase as the pclk input
	//
	BUFG tx0_clkfb_buf (.I(tx0_clkfbout), .O(tx0_clkfbin));

	//
	// regenerate pclkx2 for TX
	//
	BUFG tx0_pclkx2_buf (.I(tx0_pllclk2), .O(tx0_pclkx2));

	//
	// regenerate pclkx10 for TX
	//
	wire tx0_bufpll_lock;
	BUFPLL #(
		.DIVIDE(5)
	) tx0_ioclk_buf (
		.PLLIN(tx0_pllclk0), 
		.GCLK(tx0_pclkx2),
		.LOCKED(tx0_plllckd),
		.IOCLK(tx0_pclkx10),
		.SERDESSTROBE(tx0_serdesstrobe),
		.LOCK(tx0_bufpll_lock)
	);

	wire byp_error;
	
   
  /////////////////
  //
  // Output Port 0
  //
  /////////////////
	wire[7:0]	tx0_blue;
	wire[7:0]	tx0_green;
	wire[7:0]	tx0_red;
	reg			tx0_de;
	wire		tx0_reset;
	
	wire VGA_HSYNC_INT, VGA_VSYNC_INT;
	wire[10:0]	bgnd_hcount;
	wire		bgnd_hblnk;
	wire[10:0]	bgnd_vcount;
	wire		bgnd_vblnk;
	reg[10:0] tc_hsblnk;
	reg[10:0] tc_hssync;
	reg[10:0] tc_hesync;
	reg[10:0] tc_heblnk;
	reg[10:0] tc_vsblnk;
	reg[10:0] tc_vssync;
	reg[10:0] tc_vesync;
	reg[10:0] tc_veblnk;
	reg hvsync_polarity; //1-Negative, 0-Positive
	parameter HPIXELS_HDTV720P = 11'd1280; //Horizontal Live Pixels
	parameter VLINES_HDTV720P  = 11'd720;  //Vertical Live ines
	parameter HSYNCPW_HDTV720P = 11'd80;  //HSYNC Pulse Width
	parameter VSYNCPW_HDTV720P = 11'd5;    //VSYNC Pulse Width
	parameter HFNPRCH_HDTV720P = 11'd72;   //Horizontal Front Porch
	parameter VFNPRCH_HDTV720P = 11'd3;    //Vertical Front Porch
	parameter HBKPRCH_HDTV720P = 11'd216;  //Horizontal Front Porch
	parameter VBKPRCH_HDTV720P = 11'd22;   //Vertical Front Porch

	assign tx0_reset = (~tx0_plllckd);

	always
	begin
        hvsync_polarity = 1'b0;

        tc_hsblnk = HPIXELS_HDTV720P - 11'd1;
        tc_hssync = HPIXELS_HDTV720P - 11'd1 + HFNPRCH_HDTV720P;
        tc_hesync = HPIXELS_HDTV720P - 11'd1 + HFNPRCH_HDTV720P + HSYNCPW_HDTV720P;
        tc_heblnk = HPIXELS_HDTV720P - 11'd1 + HFNPRCH_HDTV720P + HSYNCPW_HDTV720P + HBKPRCH_HDTV720P;
        tc_vsblnk =  VLINES_HDTV720P - 11'd1;
        tc_vssync =  VLINES_HDTV720P - 11'd1 + VFNPRCH_HDTV720P;
        tc_vesync =  VLINES_HDTV720P - 11'd1 + VFNPRCH_HDTV720P + VSYNCPW_HDTV720P;
        tc_veblnk =  VLINES_HDTV720P - 11'd1 + VFNPRCH_HDTV720P + VSYNCPW_HDTV720P + VBKPRCH_HDTV720P;
	end
	
	timing timing_inst (
		.tc_hsblnk(tc_hsblnk), //input
		.tc_hssync(tc_hssync), //input
		.tc_hesync(tc_hesync), //input
		.tc_heblnk(tc_heblnk), //input
		
		.hcount(bgnd_hcount), //output
		.hsync(VGA_HSYNC_INT), //output
		.hblnk(bgnd_hblnk), //output
		
		
		.tc_vsblnk(tc_vsblnk), //input
		.tc_vssync(tc_vssync), //input
		.tc_vesync(tc_vesync), //input
		.tc_veblnk(tc_veblnk), //input
		
		.vcount(bgnd_vcount), //output
		.vsync(VGA_VSYNC_INT), //output
		.vblnk(bgnd_vblnk), //output
		
		.restart(tx0_reset),
		.clk(tx0_pclk)
	);


	///////////////////////////////////
	// Video pattern generator:
	//   SMPTE HD Color Bar
	///////////////////////////////////
	hdcolorbar clrbar(
		 .i_clk_74M(tx0_pclk)
		,.i_rst(tx0_reset)
		,.i_hcnt(bgnd_hcount)
		,.i_vcnt(bgnd_vcount)
		,.baronly(1'b0)
		,.i_format(2'b00) // Progressive-scan, 720p
		,.o_r(tx0_red)
		,.o_g(tx0_green)
		,.o_b(tx0_blue)
	);

	/////////////////////////////////////////
	// V/H SYNC and DE generator
	/////////////////////////////////////////
	assign active = !bgnd_hblnk && !bgnd_vblnk;

	reg active_q;
	reg vsync, hsync;
	reg VGA_HSYNC, VGA_VSYNC;

	always @ (posedge tx0_pclk)
	begin
		hsync <= VGA_HSYNC_INT ^ hvsync_polarity ;
		vsync <= VGA_VSYNC_INT ^ hvsync_polarity ;
		VGA_HSYNC <= hsync;
		VGA_VSYNC <= vsync;

		active_q <= active;
		tx0_de <= active_q;
	end


	dvi_encoder_top enc0 (
		.pclk        (tx0_pclk),
		.pclkx2      (tx0_pclkx2),
		.pclkx10     (tx0_pclkx10),
		.serdesstrobe(tx0_serdesstrobe),
		.rstin       (tx0_reset),
		.blue_din    (tx0_blue),
		.green_din   (tx0_green),
		.red_din     (tx0_red),
		.hsync       (VGA_HSYNC),
		.vsync       (VGA_VSYNC),
		.de          (tx0_de),
		.TMDS        (TX0_TMDS_P),
		.TMDSB       (TX0_TMDS_N)
    );
   /*
  dvi_encoder_top dvi_tx0 (
    .pclk        (tx0_pclk),
    .pclkx2      (tx0_pclkx2),
    .pclkx10     (tx0_pclkx10),
    .serdesstrobe(tx0_serdesstrobe),
    .rstin       (tx0_reset),
    .blue_din    (tx0_blue),
    .green_din   (tx0_green),
    .red_din     (tx0_red),
    .hsync       (VGA_HSYNC),
    .vsync       (VGA_VSYNC),
    .de          (tx0_de),
    .TMDS        (TX0_TMDS),
    .TMDSB       (TX0_TMDSB),
    .vid_pa      (1'b0),
    .vid_gb      (1'b0),
    .dat_pa      (1'b0),
    .dat_gb      (1'b0),
    .dat_ena     (1'b0),
    .dat_din     (1'b0),
    .ctl_code    (1'b0),
    .bypass_sdata(),
    .bypass_ena  (1'b1),
    .byp_error   ()
  );
*/
      

	reg [22:0] counter;
	always @(posedge clk26buf)
		counter <= counter + 1;

endmodule
