onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /processorrv_tb/clk
add wave -noupdate /processorrv_tb/reset
add wave -noupdate /processorrv_tb/iAddr
add wave -noupdate /processorrv_tb/iDataIn
add wave -noupdate /processorrv_tb/dWrEn
add wave -noupdate /processorrv_tb/dRdEn
add wave -noupdate -divider {New Divider}
add wave -noupdate /processorrv_tb/i_processor/RegsRISCV/regs
add wave -noupdate -divider {PC values}
add wave -noupdate -radix unsigned /processorrv_tb/i_processor/PC_reg
add wave -noupdate -radix unsigned /processorrv_tb/i_processor/PC_next
add wave -noupdate -radix unsigned /processorrv_tb/i_processor/PC_plus4
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {112 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ns} {410 ns}
