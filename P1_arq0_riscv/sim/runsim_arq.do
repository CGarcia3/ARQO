--------------------------------------------------------------------------------
# Script QuestalSim para la simulacion del procesador Risc V ArqO 2022
--------------------------------------------------------------------------------

# Crear library, borrando cualquier compilacion previa:
if [file exists work] {
   vdel -lib work -all
}
vlib work

# Compilar RTL:
vcom -work work -2008 ../rtl/RISCV_pack.vhd
vcom -work work -2008 -explicit -check_synthesis ../rtl/reg_bank.vhd
vcom -work work -2008 -explicit -check_synthesis ../rtl/alu_RV.vhd
vcom -work work -2008 -explicit -check_synthesis ../rtl/alu_control.vhd
vcom -work work -2008 -explicit -check_synthesis ../rtl/control_unit.vhd
vcom -work work -2008 -explicit -check_synthesis ../rtl/Imm_Generator.vhd
vcom -work work -2008 -explicit -check_synthesis ../rtl/processorRV.vhd

# Compilar testbench:
vcom -work work -2008 -explicit memory_data.vhd
vcom -work work -2008 -explicit memory_instr.vhd
vcom -work work -2008 -explicit processorR5_tb.vhd

# Elaboracion:
vsim -gINIT_FILENAME_INST="instrucciones.txt" -gINIT_FILENAME_DATA="datos.txt" -gN_CYCLES=150 processorRV_tb


# Opcion para guardar todas las ondas:
log -r /*

# Mostrar las ondas:
do wave_arq.do

# Opcion del simulador para evitar warnings tipicos en tiempo 0 :
set StdArithNoWarnings 1
run 0 ns
set StdArithNoWarnings 0

# Lanzar la simulacion, hasta que pare sola:
run -all

--------------------------------------------------------------------------------
