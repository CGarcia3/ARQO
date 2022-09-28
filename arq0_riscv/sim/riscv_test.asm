##########################################################################
#   Programa de prueba para Practica 1 ARQO 2022                         #
#                                                                        #
##########################################################################
# Programa en ensamblador RISC-V para probar el funcionamiento de la P1. 
# Incluye todas las instrucciones a verificar. Los saltos de los beq se 
# realizan de forma efectiva si las operaciones anteriores han devuelto
# resultados correctos en los registros. 
# El programa termina con un buble infinito (En RARS mejor ver paso a paso)
# En las formas de ondas debes analizar el funcionamiento
#
############################################################################

.data
num0:   .word  1 # posic base + 0
num1:   .word  2 # posic base + 4
num2:   .word  4 # posic base + 8
num3:   .word  8 # posic base + 12
num4:   .word 16 # posic base + 16
num5:   .word 32 # posic base + 20
num32:  .word 0xAAAA5555 # posic base + 24
buffer: .space 4

.text 
main: #Pruebas Load Store y Lui
      lui  t0, %hi(num0)  # Carga la parte alta de la dir num0
      nop
      nop
      nop
      lw   t1, 0(t0)      # En x6 un 1
      lw   t2, 4(t0)      # En x7 un 2
      lw   t3, 8(t0)      # En x28 un 4 
      lw   t4,12(t0)      # En x29 un 8
      lw   t5,16(t0)      # En x30 un 16
      lw   t6,20(t0)      # En x31 un 32
     
      sw   t1, 32(t0)     # Graba x6 en dir 32 (un 1)
      sw   t2, 36(t0)     # Graba x7 en dir 36 (En x6 un 2)
      sw   t3, 40(t0)     # Graba x28 en dir 40 (En x7 un 4) 
      sw   t4, 44(t0)     # Graba x29 en dir 44 (En x29 un 8)
      sw   t5, 48(t0)     # Graba x30 en dir 48 (En x30 un 16)
      sw   t6, 52(t0)     # Graba x31 en dir 52 (En x31 un 32)

      lw   a1, 32(t0)     # Graba en x11 desde dir 32 (un 1)
      lw   a2, 36(t0)     # Graba en x12 desde dir 36 (un 2)
      lw   a3, 40(t0)     # Graba en x13 desde dir 40 (un 4) 
      lw   a4, 44(t0)     # Graba en x14 desde dir 44 (un 8)
      lw   a5, 48(t0)     # Graba en x15 desde dir 48 (un 16)
      lw   a6, 52(t0)     # Graba en x16 desde dir 52 (un 32)
      
      bne  t1, a1, error_LW # tienen que ser iguales y no saltar
      bne  t2, a2, error_LW
      bne  t3, a3, error_LW
      bne  t4, a4, error_LW
      bne  t5, a5, error_LW
      bne  t6, a6, error_LW
      
      # si no ha saltado OK
Pru_add:
      li   t1, 55        # En x6 un 55. Asume que addi funciona correctamente
      li   t2, 77        # En x7 un 77. Asume que addi funciona correctamente
      add  a0, t2, t1    # En x10, 55+77=132. En Pipeline sin adelantamiento puede fallar
      add  a1, t2, t1    # En x11, 55+77=132. En Pipeline sin adelantamiento puede fallar   
      add  a2, t2, t1    # En x12, 55+77=132. En Pipeline sin adelantamiento puede fallar
      add  a3, t2, t1    # En x13, 55+77=132.        
      nop
      nop
      nop
Pru_and:
      li   t3, 0xF0      # En x28 0xF0
      nop
      nop
      nop
      and  a4, a3, t3    # En x14 0x80
      andi a5, a3, 0xF0  # En x15 0x80
      nop
      nop
      nop     
      beq  a4, a5, OK_AND
      nop
      nop
      nop
      j    error_AND
      nop
      nop
      nop
OK_AND: 
Pru_xor: 
      addi t4, zero, -1  # cargar x29 con FFF..FFF           
      lw   t3, 24(t0)    # En x28 AAAA5555
      xor  t5, t3, t4    # t5 = not (t3), es decir 5555AAAA
      xor  t6, t5, t3    # debe dar -1. t6 = t3 xor not(x3)
      # Este grupo de instrucciones puede fallar por riesgos de datos.
      # lo siguiente repite calculo dejando "burbujas" entre instrucciones
      nop
      nop
      xor  t5, t3, t4    # t5 = not (t3), es decir 5555AAAA
      nop
      nop
      nop
      xor  t6, t5, t3    # debe dar -1. t6 = t3 xor not(x3)   

 
Pru_AUIPC:    
      la t0, buffer         # carga la dreccion del buffer en t0 (X5) [auiPC + addi). No funciona sin control de riesgos
      lui  t1, %hi(buffer)    # Carga la parte alta de la dir buffer
      nop
      nop
      nop
      addi t1, t1, %lo(buffer) # Carga la parte baja de la dir buffer
      nop
      nop
      nop
      # bne t0, t1, error_AUI    # X5 y X6 deben ser iguales. Descomentar con control de riesgos.
      nop
      nop
      nop
           
Pru_jal:  
      jal ra, salto_y_regreso           
      nop
      nop
      nop
      addi t1, zero, -15
      nop
      nop
      nop
      bne t0, t1, error_JAL
      j OK_prog
           
salto_y_regreso: # en ra(x1) tiene que estar la dir de retorno
      addi t0, zero -15  # guardo en x5 valor FFFF_FFF1. Como marca
      nop
      nop
      nop
      jalr zero, 0(ra)   # retorno al llamado
      nop
      nop
      nop
      addi t0, zero, 0xDD #nunca debe ejecutarse

#--- Buclwa errores y final de prog 
error_LW:  addi t1, t1, -1      # decrementa infinitamente t0
           beq  x0, x0, error_LW
           nop
           nop
           nop
error_AND: addi t2, t2, -1      # decrementa infinitamente t0
           beq  x0, x0, error_AND
           nop
           nop
           nop
error_AUI: addi t3, t3, -1      # decrementa infinitamente t0
           beq  x0, x0, error_AUI
           nop
           nop
           nop    
error_JAL: addi t4, t4, -1      # decrementa infinitamente t4  # FBD
           beq  x0, x0, error_JAL
           nop
           nop
           nop       
OK_prog:   addi t6, t6, 1      # incrementa infinitamente t6 (x31)
           beq  x0, x0, OK_prog
           nop
           nop
           nop
