#-----------------------------------------------------
# Simple fibonacci (compilar con RARS). Para probar ruta de datos
# G.Sutter jun 2022
# Solo funciona para argument > 1
# Para ejecutar en RTL comentar llamadas a print y modificar el exit
#
.data
argument:  .word    8
fib_seq:   .space   16 # guarda hasta 16 valores en mem
str1:      .string "Fibonacci: 0 1"
space:     .string " "

.text
main:                          # initial value
        lw    t0, argument     # n = 10
        jal   printHeader      # imprime string
        
        xor   t2, t2, t2       # t1 indice posici�n
        la    t1, fib_seq      # apuntador a seq fib mem datos
        sw    t2, (t1)         # fib[0] = 0
        addi  t2, t2, 1
        sw    t2, 4(t1)       # fib[1] = 2
        addi  t1, t1, 4       # t1 apunta a fib[1]
                
f_loop: addi  t1, t1, 4       # t1 apunta a fib[n+1]    
        addi  t2, t2, 1       # indice de iteraci�n
        lw    t3, -8(t1)      # load fib[n-2]
        lw    t4, -4(t1)      # load fib[n-1]
        add   t5, t4, t3
        sw    t5, (t1)
        jal   print_num
        bne   t2, t0, f_loop
 
exit:   li   a7, 10
        ecall                  # exit
        beq x0, x0, exit       # solo funcionar� en RTL donde ecall es ignorado

printHeader:                   # Fibonacci: 0 1
        la    a0, str1
        li    a7, 4
        ecall                  # print string str1
        ret

print_num:   
        li    a0, 32           # space char
        li    a7, 11
        ecall                  # print space
        mv    a0, t5
        li    a7, 1
        ecall                  # print int result   
        ret

