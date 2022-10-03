--------------------------------------------------------------------------------
-- Unidad de control principal del RISCV. ArqO 2022
-- G.Sutter jun222
--
-- Implementa set reducido de instrucciones
-- R-type, lw, sw, branches (beq, bnq), jal, AuiPC, Lui
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;          
use work.RISCV_pack.all;

entity control_unit is
   port (
      -- Entrada = codigo de operacion en la instruccion:
      OpCode  : in  std_logic_vector (6 downto 0);
      -- Seniales para el PC
      Branch : out  std_logic;                       -- 1 = Ejecutandose instruccion branch
      -- Seniales relativas a la memoria
      ResultSrc: out  std_logic_vector(1 downto 0);  -- 00 salida Alu; 01 = salida de la mem.; 10 PC_plus4
      MemWrite : out  std_logic;                     -- Escribir la memoria
      MemRead  : out  std_logic;                     -- Leer la memoria
      -- Seniales para la ALU                    
      ALUSrc   : out  std_logic;                     -- 0 = oper.B es registro, 1 = es valor inm.
      AuipcLui : out  std_logic_vector (1 downto 0); -- 0 = PC. 1 = zeros, 2 = reg1.
      ALUOp    : out  std_logic_vector (2 downto 0); -- Tipo operacion para control de la ALU
      -- señal generacion salto
      Ins_jalr  : out  std_logic;                    -- 0=any instrucion, 1=jalr
      -- Seniales para el GPR
      RegWrite : out  std_logic                      -- 1=Escribir registro
   );
end control_unit;

architecture rtl of control_unit is
-- Tipo para los codigos de operacion definidos en package
begin
-- OPCode <= Instr(6 downto 0); -- 7 least significant bits

Branch   <= '1' when opCode = OP_BRANCH else
            '1' when opCode = OP_JAL    else 
            '1' when opCode = OP_JALR   else 
            '0';

ALUSrc   <= '0' when opCode = OP_RTYPE  else -- R-type
            '0' when opCode = OP_BRANCH else -- beq
            '1'; -- lw, sw, Itype

AuipcLui <= "00" when opCode = OP_AUIPC else -- add PC
            "01" when opCode = OP_LUI   else -- LUI, connects to zero
            "10"; -- default for most instructions


RegWrite <= '0' when opCode = OP_ST     else -- sw
            '0' when opCode = OP_BRANCH else -- any branch
            '1'; -- R-type, lw, I-type, lui , jal

MemRead  <= '1' when opCode = OP_LD else --lw
            '0';

MemWrite <= '1' when opCode = OP_ST else -- sw
            '0'; 

ResultSrc<= "01" when opCode = OP_LD  else -- lw
            "10" when opCode = OP_JAL else -- jal
            "00"; -- R-type, sw, beq, lui I-type

ALUOP    <= LDST_T when opCode = OP_LD     else -- ld
            LDST_T when opCode = OP_ST     else -- sd
            LDST_T when opCode = OP_LUI    else -- lui (add to zero)
            LDST_T when opCode = OP_AUIPC  else -- AuiPC (add PC)
            BRCH_T when opCode = OP_BRANCH else -- any branch
            R_Type when opCode = OP_RTYPE  else -- R-type;
            I_Type when opCode = OP_ITYPE  else -- I-type;
            "111"; --Senial de no reconocido o no opera ALU

Ins_jalr <= '1' when opCode = OP_JALR else '0';

end architecture;
