--------------------------------------------------------------------------------
-- Package for RISCV. Arq0 2022
-- G.Sutter jun2022
--
-- Define constantes para diferntes módulos
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

package RISCV_pack is

    -- Tipo para los codigos de operacion:
    subtype t_opCode is std_logic_vector (6 downto 0);
    -- Codigos de operacion para las diferentes instrucciones:
    constant OP_RTYPE  : t_opCode := "0110011";
    constant OP_ITYPE  : t_opCode := "0010011"; -- I-Type Arithm
    constant OP_BRANCH : t_opCode := "1100011";
    constant OP_ST     : t_opCode := "0100011";
    constant OP_LD     : t_opCode := "0000011";
    constant OP_LUI    : t_opCode := "0110111"; -- Load Upper Inmediate
    constant OP_AUIPC  : t_opCode := "0010111"; -- Load Upper Inmediate + PC
    constant OP_JAL    : t_opCode := "1101111"; -- Jump and Link
    constant OP_JALR   : t_opCode := "1100111"; -- Jump and Link Register
 
    -- Tipo para los codigos de control de la ALU:
    subtype t_aluControl is std_logic_vector (3 downto 0);
    subtype t_aluOP      is std_logic_vector (2 downto 0);

    -- Codigos ALUOP
    constant R_Type  : t_aluOP := "010";
    constant I_Type  : t_aluOP := "011";
    constant LDST_T  : t_aluOP := "000";
    constant BRCH_T  : t_aluOP := "001";

    -- Codigos de control:
    constant ALU_ADD  : t_aluControl := "0010";
    constant ALU_SUB  : t_aluControl := "0110";
    constant ALU_AND  : t_aluControl := "0000";
    constant ALU_OR   : t_aluControl := "0001";
    constant ALU_NOT  : t_aluControl := "0101";
    constant ALU_XOR  : t_aluControl := "0111";
    constant ALU_SLT  : t_aluControl := "1010";
    constant ALU_S12  : t_aluControl := "1101";
    constant ALU_NIM  : t_aluControl := "XXXX"; --ALU not implemented yet

    -- Tipo para los codigos func3 en branches
    subtype t_funct3_branch   is std_logic_vector (2 downto 0);
    constant BR_F3_BEQ  : t_funct3_branch := "000";
    constant BR_F3_BNE  : t_funct3_branch := "001";
    constant BR_F3_BLT  : t_funct3_branch := "100";
    constant BR_F3_BGT  : t_funct3_branch := "101";
    
end package RISCV_pack;

package body RISCV_pack is
-- declare common fnctions and procedures
end package body;