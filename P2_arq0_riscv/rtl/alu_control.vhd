--------------------------------------------------------------------------------
-- Bloque de control para la ALU RISCV. Arq0 2022.
-- G.Sutter jun2022
--
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use work.RISCV_pack.all;

entity alu_control is
   port (
      -- Entradas:
      ALUOp  : in std_logic_vector (2 downto 0); -- Codigo de control desde la unidad de control
      Funct7 : in std_logic_vector (6 downto 0); -- Campo "funct7" de la instruccion (I(31:25))
      Funct3 : in std_logic_vector (2 downto 0); -- Campo "funct3" de la instruccion (I(14:12))
      -- Salida de control para la ALU:
      ALUControl : out std_logic_vector (3 downto 0) -- Define operacion a ejecutar por la ALU
   );
end alu_control;

architecture rtl of alu_control is
  -- Tipo para los codigos de control de la ALU definidos en package.

begin

    AluEfOp: process( AluOp, funct7, funct3 )
    begin
       case AluOp is
       when R_Type =>           --R-type (opcode "0110011")
          case funct3 is
          when "000" =>
             case funct7 is
                   when "0000000" =>               --ADD
                      AluControl <= ALU_ADD;
                   when "0100000" =>   
                      AluControl <= ALU_SUB;       --SUB
                   when others =>                  --not included instructions
                      AluControl <= ALU_NIM;                       
             end case;
          when "001" =>                           --SLL
                AluControl <= ALU_NIM;                       
          when "010" =>                           --SLT
                AluControl <= ALU_NIM;                       
          when "100" =>                           --XOR
                AluControl <= ALU_XOR;                       
          when "101"  =>                          --SRL
                AluControl <= ALU_NIM;                       
          when "110"  =>                          --OR
                AluControl <= ALU_OR;                       
          when "111"  =>                          --AND
                AluControl <= ALU_AND;                       
          when others =>
                AluControl <= ALU_NIM;                       
          end case; -- case funct3 R_type
       when I_Type =>          --I-type immediate arithm (opcode "0010011" )
          case funct3 is
          when "000" =>                   --ADDI
                AluControl <= ALU_ADD;
          when "111" =>                   --ANDI
                AluControl <= ALU_AND;                       
          when "100" =>                   --XORI
                AluControl <= ALU_XOR;                       
          when "110" =>                   --ORI
                AluControl <= ALU_OR;                       
          when others =>
                AluControl <= ALU_NIM;                        
          end case; -- case funct3 I_type
       when LDST_T =>         --I-type LOAD or STORE (opcode "0000011" or "0100011")
             AluControl <= ALU_ADD;               
       when BRCH_T =>                     --Branches
             AluControl <= ALU_SUB;       --SUB
       when others =>                  
             AluControl <= ALU_NIM;                        
       end case;    
    end process;

end architecture;
