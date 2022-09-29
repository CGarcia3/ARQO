--------------------------------------------------------------------------------
-- Unidad Generador del operando inmediato del RISCV. Arq0 2022
-- G.Sutter jun 2022
--
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use IEEE.numeric_std.all;
use ieee.std_logic_unsigned.all;
use work.RISCV_pack.all;

entity Imm_Gen is
    port (
        instr     : in std_logic_vector(31 downto 0);
        imm       : out std_logic_vector(31 downto 0)
    );
end entity Imm_Gen;

architecture rtl of Imm_Gen is

    signal opcode : std_logic_vector(6 downto 0);
    signal ItypeImm, StypeImm : std_logic_vector(31 downto 0);
    signal UtypeImm           : std_logic_vector(31 downto 0);
    signal BtypeImm, JtypeImm : std_logic_vector(31 downto 0);

begin

    ItypeImm  <= X"00000" & instr(31 downto 20) when instr(31) = '0' else 
                (X"FFFFF" & instr(31 downto 20));
    StypeImm  <= X"00000" & ( instr(31 downto 25) & instr(11 downto 7) ) when instr(31) = '0' else
                (X"FFFFF" & ( instr(31 downto 25) & instr(11 downto 7) ));
    BtypeImm  <= X"00000" & (instr(7) & instr(30 downto 25) & instr(11 downto 8) & '0') when instr(31) = '0' else
                (X"FFFFF" & (instr(7) & instr(30 downto 25) & instr(11 downto 8) & '0')); -- instr(31) implicit at position 12
    JtypeImm  <= X"000" & ( instr(19 downto 12) & instr(20) & instr(30 downto 21) & '0') when instr(31) = '0' else
                (X"FFF" & ( instr(19 downto 12) & instr(20) & instr(30 downto 21))& '0'); -- instr(31) implicit at position 20
    UtypeImm  <=  instr(31 downto 12) & X"000"; --for LUI & AUIPC
 
    opcode <= instr(6 downto 0);

    process( opcode, ItypeImm, StypeImm, BtypeImm, JtypeImm, UtypeImm)
    begin
        case opcode is
            when OP_ITYPE    => imm <= ItypeImm;         --Imm arith 
            when OP_LD       => imm <= ItypeImm;         --loads
            when OP_ST       => imm <= StypeImm;         --stores
            when OP_LUI      => imm <= UtypeImm;         --LUI
            when OP_AUIPC    => imm <= UtypeImm;         --AUIPC
            when OP_BRANCH   => imm <= BtypeImm;         --branches
            when OP_JALR     => imm <= ItypeImm;         --JALR
            when OP_JAL      => imm <= JtypeImm;         --JAL
            when others      => imm <= (others => '0');
        end case;       
    end process ; 
    
end architecture rtl;