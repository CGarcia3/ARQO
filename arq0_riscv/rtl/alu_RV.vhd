--------------------------------------------------------------------------------
-- EPS - UAM. Laboratorio de ARQ 2022
--
-- ALU simple for RiscV.
-- * Soporta las operaciones: +, -, and, or, xor, not, slt
-- * Genera el flag Zero (ZFlag), flag de Signo (SignFlag) y Carry
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use work.RISCV_pack.all;

entity alu_RV is
   port (
      OpA     : in  std_logic_vector (31 downto 0); -- Operando A
      OpB     : in  std_logic_vector (31 downto 0); -- Operando B
      Control : in  std_logic_vector ( 3 downto 0); -- Codigo de control=op. a ejecutar
      Result  : out std_logic_vector (31 downto 0); -- Resultado
      SignFlag: out std_logic;                      -- Sign Flag
      carryOut: out std_logic;                      -- Carry bit
      ZFlag   : out std_logic                       -- Flag Z
   );
end alu_RV;

architecture rtl of alu_RV is
  -- Tipo para los codigos de control de la ALU definidos en package.
  -- Seniales intermedias:
  signal subExt    : std_logic_vector (32 downto 0); -- resta extendida a 33 bits
  signal addExt    : std_logic_vector (32 downto 0); -- suma extendida a 33 bits
  signal sigResult : std_logic_vector (31 downto 0); -- alias interno de Result

begin

  subExt <= (OpA(31) & OpA) - (OpB(31) & OpB);
  addExt <= (OpA(31) & OpA) + (OpB(31) & OpB);

  process (Control, OpA, OpB, subExt)
  begin
    case Control is
       when ALU_OR  => sigResult <= OpA or OpB;
       when ALU_NOT => sigResult <= not OpA;
       when ALU_XOR => sigResult <= OpA xor OpB;
       when ALU_AND => sigResult <= OpA and OpB;
       when ALU_SUB => sigResult <= subExt (31 downto 0);
       when ALU_ADD => sigResult <= addExt (31 downto 0);
       when ALU_SLT => sigResult <= x"0000000" & "000" & subExt(32);
       when others => sigResult <= (others => '0');
    end case;
  end process;

  Result   <= sigResult;
  SignFlag <= sigResult(31);
  carryOut <= addExt(32) when (Control = ALU_ADD) else subExt(32);
  ZFlag    <= '1' when sigResult = x"00000000" else '0';

end architecture;

