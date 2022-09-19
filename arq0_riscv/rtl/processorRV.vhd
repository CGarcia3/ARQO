--------------------------------------------------------------------------------
-- Procesador RISC V uniciclo curso Arquitectura Ordenadores 2022
-- Initial Release G.Sutter jun 2022
-- 
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use work.RISCV_pack.all;

entity processorRV is
   port(
      Clk      : in  std_logic;                     -- Reloj activo en flanco subida
      Reset    : in  std_logic;                     -- Reset asincrono activo nivel alto
      -- Instruction memory
      IAddr    : out std_logic_vector(31 downto 0); -- Direccion Instr
      IDataIn  : in  std_logic_vector(31 downto 0); -- Instruccion leida
      -- Data memory
      DAddr    : out std_logic_vector(31 downto 0); -- Direccion
      DRdEn    : out std_logic;                     -- Habilitacion lectura
      DWrEn    : out std_logic;                     -- Habilitacion escritura
      DDataOut : out std_logic_vector(31 downto 0); -- Dato escrito
      DDataIn  : in  std_logic_vector(31 downto 0)  -- Dato leido
   );
end processorRV;

architecture rtl of processorRV is

  component alu_RV
    port (
      OpA     : in  std_logic_vector (31 downto 0); -- Operando A
      OpB     : in  std_logic_vector (31 downto 0); -- Operando B
      Control : in  std_logic_vector ( 3 downto 0); -- Codigo de control=op. a ejecutar
      Result  : out std_logic_vector (31 downto 0); -- Resultado
      SignFlag: out std_logic;                      -- Sign Flag
      carryOut: out std_logic;                      -- Carry bit
      ZFlag   : out std_logic                       -- Flag Z
    );
  end component;

  component reg_bank
     port (
        Clk   : in  std_logic;                      -- Reloj activo en flanco de subida
        Reset : in  std_logic;                      -- Reset as�ncrono a nivel alto
        A1    : in  std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Rd1
        Rd1   : out std_logic_vector(31 downto 0);  -- Dato del puerto Rd1
        A2    : in  std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Rd2
        Rd2   : out std_logic_vector(31 downto 0);  -- Dato del puerto Rd2
        A3    : in  std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Wd3
        Wd3   : in  std_logic_vector(31 downto 0);  -- Dato de entrada Wd3
        We3   : in  std_logic                       -- Habilitaci�n de la escritura de Wd3
     ); 
  end component reg_bank;

  component control_unit
     port (
        -- Entrada = codigo de operacion en la instruccion:
        OpCode   : in  std_logic_vector (6 downto 0);
        -- Seniales para el PC
        Branch   : out  std_logic;                     -- 1 = Ejecutandose instruccion branch
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
        RegWrite : out  std_logic                      -- 1 = Escribir registro
     );
  end component;

  component alu_control is
    port (
      -- Entradas:
      ALUOp  : in std_logic_vector (2 downto 0);     -- Codigo de control desde la unidad de control
      Funct3 : in std_logic_vector (2 downto 0);     -- Campo "funct3" de la instruccion (I(14:12))
      Funct7 : in std_logic_vector (6 downto 0);     -- Campo "funct7" de la instruccion (I(31:25))     
      -- Salida de control para la ALU:
      ALUControl : out std_logic_vector (3 downto 0) -- Define operacion a ejecutar por la ALU
    );
  end component alu_control;

 component Imm_Gen is
    port (
        instr     : in std_logic_vector(31 downto 0);
        imm       : out std_logic_vector(31 downto 0)
    );
  end component Imm_Gen;

  signal Alu_Op1      : std_logic_vector(31 downto 0);
  signal Alu_Op2      : std_logic_vector(31 downto 0);
  signal Alu_ZERO     : std_logic;
  signal Alu_SIGN      : std_logic;
  signal AluControl   : std_logic_vector(3 downto 0);
  signal reg_RD_data  : std_logic_vector(31 downto 0);

  signal branch_true : std_logic;
  signal PC_next        : std_logic_vector(31 downto 0);
  signal PC_reg         : std_logic_vector(31 downto 0);
  signal PC_plus4       : std_logic_vector(31 downto 0);

  signal Instruction    : std_logic_vector(31 downto 0); -- La instrucción desde lamem de instr
  signal Inm_ext        : std_logic_vector(31 downto 0); -- La parte baja de la instrucción extendida de signo
  signal reg_RS, reg_RT : std_logic_vector(31 downto 0);

  signal dataIn_Mem     : std_logic_vector(31 downto 0); -- From Data Memory
  signal Addr_Branch    : std_logic_vector(31 downto 0);

  signal Ctrl_Jalr, Ctrl_Branch, Ctrl_MemWrite, Ctrl_MemRead,  Ctrl_ALUSrc, Ctrl_RegWrite : std_logic;
  
  --Ctrl_RegDest,
  signal Ctrl_ALUOP     : std_logic_vector(2 downto 0);
  signal Ctrl_PcLui     : std_logic_vector(1 downto 0);
  signal Ctrl_ResSrc    : std_logic_vector(1 downto 0);

  signal Addr_jalr      : std_logic_vector(31 downto 0);
  signal Addr_Jump_dest : std_logic_vector(31 downto 0);
  signal desition_Jump  : std_logic;
  signal Alu_Res        : std_logic_vector(31 downto 0);
  -- Instruction filds
  signal Funct3         : std_logic_vector(2 downto 0);
  signal Funct7         : std_logic_vector(6 downto 0);
  signal RS1, RS2, RD   : std_logic_vector(4 downto 0);


begin

  PC_next <= Addr_Jump_dest when desition_Jump = '1' else PC_plus4;

  -- Program Counter
  PC_reg_proc: process(Clk, Reset)
  begin
    if Reset = '1' then
      PC_reg <= (22 => '1', others => '0'); -- 0040_0000
    elsif rising_edge(Clk) then
      PC_reg <= PC_next;
    end if;
  end process;

  PC_plus4    <= PC_reg + 4;
  IAddr       <= PC_reg;
  Instruction <= IDataIn;
  Funct3      <= instruction(14 downto 12); -- Campo "funct3" de la instruccion
  Funct7      <= instruction(31 downto 25); -- Campo "funct7" de la instruccion
  RD          <= Instruction(11 downto 7);
  RS1         <= Instruction(19 downto 15);
  RS2         <= Instruction(24 downto 20);

  RegsRISCV : reg_bank
  port map (
    Clk   => Clk,
    Reset => Reset,
    A1    => RS1, --Instruction(19 downto 15), --rs1
    Rd1   => reg_RS,
    A2    => RS2, --Instruction(24 downto 20), --rs2
    Rd2   => reg_RT,
    A3    => RD, --Instruction(11 downto 7),,
    Wd3   => reg_RD_data,
    We3   => Ctrl_RegWrite
  );

  UnidadControl : control_unit
  port map(
    OpCode   => Instruction(6 downto 0),
    -- Señales para el PC
    --Jump   => CONTROL_JUMP,
    Branch   => Ctrl_Branch,
    -- Señales para la memoria
    ResultSrc=> Ctrl_ResSrc,
    MemWrite => Ctrl_MemWrite,
    MemRead  => Ctrl_MemRead,
    -- Señales para la ALU
    ALUSrc   => Ctrl_ALUSrc,
    AuipcLui => Ctrl_PcLui,
    ALUOP    => Ctrl_ALUOP,
    -- señal generacion salto
    Ins_jalr => Ctrl_jalr, -- 0=any instrucion, 1=jalr
    -- Señales para el GPR
    RegWrite => Ctrl_RegWrite
  );

  inmed_op : Imm_Gen
  port map (
        instr    => Instruction,
        imm      => Inm_ext 
  );

  Addr_Branch    <= PC_reg + Inm_ext;
  Addr_jalr      <= reg_RS + Inm_ext;

  desition_Jump  <= Ctrl_Jalr or (Ctrl_Branch and branch_true);
  branch_true    <= '1' when ( ((Funct3 = BR_F3_BEQ) and (Alu_ZERO = '1')) or
                               ((Funct3 = BR_F3_BNE) and (Alu_ZERO = '0')) or
                               ((Funct3 = BR_F3_BLT) and (Alu_SIGN = '1')) or
                               ((Funct3 = BR_F3_BGT) and (Alu_SIGN = '0')) ) else
                    '0';
 
  Addr_Jump_dest <= Addr_jalr   when Ctrl_jalr = '1' else
                    Addr_Branch when Ctrl_Branch='1' else
                    (others =>'0');

  Alu_control_i: alu_control
  port map(
    -- Entradas:
    ALUOp  => Ctrl_ALUOP, -- Codigo de control desde la unidad de control
    Funct3  => Funct3,    -- Campo "funct3" de la instruccion
    Funct7  => Funct7,    -- Campo "funct7" de la instruccion
    -- Salida de control para la ALU:
    ALUControl => AluControl -- Define operacion a ejecutar por la ALU
  );

  Alu_RISCV : alu_RV
  port map (
    OpA      => Alu_Op1,
    OpB      => Alu_Op2,
    Control  => AluControl,
    Result   => Alu_Res,
    Signflag => Alu_SIGN,
    carryOut => open,
    Zflag    => Alu_ZERO
  );

  Alu_Op1    <= PC_reg           when Ctrl_PcLui = "00" else
                (others => '0')  when Ctrl_PcLui = "01" else
                reg_RS; -- any other 
  Alu_Op2    <= reg_RT when Ctrl_ALUSrc = '0' else Inm_ext;


  DAddr      <= Alu_Res;
  DDataOut   <= reg_RT;
  DWrEn      <= Ctrl_MemWrite;
  dRdEn      <= Ctrl_MemRead;
  dataIn_Mem <= DDataIn;

  reg_RD_data <= dataIn_Mem when Ctrl_ResSrc = "01" else
                 PC_plus4   when Ctrl_ResSrc = "10" else 
                 Alu_Res; -- When 00

end architecture;
