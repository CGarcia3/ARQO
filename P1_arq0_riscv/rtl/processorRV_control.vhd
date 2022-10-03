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

  --------------------------------------------------------------
  --------------------------------------------------------------
  --------------------------------------------------------------
  --------------------------------------------------------------
  -- IF signals-------------------------------------------------
  signal enable_IFID    : std_logic;
  signal PC_IF          : std_logic_vector(31 downto 0);
  signal Instruction_IF : std_logic_vector(31 downto 0);
  
  -- ID signals
  -- izq
  signal enable_IDEX    : std_logic;
  signal PC_ID          : std_logic_vector(31 downto 0);
  signal Instruction_ID : std_logic_vector(31 downto 0);
  -- der  
  signal reg_RS_ID, reg_RT_ID : std_logic_vector(31 downto 0);
  
  signal Ctrl_Branch_ID, Ctrl_MemWrite_ID, Ctrl_MemRead_ID  : std_logic;
  signal Ctrl_ALUSrc_ID,  Ctrl_jalr_ID, Ctrl_RegWrite_ID    : std_logic; 
  signal Ctrl_ResSrc_ID, Ctrl_PcLui_ID : std_logic_vector(1 downto 0);
  signal Ctrl_ALUOP_ID  : std_logic_vector(2 downto 0);
  
  signal Inm_ext_ID     : std_logic_vector(31 downto 0);

  -- EX signals
  --izq
  SIGNAL PC_EX          : std_logic_vector(31 downto 0);

  signal Inst11_7_EX    : std_logic_vector(4 downto 0);
  signal Inst31_25_EX   : std_logic_vector(6 downto 0);
  signal Inst14_12_EX   : std_logic_vector(2 downto 0);
  
  signal reg_RS_EX, reg_RT_EX : std_logic_vector(31 downto 0);
  
  signal Ctrl_Branch_EX, Ctrl_MemWrite_EX, Ctrl_MemRead_EX  : std_logic;
  signal Ctrl_ALUSrc_EX,  Ctrl_jalr_EX, Ctrl_RegWrite_EX    : std_logic; 
  signal Ctrl_ResSrc_EX, Ctrl_PcLui_EX : std_logic_vector(1 downto 0);
  signal Ctrl_ALUOP_EX  : std_logic_vector(2 downto 0);
  
  signal Inm_ext_EX     : std_logic_vector(31 downto 0);
  --der
  signal PC_plusAddress_EX  : std_logic_vector(31 downto 0);
  signal zero_EX        :std_logic;
  signal Alu_Res_EX     :std_logic_vector(31 downto 0); 

  -- MEM signals
  --izq
  signal zero_MEM       :std_logic;
  signal Mem_addr_MEM   :std_logic_vector(31 downto 0); 
  signal Mem_Wd_MEM     :std_logic_vector(31 downto 0);
  signal Inst11_7_MEM   : std_logic_vector(4 downto 0); 

  signal Ctrl_Branch_MEM, Ctrl_MemWrite_MEM, Ctrl_MemRead_MEM  : std_logic;
  signal Ctrl_jalr_MEM, Ctrl_RegWrite_MEM    : std_logic; 
  signal Ctrl_ResSrc_MEM, Ctrl_PcLui_MEM : std_logic_vector(1 downto 0);

  --der
  signal Mem_Rd_MEM     : std_logic_vector(31 downto 0);

  -- WB signals
  signal Mem_Rd_WB, Mem_addr_WB : std_logic_vector(31 downto 0);
  signal Inst11_7_WB     : std_logic_vector(4 downto 0); 

  signal Ctrl_jalr_WB, Ctrl_RegWrite_WB    : std_logic; 
  signal Ctrl_ResSrc_WB, Ctrl_PcLui_WB : std_logic_vector(1 downto 0);

begin

  PC_next <= Addr_Jump_dest when desition_Jump = '1' else PC_plus4;

  -- Elements involved in the IFID register following the diagram: 
  -- PC (maybe pc+4 too?)
  -- Instruction vector

  IFID: process(clk, reset)
  begin
    if reset = '1' then
      PC_ID <= (others=>'0');
      Instruction_ID <= (others=>'0');
    elsif rising_edge(clk) and enable_IFID = '1' then
      PC_ID <= PC_IF;
      Instruction_ID <= Instruction_IF;
    end if;
  end process;

  IDEX: process(clk, reset)
  begin
    if reset = '1' then
      --signals to 0
      PC_EX <= (others=>'0');
      Inst11_7_EX <= (others=>'0');
      Inst31_25_EX <= (others=>'0');
      Inst14_12_EX <= (others=>'0');
  
      reg_RS_EX <= (others=>'0');
      reg_RT_EX <= (others=>'0');
  
      Ctrl_Branch_EX <= '0';
      Ctrl_MemWrite_EX <= '0';
      Ctrl_MemRead_EX <= '0';
      Ctrl_ALUSrc_EX <= '0';
      Ctrl_jalr_EX <= '0';
      Ctrl_RegWrite_EX <= '0';
      Ctrl_ResSrc_EX <= (others=>'0');
      Ctrl_PcLui_EX <= (others=>'0');
      Ctrl_ALUOP_EX <= (others=>'0');
  
      Inm_ext_EX <= (others=>'0');

    elsif rising_edge(clk) and enable_IDEX = '1' then
      --signals to values
      PC_EX <= PC_ID;
      Inst11_7_EX <= Instruction_ID(11 downto 7);
      Inst31_25_EX <= Instruction_ID(31 downto 25);
      Inst14_12_EX <= Instruction_ID(14 downto 12);
  
      reg_RS_EX <= reg_RS_ID;
      reg_RT_EX <= reg_RT_ID;
  
      Ctrl_Branch_EX <= Ctrl_Branch_ID;
      Ctrl_MemWrite_EX <= Ctrl_MemWrite_ID;
      Ctrl_MemRead_EX <= Ctrl_MemRead_ID;
      Ctrl_ALUSrc_EX <= Ctrl_ALUSrc_ID;
      Ctrl_jalr_EX <= Ctrl_jalr_ID;
      Ctrl_RegWrite_EX <= Ctrl_RegWrite_ID;
      Ctrl_ResSrc_EX <= Ctrl_ResSrc_ID;
      Ctrl_PcLui_EX <= Ctrl_PcLui_ID;
      Ctrl_ALUOP_EX <= Ctrl_ALUOP_ID;
  
      Inm_ext_EX <= Inm_ext_ID;

    end if;
  end process;


  EXMEM: process(clk, reset)
  begin
    if reset = '1' then
      --signals to 0
      Addr_Jump_dest <= (others=>'0');
      zero_MEM <= '0';
      Mem_addr_MEM <= (others=>'0');
      Mem_Wd_MEM <= (others=>'0');
  
      Ctrl_Branch_MEM <= '0';
      Ctrl_MemWrite_MEM <= '0';
      Ctrl_MemRead_MEM <= '0';
      Ctrl_jalr_MEM <= '0';
      Ctrl_RegWrite_MEM <= '0';
      Ctrl_ResSrc_MEM <= (others=>'0');
      Ctrl_PcLui_MEM <= (others=>'0');

      Inst11_7_MEM <= (others=>'0');

    elsif rising_edge(clk) and enable_EXMEM = '1' then
      --signals to values
      Addr_Jump_dest <= PC_plusAddress_EX;
      zero_MEM <= zero_EX;
      Mem_addr_MEM <= Alu_Res_EX;
      Mem_Wd_MEM <= reg_RT_EX;
      Inst11_7_MEM <= Inst11_7_EX;
  
      Ctrl_Branch_MEM <= Ctrl_Branch_EX;
      Ctrl_MemWrite_MEM <= Ctrl_MemWrite_EX;
      Ctrl_MemRead_MEM <= Ctrl_MemRead_EX;
      Ctrl_jalr_MEM <= Ctrl_jalr_EX;
      Ctrl_RegWrite_MEM <= Ctrl_RegWrite_EX;
      Ctrl_ResSrc_MEM <= Ctrl_ResSrc_EX;
      Ctrl_PcLui_MEM <= Ctrl_PcLui_EX;

    end if;
  end process;
  

  MEMWB: process(clk, reset)
  begin
    if reset = '1' then
      --signals to 0
      Mem_Rd_WB <= (others=>'0');
      Mem_addr_WB <= (others=>'0');
  
      Ctrl_jalr_WB <= '0';
      Ctrl_RegWrite_WB <= '0';
      Ctrl_ResSrc_WB <= '0';
      Ctrl_PcLui_WB <= '0';

      Inst11_7_WB <= (others=>'0');

    elsif rising_edge(clk) and enable_MEMWB = '1' then
      --signals to values
      Mem_Rd_WB <= Mem_Rd_MEM;
      Mem_addr_WB <= Mem_addr_MEM;
  
      Ctrl_jalr_WB <= Ctrl_jalr_MEM;
      Ctrl_RegWrite_WB <= Ctrl_RegWrite_MEM;
      Ctrl_ResSrc_WB <= Ctrl_ResSrc_MEM;
      Ctrl_PcLui_WB <= Ctrl_PcLui_MEM;

      Inst11_7_WB <= Inst11_7_MEM;

    end if;
  end process;


  enable_IFID <= '1';
  enable_IDEX <= '1';
  enable_EXMEM <= '1';
  enable_MEMWB <= '1';


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
  Instruction_IF <= IDataIn;
  Funct3      <= Inst14_12_EX; -- Campo "funct3" de la instruccion
  Funct7      <= Inst31_25_EX; -- Campo "funct7" de la instruccion
  RD          <= Inst11_7_WB;
  RS1         <= Instruction_ID(19 downto 15);
  RS2         <= Instruction_ID(24 downto 20);

  RegsRISCV : reg_bank
  port map (
    Clk   => Clk,
    Reset => Reset,
    A1    => Instruction_ID(19 downto 15), --Instruction(19 downto 15), --rs1
    Rd1   => reg_RS_ID,
    A2    => Instruction_ID(24 downto 20), --Instruction(24 downto 20), --rs2
    Rd2   => reg_RT_ID,
    A3    => Inst11_7_WB, --Instruction(11 downto 7),,
    Wd3   => reg_RD_data,
    We3   => Ctrl_RegWrite_MEM
  );

  UnidadControl : control_unit
  port map(
    OpCode   => Instruction_ID(6 downto 0),
    -- Señales para el PC
    --Jump   => CONTROL_JUMP,
    Branch   => Ctrl_Branch_ID,
    -- Señales para la memoria
    ResultSrc=> Ctrl_ResSrc_ID,
    MemWrite => Ctrl_MemWrite_ID,
    MemRead  => Ctrl_MemRead_ID,
    -- Señales para la ALU
    ALUSrc   => Ctrl_ALUSrc_ID,
    AuipcLui => Ctrl_PcLui_ID,
    ALUOP    => Ctrl_ALUOP_ID,
    -- señal generacion salto
    Ins_jalr => Ctrl_jalr_ID, -- 0=any instrucion, 1=jalr
    -- Señales para el GPR
    RegWrite => Ctrl_RegWrite_ID
  );

  inmed_op : Imm_Gen
  port map (
        instr    => Instruction_ID,
        imm      => Inm_ext_ID 
  );

  Addr_Branch    <= PC_reg + Inm_ext;
  Addr_jalr      <= reg_RS + Inm_ext;

  desition_Jump  <= Ctrl_Jalr or (Ctrl_Branch and branch_true);
  branch_true    <= '1' when ( ((Inst14_12_EX = BR_F3_BEQ) and (zero_EX = '1')) or
                               ((Inst14_12_EX = BR_F3_BNE) and (zero_EX = '0')) or
                               ((Inst14_12_EX = BR_F3_BLT) and (Alu_SIGN = '1')) or
                               ((Inst14_12_EX = BR_F3_BGT) and (Alu_SIGN = '0')) ) else
                    '0';
 
  Addr_Jump_dest <= Addr_jalr   when Ctrl_jalr = '1' else
                    Addr_Branch when Ctrl_Branch='1' else
                    (others =>'0');

  Alu_control_i: alu_control
  port map(
    -- Entradas:
    ALUOp  => Ctrl_ALUOP_ID, -- Codigo de control desde la unidad de control
    Funct3  => Inst14_12_EX,    -- Campo "funct3" de la instruccion
    Funct7  => Inst31_25_EX,    -- Campo "funct7" de la instruccion
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
    Zflag    => zero_EX
  );


  Alu_Op1    <= PC_EX           when Ctrl_PcLui_EX = "00" else
                (others => '0')  when Ctrl_PcLui_EX = "01" else
                reg_RS_EX; -- any other 
  Alu_Op2    <= reg_RT_EX when Ctrl_ALUSrc_EX = '0' else Inm_ext_EX;


  DAddr      <= Mem_addr_MEM;
  DDataOut   <= Mem_Rd_MEM;
  DWrEn      <= Ctrl_MemWrite_MEM;
  dRdEn      <= Ctrl_MemRead_MEM;
  Mem_Rd_MEM <= DDataIn;

  reg_RD_data <= Mem_Rd_WB when Ctrl_ResSrc_WB = "01" else
                 PC_plus4   when Ctrl_ResSrc_WB = "10" else 
                 Mem_addr_WB; -- When 00

end architecture;
