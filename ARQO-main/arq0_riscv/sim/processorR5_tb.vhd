---------------------------------------------------------------------------------------------------
-- EPS - UAM
-- Laboratorio de ARQ0 2022
--
-- Testbench simple para el procesador RISC V.
---------------------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

entity processorRV_tb is
   generic (
      INIT_FILENAME_INST : string   := "instrucciones.txt"; -- Fichero con las instrucciones
      INIT_FILENAME_DATA : string   := "datos.txt";         -- Fichero con los datos
      BASE_INSTRUC       : std_logic_vector(31 downto 0) := X"00400000";
      BASE_DATA          : std_logic_vector(31 downto 0) := X"10010000";
      N_CYCLES           : positive := 100              -- Numero de ciclos a ejecutar
   );
end processorRV_tb;

architecture tb of processorRV_tb is

   component processorRV
      port(
         Clk         : in  std_logic; -- Reloj activo flanco subida
         Reset       : in  std_logic; -- Reset asincrono activo nivel alto
         -- Instruction memory
         IAddr       : out std_logic_vector(31 downto 0); -- Direccion
         IDataIn     : in  std_logic_vector(31 downto 0); -- Dato leido
         -- Data memory
         DAddr       : out std_logic_vector(31 downto 0); -- Direccion
         DRdEn       : out std_logic;                     -- Habilitacion lectura
         DWrEn       : out std_logic;                     -- Habilitacion escritura
         DDataOut    : out std_logic_vector(31 downto 0); -- Dato escrito
         DDataIn     : in  std_logic_vector(31 downto 0)  -- Dato leido
      );
   end component;

   component memory_instr is
   generic(
      INIT_FILENAME  : string := "instrucciones"; -- nombre fichero con datos iniciales
      MEM_BASE_ADDR  : std_logic_vector(31 downto 0) := X"00400000"; -- Base de la mem
      MEM_SIZE       : integer := 1024            -- tamanio, en bytes (256 instr)                                                  
   );
   Port (
      Clk     : in std_logic ;                     -- Reloj
      Addr    : in std_logic_vector(31 downto 0);  -- Direccion de lectura o escritura
      RdEn    : in std_logic ;                     -- Habilitacion de lectura (si =1)
      WrEn    : in std_logic ;                     -- Habilitacion de escritura (si =1)
      DataIn  : in std_logic_vector(31 downto 0);  -- Dato a escribir
      DataOut : out std_logic_vector(31 downto 0)  -- Dato leido
   );
  end component memory_instr;

   component memory_data is
   generic(
      INIT_FILENAME  : string := "datos";         -- nombre fichero con datos iniciales
      MEM_BASE_ADDR  : std_logic_vector(31 downto 0) := X"10010000"; -- Base de la mem
      MEM_SIZE       : integer := 4096            -- tamanio, en bytes (256 instr)                                                  
   );
   Port (
      Clk     : in std_logic ;                     -- Reloj
      Addr    : in std_logic_vector(31 downto 0);  -- Direccion de lectura o escritura
      RdEn    : in std_logic ;                     -- Habilitacion de lectura (si =1)
      WrEn    : in std_logic ;                     -- Habilitacion de escritura (si =1)
      DataIn  : in std_logic_vector(31 downto 0);  -- Dato a escribir
      DataOut : out std_logic_vector(31 downto 0)  -- Dato leido
   );
   end component memory_data;

   -- Constantes:

   constant CLK_PERIOD  : time := 10 ns;   -- Periodo de reloj
   constant RESET_TIME  : time := 50 ns;   -- Tiempo inicial de reset

   --  Seniales de interconexion entre bloques:
   signal clk         : std_logic;
   signal reset       : std_logic;
   -- Instruction memory
   signal iAddr      : std_logic_vector(31 downto 0);
   signal iDataIn    : std_logic_vector(31 downto 0);
   -- Data memory
   signal dAddr      : std_logic_vector(31 downto 0);
   signal dRdEn      : std_logic;
   signal dWrEn      : std_logic;
   signal dDataOut   : std_logic_vector(31 downto 0);
   signal dDataIn    : std_logic_vector(31 downto 0);

   -- Seniales de control del testbench:
   signal endSimulation : boolean := false;

begin

   -- Instanciaciones del procesador y las memorias:

   i_processor : processorRV
      port map (
         Clk       => clk,
         Reset     => reset,
         -- Instruction memory
         IAddr     => iAddr (31 downto 0),
         IDataIn   => iDataIn (31 downto 0),
         -- Data memory
         DAddr     => dAddr (31 downto 0),
         DRdEn     => dRdEn,
         DWrEn     => dWrEn,
         DDataOut  => dDataOut (31 downto 0),
         DDataIn   => dDataIn (31 downto 0)
      );

   i_inst_mem : memory_instr
      generic map (
         INIT_FILENAME => INIT_FILENAME_INST,
         MEM_BASE_ADDR => BASE_INSTRUC,
         MEM_SIZE      => 1024
      )
      port map (
         Clk     => clk,
         Addr    => iAddr (31 downto 0),
         RdEn    => '1',
         WrEn    => '0',
         DataIn  => x"00000000",
         DataOut => iDataIn (31 downto 0)
      );

   i_data_mem : memory_data
      generic map (
         INIT_FILENAME => INIT_FILENAME_DATA,
         MEM_BASE_ADDR => BASE_DATA,
         MEM_SIZE      => 8192
      )
      port map(
         Clk     => clk,
         Addr    => dAddr (31 downto 0),
         RdEn    => dRdEn,
         WrEn    => dWrEn,
         DataIn  => dDataOut (31 downto 0),
         DataOut => dDataIn (31 downto 0)
      );
   
   -- Generacion del reloj:
   gen_clk: process
   begin
      while not endSimulation loop
         clk <= '0';
         wait for CLK_PERIOD/2;
         clk <= '1';
         wait for CLK_PERIOD/2;
      end loop;
      wait;
   end process;

   -- Proceso principal: reset y ejecucion de N_CYCLES ciclos:
   rst_run: process
   begin
      report "COMIENZA LA SIMULACION";
      reset <= '1';
      wait for RESET_TIME;
      reset <= '0';
      for i in 1 to N_CYCLES loop
         wait until rising_edge (clk);
      end loop;
      wait until rising_edge (clk);
      endSimulation <= true;
      report "SIMULACION FINALIZADA. COMPROBAR FORMA DE ONDAS.";
      wait;
   end process;

end architecture;
