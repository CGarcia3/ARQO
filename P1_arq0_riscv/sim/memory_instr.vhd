--------------------------------------------------------------------------------
-- EPS - UAM. Laboratorio de ArqO 2022
--
-- Memoria de instrucciones simple. For RISC V
-- Lee fichero de instrucciones exportado desde RARs
--------------------------------------------------------------------------------

library std;
use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_textio.all;

entity memory_instr is
   generic(
      INIT_FILENAME   : string := "instrucciones.txt"; -- nombre fichero con datos iniciales
      MEM_BASE_ADDR   : std_logic_vector(31 downto 0) := X"00040000"; -- Base de la mem
      MEM_SIZE        : integer := 1024            -- tamanio, en bytes (256 instr)                                    
   );
   Port (
      Clk     : in std_logic ;                     -- Reloj
      Addr    : in std_logic_vector(31 downto 0);  -- Direccion de lectura o escritura
      RdEn    : in std_logic ;                     -- Habilitacion de lectura (si =1)
      WrEn    : in std_logic ;                     -- Habilitacion de escritura (si =1)
      DataIn  : in std_logic_vector(31 downto 0);  -- Dato a escribir
      DataOut : out std_logic_vector(31 downto 0)  -- Dato leido
   );
end memory_instr;

architecture Behavioral of memory_instr is 
        
   type matrix is array(0 to (MEM_SIZE/4)-1) of std_logic_vector(31 downto 0);
   signal memo: matrix;

   signal rAddr : std_logic_vector(31 downto 0);
   signal effectAddress: integer;
   signal addr_in_range: boolean;
begin

init_mem: process (clk)
   variable initial_load : boolean := true;
   variable address : std_logic_vector(31 downto 0);
   variable datum : std_logic_vector(31 downto 0);
   file bin_file : text open READ_MODE is INIT_FILENAME;
   variable  current_line : line;
   variable str: string(1 to 2);
   variable effective_addr: integer;
begin

   if initial_load then 
      -- primero iniciamos la memoria con ceros
      for i in 0 to (MEM_SIZE/4)-1 loop
         memo(i) <= (others => '0');
      end loop; 
      -- luego cargamos el archivo en la misma
      -- Lee fichero instrucciones generado por RARS
      -- Se deben descartar las dos primeras lineas
        readline (bin_file, current_line);
        readline (bin_file, current_line);
      while (not endfile (bin_file)) loop
         readline (bin_file, current_line);
         read(current_line, str); --Read chars "0x"
         hread(current_line, address);
         read(current_line, str); --Read chars "  "
         read(current_line, str); --Read chars "0x"
         hread(current_line, datum);

    	   --report "Direccion: " & to_hstring(address) & "   Data: " & to_hstring(datum);

         assert CONV_INTEGER(address(31 downto 0)) < (CONV_INTEGER(MEM_BASE_ADDR) + MEM_SIZE) 
            report "Direccion fuera de rango (muy alto): " & to_hstring(address)
            severity failure;
         assert CONV_INTEGER(address(31 downto 0)) >= CONV_INTEGER(MEM_BASE_ADDR) 
            report "Direccion fuera de rango (muy bajo): " & to_hstring(address)
            severity failure;
         effective_addr := CONV_INTEGER( address(31 downto 2) ) - CONV_INTEGER(MEM_BASE_ADDR(31 downto 2));
         memo( effective_addr ) <= datum;
      end loop;

      -- por ultimo cerramos el archivo y actualizamos el flag de memoria cargada
      file_close (bin_file);
      initial_load := false;

      report "Se ha cargado la memoria '" & INIT_FILENAME & "'" severity note;

   elsif rising_edge(clk) then                    
      if (WrEn = '1') then
         assert ( CONV_INTEGER(Addr(31 downto 0)) < MEM_SIZE )
            report "Direccion fuera de rango en el fichero de la memoria"
            severity failure;
         memo( CONV_INTEGER( Addr(31 downto 2) ) ) <= DataIn;
      end if;
   end if;
end process;

rAddr   <= Addr when RdEn = '1'; -- latch: no ejecutan nuevas lecturas si no se activa RdEn
effectAddress <= CONV_INTEGER( rAddr(31 downto 2) ) - CONV_INTEGER(MEM_BASE_ADDR(31 downto 2));
addr_in_range <= true when ( (CONV_INTEGER(rAddr(31 downto 0)) < (CONV_INTEGER(MEM_BASE_ADDR) + MEM_SIZE)) 
                         and (CONV_INTEGER(rAddr(31 downto 0)) >= CONV_INTEGER(MEM_BASE_ADDR))) else false;
DataOut <= memo( effectAddress ) when addr_in_range else x"FABADA00";

end Behavioral;
