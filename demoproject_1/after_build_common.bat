@echo off
set keil_compiler_include_floder=%1
set full_name_path=%2
set linker_output_file_name=%3
echo keil_compiler_include_floder %keil_compiler_include_floder%
echo full_name_path %full_name_path%
echo linker_output_file_name %linker_output_file_name%

%keil_compiler_include_floder%\..\bin\fromelf.exe --bin -o "bin/" "%full_name_path%"
echo fromelf1
%keil_compiler_include_floder%\..\bin\fromelf.exe -acd --interleave=source -o "bin\%linker_output_file_name%.disasm" "%full_name_path%"
echo fromelf2

::..\..\..\tool\prepend_header\prepend_header.exe -r "..\..\..\tool\keys\rtk_rsa.pem" -t app_code -p "bin\%linker_output_file_name%.bin" -m 1 -c crc -a "..\..\..\tool\key.json"
::

..\tool\prepend_header\prepend_header.exe -t app_code -p "bin\app.bin" -m 1 -c sha256 -a "..\tool\key.json"
echo prepend_header
..\tool\MD5\md5.exe "bin\app.bin"
..\tool\MD5\md5.exe "bin\app_MP.bin"
echo md5
::..\..\..\tool\srec_cat "bin\%linker_output_file_name%.bin" -binary -offset 0x80D000 -o "Objects\%linker_output_file_name%.hex" -intel
del "bin\app_MP.bin"