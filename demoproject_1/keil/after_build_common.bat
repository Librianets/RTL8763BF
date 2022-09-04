@echo off
set keil_compiler_include_floder=%1
set full_name_path=%2
set linker_output_file_name=%3
set fromelf_bin=app
echo keil_compiler_include_floder = %keil_compiler_include_floder%
echo full_name_path = %full_name_path%
echo linker_output_file_name = %linker_output_file_name%
echo fromelf_bin = %fromelf_bin%

%keil_compiler_include_floder%\..\bin\fromelf.exe --bin --output=.\bin\ "%full_name_path%"
echo fromelf1

%keil_compiler_include_floder%\..\bin\fromelf.exe -acd --interleave=source -o "bin\%linker_output_file_name%.disasm" "%full_name_path%"
echo fromelf2
ren .\bin\%fromelf_bin%.bin %linker_output_file_name%.bin
ren .\bin\%fromelf_bin%.trace %linker_output_file_name%.trace
echo ren
..\tool\prepend_header\prepend_header.exe -t app_code -p "bin\%linker_output_file_name%.bin" -m 1 -c sha256 -a "..\..\..\tool\key.json"
echo prepend_header
..\tool\MD5\md5.exe "bin\%linker_output_file_name%.bin"
echo md51
..\tool\MD5\md5.exe "bin\%linker_output_file_name%_MP.bin"
echo md52
del "bin\%linker_output_file_name%_MP.bin"
echo del


::..\..\..\tool\prepend_header\prepend_header.exe -r "..\..\..\tool\keys\rtk_rsa.pem" -t app_code -p "bin\%linker_output_file_name%.bin" -m 1 -c crc -a "..\..\..\tool\key.json"

::..\..\..\tool\srec_cat "bin\%linker_output_file_name%.bin" -binary -offset 0x80D000 -o "Objects\%linker_output_file_name%.hex" -intel
