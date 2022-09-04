SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\WinAVR-20100110\bin;C:\WinAVR-20100110\utils\bin;C:\Program Files (x86)\STMicroelectronics\st_toolset\asm;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files (x86)\Microchip\MPLAB C32 Suite\bin;Z:\mingw\bin;C:\Users\libri\AppData\Local\Programs\Python\Python39\Scripts\;C:\Users\libri\AppData\Local\Programs\Python\Python39\;C:\Users\libri\AppData\Local\Microsoft\WindowsApps;C:\adb;C:\Program Files (x86)\dotnet\;
SET CPU_TYPE=ARMCM4_FP
SET CPU_VENDOR=ARM
SET UV2_TARGET=RTL8786BF
SET CPU_CLOCK=0x00B71B00
before_build_common.bat C:\Keil_v5\
before_build_special.bat
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\system_rtl8762c.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\app_task.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\iohandle.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\rtl876x_io_dlps.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\rtl876x_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\rtl876x_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\ias.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\lls.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\tps.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\bas.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\kns.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\dis.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\pxp_app.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\objects\overlay_mgr.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via ".\objects\startup_rtl8762c._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via ".\Objects\rtp8763_demo.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" ".\Objects\rtp8763_demo.axf" --i32combined --output ".\Objects\rtp8763_demo.hex"
after_build_common.bat C:\Keil_v5\ARM\ARMCC\include W:\git\RTL8763BF\demoproject_1\keil\Objects\rtp8763_demo.axf rtp8763_demo
after_build_special.bat
