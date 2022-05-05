
.\bin\srec_cat.exe .\hekk\hekk.hex -intel -offset - -minimum-addr .\hekk\hekk.hex -intel -o hekk.bin -Binary
.\bin\srec_cat.exe .\MDK-ARM\obj\project.hex -intel -fill 0xFF 0x8000000 0x8004000 .hekk\hekk.hex -intel -o hekk_BootAndMainProg.hex -intel