Build: 
  vscode + esp idf (works on 5.4.2) with arduino as component. 
  Create symlink: cmd in project/components, enter - mklink /D arduino-esp32 "C:\Users\User\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.1"
Flash: 
  [Flash Download Tool](https://docs.espressif.com/projects/esp-test-tools/en/latest/esp32/production_stage/tools/flash_download_tool.html)
  offsets: bootloader [0x1000], hall_sensor.bin [0x10000], partition_table [0x8000]
  spi mode - dio, baudrate - 921600
