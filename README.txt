daniel@Pi5:~/pico $ git clone https://github.com/danjperron/PicowSendBT
daniel@Pi5:~/pico $ cd PicowSendBT
daniel@Pi5:~/pico/PicowSendBT $ mkdir build
daniel@Pi5:~/pico/PicowSendBT $ cd build
daniel@Pi5:~/pico/PicowSendBT/build $ make ..
daniel@Pi5:~/pico/PicowSendBT/build $ make
daniel@Pi5:~/pico/PicowSendBT/build $ sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program PicowSendBT.elf verify reset exit"
daniel@Pi5:~ $ 
daniel@Pi5:~ $ sudo bluetoothctl
Agent registered
[bluetooth]# agent on
Agent is already registered
[bluetooth]# scan on
Discovery started
[CHG] Controller D8:3A:DD:A5:B3:1B Discovering: yes
[NEW] Device FC:E2:6C:11:3D:82 MacBook Air de Daniel
[NEW] Device 28:CD:C1:02:0E:D9 PicowSendBT 28:CD:C1:02:0E:D9
[bluetooth]# trust 28:CD:C1:02:0E:D9
[bluetooth]# pair 28:CD:C1:02:0E:D9
Attempting to pair with 28:CD:C1:02:0E:D9
[CHG] Device 28:CD:C1:02:0E:D9 Connected: yes
Request confirmation
[agent] Confirm passkey 795112 (yes/no): yes
[CHG] Device 28:CD:C1:02:0E:D9 Bonded: yes
[CHG] Device 28:CD:C1:02:0E:D9 UUIDs: 00001101-0000-1000-8000-00805f9b34fb
[CHG] Device 28:CD:C1:02:0E:D9 ServicesResolved: yes
[CHG] Device 28:CD:C1:02:0E:D9 Paired: yes
Pairing successful
[CHG] Device 28:CD:C1:02:0E:D9 ServicesResolved: no
[CHG] Device 28:CD:C1:02:0E:D9 Connected: no
[bluetooth]# quit

daniel@Pi5:~ $ sudo rfcomm bind /dev/rfcomm1 28:CD:C1:02:0E:D9 1
daniel@Pi5:~ $ cat /dev/rfcomm1
$02D8005B0016005A001600660016006C0017006B0016006B0016
$02D800580016005A0016006C001600770016006F001600700016
$02D800580016006B001600730016007400160073001600730016
$02D600580016006B001600740016007600160073001600750016
$02D800560016006B001600740016007500160072001600740016
$02D800580016006C001700720016007600160073001600750016
$02D8005800160069001600750016007500160072001600740016
^C
daniel@Pi5:~ $ 
