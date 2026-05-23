# RC Dual Servos - ESP32C3 BLE Master/Slave Framework

猫头鹰哨兵 ESP-IDF BLE 通讯框架项目，包含master和slave两个ESP32C3项目。

## 项目结构

- **master/**: BLE客户端项目，主动扫描并连接到slave设备
- **slave/**: BLE服务器项目，广播并等待master连接

## 功能特性

- ESP32C3 BLE通信
- GitHub Actions自动编译
- 自动生成merged.bin固件文件

## 编译

项目配置了GitHub Actions CI/CD，每次推送到main分支时会自动编译。

也可以本地编译：
```bash
cd master
idf.py set-target esp32c3
idf.py build

cd ../slave
idf.py set-target esp32c3
idf.py build
```

## 烧录

```bash
# 烧录 master
esptool.py --chip esp32c3 write_flash 0x0 master/build/master_merged.bin

# 烧录 slave
esptool.py --chip esp32c3 write_flash 0x0 slave/build/slave_merged.bin
```
