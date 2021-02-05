# VL53L0X API Component for Esp32

This project based on
[STSW-IMG005](https://www.st.com/en/embedded-software/stsw-img005.html) (STMicroelectroics Provide) software packages. in this project not provied sources in STSW-IMG005. you can download above link.

## Prepare VL53L0X Packages

after download and decompress VL53L0X API files. you can check folder structure

```
vl53l0x/api/folder
├── Api             <== it's your API PATH
│   ├── core
│   │   ├── inc
│   │   └── src
│   └── platform
│       ├── inc
│       └── src
├── ...

```

## Configure API Path

Clone repository
```shell
$ cd components
$ git clone https://github.com/ys-oh/vl53l0x_esp32

$ cd vl53l0x_esp32
```


In component folder. run script to configure api path

```shell
$ ./configure_api_path.sh "your/vl53l0x/api/path"
```


## Build your Project

Return to your Project Folder. then build

```shell
$ cd "your/esp32/project/folder"

$ idf.py build
```

enjoy your project :)

