- shell terminal yang digunakan ialah zsh dan menggunakan ohmyzsh
https://medium.com/@satriajanaka09/setup-zsh-oh-my-zsh-powerlevel10k-on-ubuntu-20-04-c4a4052508fd

- untuk jam bisa digunakan sudo ./time.sh (syarat harus terkoneksi dengan internet terlebih dahulu)
- untuk joystick connect dengan ./joy.sh dan disconnect dengan ./disconnect_joy.sh
- nyalakan joystick seperti biasa

- untuk kode arduino ada pada lssArduino.zip
- terdapat perubahan pada library yang menyesuaikan dari kinematics omniwheel 4 roda
- vx vy dan w ialah local (sudut pandang robot) nilainya diread dari serial
- getrpm untuk 4 roda, ini saya gunakan karena pada library itu getrpm didapat dari speedPPS 
yang telah di sampling (bisa di cari pada motorwheel.cpp atau motorwheel.h) kemudian nilainya 
diwrite ke serial

- untuk raspi sendiri yang harus di install ialah ros2 yang fulldesktop
- kemudian install atau upgrade python dan juga pip install library seperti serial dan juga i2c
- jika masih ada error berarti library atau env belum terinstall sepenuhnya

- package bno055 di clone dari github cara run nya ialah 
ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_params_i2c.yaml

- package rplidar_ros di clone dari github cara run nya ialah 
ros2 launch rplidar_ros rplidar_a1_launch.py 

- package joy di clone dari github caran run nya ialah
ros2 run joy joy_node

- package robot 
    - node_bno hanya mengambil yaw saja dan tidak jadi dipakai
    - save_imu merupakan node yang berfungsi untu merecord data dari topic /bno055/imu yang
    dipublish dari package bno055 lalu di save dalam file.csv (sampai ctrl-c)
    - node_serial digunakan untuk read and write data ke arduino
    - ros2 run robot node_serial
    - ros2 run robot save_imu
    - ros2 run robot save_odom

- package nexus
    - ros2 run nexus main 
    - ros2 run nexus icp_node (tidak jadi)
    - pada main.cpp subscribe data motor lalu di olah dan juga lidar
    - mensubscribe data joystick
    - mensubscribe data velocity dari icp_node (tidak jadi)
    - mempublish data velocity pada topic joy_command lalu di subscribe oleh node serial
    - X manual O otomatis

- jika pada saat compile error clock_skew maka touch file yang error
    contoh 1: touch /home/divspanpi2/robo_ws/src/nexus/src/lidar/icp_node.cpp
    contoh 2: touch /home/divspanpi2/robo_ws/src/nexus/src/main.cpp

pada ~/.zshrc telah di tambahkan 
source /opt/ros/humble/setup.zsh
source ~/robo_ws/install/setup.zsh
sehingga tidak perlu sourcing lagi

untuk remote ssh divspanpi2@192.168.1.1 (ip menyesuaikan dan juga bisa di bikin static)

build nya colcon build
jika ingin satu package saja maka colcon build --package-select my_package

PR dan Notes
- pid dan kinematics motor bisa disesuaikan dan dioptimalkan lagi
- icp belum sempurna dan juga nnti dapat di buat rviz nya
- mapping juga belum
- kalau icp bisa di predefine mapnya
- kalau nntinya jadi pakai slam itu robotnya harus jalan2 dulu untuk save mapnya  dan bisa pake tombol joystick untuk jalan dan save map dan path



urutan : 
ros2 run robot node_serial
ros2 run robot joy joynode
ros2 run nexus main
