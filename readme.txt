How to Configurate the circuit
The ulcd should be connected to the mbed, the pins are connected like lab5(ulcd)

How to set the threshold angle
1. Firstly, to compile the program, you need to be under dictionary /hw3/src/model_deploy
2. Then enter the command $ sudo mbed compile --source . --source ~/ee2405/mbed-os/ -m B_L4S5I_IOT01A -t GCC_ARM -f
to compile the program
3. After that, enter the command $ sudo screen /dev/ttyACM0 to the screen, 
then enter the command $ /GUI_Thread/run 3 1 to go to the GUI mode
4. The led3 will light to indicate the start of GUI mode
5. In the GUI mode, you can make gestures to select different threshold angles(15°，30°，45° and 60°）
6. The gesture "ring" represents shifting down, "slope" means shifting up the selected threshold angles
7. Push the user button to select the angle

How to detect current angle
1. Then enter the command $ /DET_Thread/run 2 1 in the screen to enter the detection mode
2. During the first second, the led2 will light to indicate the initialization of gravity acceleration
3. After one second, the the led2 will be off and the led1 begin to light, which means you can tilt the mbed
4. From this moment, the ulcd begin to show the current tilted angle every 0.1s
5. open another terminal, $ cd ~/ee2405/mbed10/10_2_MQTT_Python, $ sudo python3 wifi_mqtt/mqtt_client.py,
the client will begin to receive message from mbed/linux
6. Of course, the linux IP address and wifi information in the code need to be modified accroding to your IP and wifi like lab10
7. After the accelerator detects 3 tilt events (angle > threshold angle), it will send these 3 events to client
8. Then the detection mode ends, the led1 will be off. 
