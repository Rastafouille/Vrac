Sommaire général

- [Arbotix](#arbotix)
- [ROS](#ros)
- [Gazebo](#gazebo)
- [pcDuino](#pcduino)
- [Le Robot du CEA](#le-robot-du-cea)
- [Markdown](#markdown)
- [LaTeX](#latex)
- [Git](#git)
- [Robots Hexapodes](#robots-hexapodes)
- [Divers](#divers)
- [Pour la suite](#pour-la-suite)


[TOC]

# ArbotiX
- Téléchargement de **DynaManager** : <http://learn.trossenrobotics.com/arbotix/1-using-the-tr-dynamixel-servo-tool#&panel1-1>
- ArbotiX-M Hardware Overview : <http://learn.trossenrobotics.com/arbotix/arbotix-getting-started/38-arbotix-m-hardware-overview#&panel1-1>

Mais problème avec l'ArbotiX, impossible de le flasher avec le firmware pour ROS.

## Flasher Bootloader d'un Arduino (IDE)
- En cas de problème avec l'USBasp : <http://forum.arduino.cc/index.php?topic=212648.0>
	Il y est donné des udev rules pour l'USBasp, à écrire dans un ficher comme `/etc/udev/rules.d/90-usbasp.rules` :

		# From https://wiki.archlinux.org/index.php/Udev#Accessing_Firmware_Programmers_and_USB_Virtual_Comm_Devices
		SYSFS{idVendor}=="16c0",  SYSFS{idProduct}=="05dc", MODE="0660", GROUP="users"
		SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="05dc", GROUP="users", MODE="0666"

- Utiliser un Arduino comme ISP Programmer : <http://arduino.cc/en/Tutorial/ArduinoISP>
- Version plus récente de l'ArduinoISP : <https://github.com/rsbohn/ArduinoISP/>
- L'auto-reset de l'Arduino UNO servant d'ISP peut poser problème. Pour éviter cela, ajouter un condensateur de 10 µF entre la pin RESET et GND. (<http://playground.arduino.cc/Main/DisablingAutoResetOnSerialConnection>)
- Test avec le Bootloader Optiboot (la version 644P est en béta), mais impossible de le flasher : <https://code.google.com/p/optiboot/>
- Méthode en utilisant `gdb` pour ralentir le proccess, mais sans succès : <http://www.xuan-wu.com/2012-11-19-Arudino-Uno-Bootloader>
- Tuto pour ISP un 644P pour le cablâge : <http://smallchaoslab.blogspot.fr/2012/10/tutorial-programmation-isp-dun-atmega.html>
- Burning the Sanguino Bootloader using Arduino as ISP : <http://reprap.org/wiki/Burning_the_Sanguino_Bootloader_using_Arduino_as_ISP>
- Leonardo as Programmer (remplace `#define RESET SS` par `#define RESET 10`)
- FTDI as Programmer using Bitbang method : <http://doswa.com/2010/08/24/avrdude-5-10-with-ftdi-bitbang.html>

Toujours le même problème, lorsqu'on tente d'écrire un programme sans bootloader avec :

		$ avrdude -c stk500v1 -p m644p -P /dev/ttyACM0 -b 19200 -Uflash:w:/tmp/build4656764002673238722.tmp/ArbotiXBlink.cpp.hex:i

On obtient cette erreur :
		avrdude: verification error, first mismatch at byte 0x0002
		         0x7d != 0xb9

L'écriture des *fuse* avec la commande :

		avrdude -p m644p -c stk500v1 -P /dev/ttyACM0 -b 19200 -B 1 -v -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xDC:m -Ulfuse:w:0xFF:m

Donne l'erreur suivante :

		avrdude: verification error, first mismatch at byte 0x0000
		         0x3f != 0x0f

La lecture de l'état avec `-v` est bonne.

Problème à l'unlock du bootloader et à l'écriture d'un programme ou bootloader lors de la vérification.

## Programmation *Manuelle* d'un Arduino
D'après : <http://forum.arduino.cc/index.php?topic=54190.0>

- Voir si la puce répond

		avrdude -p m644p -P /dev/ttyACM0 -c avrisp -b 19200 -v

- Fuses

		avrdude -p m644p -P /dev/ttyACM0 -c avrisp -b 19200 -U lfuse:w:0xff:m -U hfuse:w:0xdc:m -U efuse:w:0xfd:m

- Unlock Bootloader

		avrdude -p m644p -P /dev/ttyACM0 -c avrisp -b 19200 -U lock:w:0x3F:m

- Flash Bootloader

		avrdude -p m644p -P /dev/ttyACM0 -c avrisp -b 19200 -e -U flash:w:ATmegaBOOT_644P.hex

- Lock Bootloader

		avrdude -p m644p -P /dev/ttyACM0 -c avrisp -b 19200 -U lock:w:0x0F:m

- Upload a hex file

		avrdude -p m644p -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:/tmp/build32029759636681119.tmp/ArbotiXBlink.cpp.hex

Remarque : La commande de *Unlock Bootloader* ne fonctionne pas et le chargement d'un programme ou du Bootloader s'arrête en cours (sync()...). À tester avec un condensateir de 10 µF sur l'Arduino UNO...

## Mise à jour des fichiers `hardware` pour Arduino 1.6.x
- Procédure de migration : <https://github.com/arduino/Arduino/wiki/Arduino-Hardware-Cores-migration-guide-from-1.0-to-1.6>
- Copiage du platform.txt de celui Arduino d'origine
- Problème de `prog_char` surmonté avec ce lien : <http://postwarrior.com/arduino-ethershield-error-prog_char-does-not-name-a-type/> (un peu sale quand même...)
- Problème dans le fichier `HardwareSerial.cpp` à cause de vieilleries maintenant dépréciée -> Commentaire de la partie de code posant problème (très sale) (problème mentionné ici : <http://forum.arduino.cc/index.php?topic=92364.0> et là <https://code.google.com/p/arduino/issues/detail?id=955>)

## Code déjà présent dans le robot
<https://github.com/TXBDan/BETH_Arduino>
Au démarrage, le message suivant appraît (38400 bauds)

		DAN’S HEXAPOD! ! !
		System Voltage : 12,00 volts

On remarque que lors d'opérations trop brusques, trop rapides, le robot semble s'éteindre puisqu'il s'effondre sur le sol, puis se redresse. On suppose donc que dans le cas d'un mouvement trop rapide, l'appel de courant est trop important pour l'alimentation à découpage de 5 A utilisée. L'ArbotiX-M subit donc un reset. Il faudra donc bien faire attention à l'alimentation avec l'utilisation du pcDuino (même su sur une batterie LiPo, le courant en sortie peut être très important).

## Quelques commandes avrdude

Visualiser les fuse bits d'un ATmega avec un programmer ISP :

    $ sudo avrdude -p m644p -c usbasp -v

    avrdude: Version 5.11.1, compiled on Oct 30 2011 at 10:41:10
             Copyright (c) 2000-2005 Brian Dean, http://www.bdmicro.com/
             Copyright (c) 2007-2009 Joerg Wunsch

             System wide configuration file is "/etc/avrdude.conf"
             User configuration file is "/home/rr244992/.avrduderc"
             User configuration file does not exist or is not a regular file, skipping

             Using Port                    : /dev/parport0
             Using Programmer              : usbasp
             AVR Part                      : ATMEGA644P
             Chip Erase delay              : 9000 us
             PAGEL                         : PD7
             BS2                           : PA0
             RESET disposition             : dedicated
             RETRY pulse                   : SCK
             serial program mode           : yes
             parallel program mode         : yes
             Timeout                       : 200
             StabDelay                     : 100
             CmdexeDelay                   : 25
             SyncLoops                     : 32
             ByteDelay                     : 0
             PollIndex                     : 3
             PollValue                     : 0x53
             Memory Detail                 :

                                      Block Poll               Page                       Polled
               Memory Type Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack
               ----------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------
               eeprom        65    10   128    0 no       2048    8      0  9000  9000 0xff 0xff
               flash         33     6   256    0 yes     65536  256    256  4500  4500 0xff 0xff
               lock           0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
               lfuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
               hfuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
               efuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
               signature      0     0     0    0 no          3    0      0     0     0 0x00 0x00
               calibration    0     0     0    0 no          1    0      0     0     0 0x00 0x00

             Programmer Type : usbasp
             Description     : USBasp, http://www.fischl.de/usbasp/

    avrdude: auto set sck period (because given equals null)
    avrdude: AVR device initialized and ready to accept instructions

    Reading | ################################################## | 100% 0.01s

    avrdude: Device signature = 0x1e960a
    avrdude: safemode: lfuse reads as FF
    avrdude: safemode: hfuse reads as DC
    avrdude: safemode: efuse reads as FD

    avrdude: safemode: lfuse reads as FF
    avrdude: safemode: hfuse reads as DC
    avrdude: safemode: efuse reads as FD
    avrdude: safemode: Fuses OK

    avrdude done.  Thank you.

Lecture du lock bit :

    $ sudo avrdude -p m644p -c usbasp -U lock:r:-:h

    avrdude: AVR device initialized and ready to accept instructions

    Reading | ################################################## | 100% 0.01s

    avrdude: Device signature = 0x1e960a
    avrdude: reading lock memory:

    Reading | ################################################## | 100% 0.00s

    avrdude: writing output file "<stdout>"
    0xf

    avrdude: safemode: Fuses OK

    avrdude done.  Thank you.

Écriture du bootloader :

    $ sudo avrdude -p m644p -c usbasp -e -U flash:w:ATmegaBOOT_644P.hex

    avrdude: AVR device initialized and ready to accept instructions

    Reading | ################################################## | 100% 0.01s

    avrdude: Device signature = 0x1e960a
    avrdude: erasing chip
    avrdude: reading input file "ATmegaBOOT_644P.hex"
    avrdude: input file ATmegaBOOT_644P.hex auto detected as Intel Hex
    avrdude: writing flash (65382 bytes):

    Writing | ################################################## | 100% 27.04s



    avrdude: 65382 bytes of flash written
    avrdude: verifying flash memory against ATmegaBOOT_644P.hex:
    avrdude: load data flash data from input file ATmegaBOOT_644P.hex:
    avrdude: input file ATmegaBOOT_644P.hex auto detected as Intel Hex
    avrdude: input file ATmegaBOOT_644P.hex contains 65382 bytes
    avrdude: reading on-chip flash data:

    Reading | ################################################## | 100% 18.31s



    avrdude: verifying ...
    avrdude: verification error, first mismatch at byte 0x0000
             0xff != 0x0c
    avrdude: verification error; content mismatch

    avrdude: safemode: Fuses OK

    avrdude done.  Thank you.

Écriture du lock bit pour permettre l'écriture du bootloader (description des lock bits à la page 280 de la datasheet de l'ATMega644p) :

    $ sudo avrdude -p m644p -c usbasp -U lock:w:0x3F:m

    avrdude: AVR device initialized and ready to accept instructions

    Reading | ################################################## | 100% 0.01s

    avrdude: Device signature = 0x1e960a
    avrdude: erasing chip
    avrdude: reading input file "0x3F"
    avrdude: writing lock (1 bytes):

    Writing | ################################################## | 100% 0.01s

    avrdude: 1 bytes of lock written
    avrdude: verifying lock memory against 0x3F:
    avrdude: load data lock data from input file 0x3F:
    avrdude: input file 0x3F contains 1 bytes
    avrdude: reading on-chip lock data:

    Reading | ################################################## | 100% 0.00s

    avrdude: verifying ...
    avrdude: verification error, first mismatch at byte 0x0000
             0x3f != 0x0f
    avrdude: verification error; content mismatch

    avrdude: safemode: Fuses OK

    avrdude done.  Thank you.

Écriture des fuse bits :

    $ sudo avrdude -p m644p -c usbasp -U lfuse:w:0xff:m -U hfuse:w:0xdc:m -U efuse:w:0xfd:m

    avrdude: AVR device initialized and ready to accept instructions

    Reading | ################################################## | 100% 0.01s

    avrdude: Device signature = 0x1e960a
    avrdude: reading input file "0xff"
    avrdude: writing lfuse (1 bytes):

    Writing | ################################################## | 100% 0.00s

    avrdude: 1 bytes of lfuse written
    avrdude: verifying lfuse memory against 0xff:
    avrdude: load data lfuse data from input file 0xff:
    avrdude: input file 0xff contains 1 bytes
    avrdude: reading on-chip lfuse data:

    Reading | ################################################## | 100% 0.00s

    avrdude: verifying ...
    avrdude: 1 bytes of lfuse verified
    avrdude: reading input file "0xdc"
    avrdude: writing hfuse (1 bytes):

    Writing | ################################################## | 100% 0.00s

    avrdude: 1 bytes of hfuse written
    avrdude: verifying hfuse memory against 0xdc:
    avrdude: load data hfuse data from input file 0xdc:
    avrdude: input file 0xdc contains 1 bytes
    avrdude: reading on-chip hfuse data:

    Reading | ################################################## | 100% 0.00s

    avrdude: verifying ...
    avrdude: 1 bytes of hfuse verified
    avrdude: reading input file "0xfd"
    avrdude: writing efuse (1 bytes):

    Writing | ################################################## | 100% 0.00s

    avrdude: 1 bytes of efuse written
    avrdude: verifying efuse memory against 0xfd:
    avrdude: load data efuse data from input file 0xfd:
    avrdude: input file 0xfd contains 1 bytes
    avrdude: reading on-chip efuse data:

    Reading | ################################################## | 100% 0.00s

    avrdude: verifying ...
    avrdude: 1 bytes of efuse verified

    avrdude: safemode: Fuses OK

    avrdude done.  Thank you.

Calculateur de fuse bits : <http://www.engbedded.com/fusecalc/>

Lecture du programme actuel en passant par une liaison série et le Bootloader Arduino :

    $ avrdude -p m644p -c arduino -b 38400 -P /dev/ttyUSB0 -U flash:r:loadedFirmware.hex:i

Attention, la vitesse est ici de 38400 bauds mais elle est de 19200 pour un Arduino UNO.

Écrire un programme par la liaison série et le bootloader :

    $ avrdude -p m644p -c arduino -b 38400 -P /dev/ttyUSB0 -U flash:w:myProgram.hex:i

# ROS
## Utilisation des Dynamixels sous ROS avec l'USB2DYNAMIXEL
Cette partie utlise le paquet ROS "officiel" (<http://wiki.ros.org/dynamixel_motor>).

*Remarque* : Nous utilisons des servomoteurs Dynamixel AX-18A <http://support.robotis.com/en/product/dynamixel/ax_series/ax-18f.htm>

Pour la documentation, voir l'onglet **ELECTRIC** du [wiki de dynamixel_controllers](http://wiki.ros.org/dynamixel_controllers) (les onglets des versions plus récentes ne profitent pas de la documentation).

### Installation et test
Source : <http://wiki.ros.org/dynamixel_controllers/Tutorials/ConnectingToDynamixelBus>

- Installation des paquets ROS

		$ sudo apt-get install ros-hydro-dynamixel-motor

- Création d'un workspace (<http://wiki.ros.org/catkin/Tutorials/create_a_workspace>)

		$ mkdir -p ~/catkin_ws/src
		$ cd ~/catkin_ws/src
		$ catkin_init_workspace
		$ source devel/setup.bash

- Création d'un paquet (<http://wiki.ros.org/dynamixel_controllers/Tutorials/ConnectingToDynamixelBus>)

		$ cd ~/catkin_ws/src
		$ catkin_create_pkg my_dynamixel_tutorial dynamixel_controllers std_msgs rospy roscpp

- Pour régler les différents paramètres des Dynamixel, nous créons un fichier *launch* :

		$ cd my_dynamixel_tutorial/
		$ mkdir launch
		$ cd launch/
		$ vim controller_manager.launch

	Compléter ce fichier avec le texte suivant (à ajuster si besoin) :

    ```xml
    <launch>
        <node name="dynamixel_manager" pkg="dynamixel_controllers" type="controller_manager.py" required="true" output="screen">
            <rosparam>
                namespace: dxl_manager
                serial_ports:
                    pan_tilt_port:
                        port_name: "/dev/ttyUSB0"
                        baud_rate: 1000000
                        min_motor_id: 1
                        max_motor_id: 25
                        update_rate: 20
            </rosparam>
        </node>
    </launch>
    ```

- On vérifie le fonctionnement en lançant :

		$ roslaunch my_dynamixel_tutorial controller_manager.launch
	(Si cela ne fonctionne pas, penser au `source ~/catkin_ws/devel/setup.bash`)

	On devrait voir apparaître un message du genre :

		[INFO] [WallTime: 1425373923.941314] pan_tilt_port: Pinging motor IDs 1 through 25...
		[INFO] [WallTime: 1425375925.117468] pan_tilt_port: Found 15 motors - 15 AX-18 [2, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18], initialization complete.

- Observer le retour des moteurs avec

		$ rostopic echo /motor_states/pan_tilt_port

### Création d'un controleur pour une articulation
Source : <http://wiki.ros.org/dynamixel_controllers/Tutorials/CreatingJointPositionController>

- Création d'un fichier de configuration

		$ roscd my_dynamixel_tutorial
		$ vim tilt.yaml

	Écrire :

    ```yaml
    tilt_controller:
        controller:
            package: dynamixel_controllers
            module: joint_position_controller
            type: JointPositionController
        joint_name: tilt_joint
        joint_speed: 1.17
        motor:
            id: 18
            init: 512
            min: 170
            max: 700
    ```

    Remarque : en inversant `min` et `max`, on indique que le servomoteur est inversé.

- Créer un fichier launch

		$ cd launch
		$ vim start_tilt_controller.launch

	Écrire

    ```xml
    <launch>
        <!-- Start tilt joint controller -->
        <rosparam file="$(find my_dynamixel_tutorial)/tilt.yaml" command="load"/>
        <node name="tilt_controller_spawner" pkg="dynamixel_controllers" type="controller_spawner.py"
              args="--manager=dxl_manager
                    --port pan_tilt_port
                    tilt_controller"
              output="screen"/>
    </launch>
    ```

- Lancement du controleur créé précédemment

		$ roslaunch my_dynamixel_tutorial controller_manager.launch

- Lancement du fichier `launch` qui vient d'être créé

		$ roslaunch my_dynamixel_tutorial start_tilt_controller.launch

	Remarque, le processus s'exécude puis nous rend la main sur le terminal, c'est normal.

- On peut maintenant voir de nouveaux topics :

		$ rostopic list

		/diagnostics
		/motor_states/pan_tilt_port
		/rosout
		/rosout_agg
		/tilt_controller/command
		/tilt_controller/state

	- `/tilt_controller/command` sert à envoyer des commandes sous formes de messages de type `std_msgs/Float64` pour donner l'angle en radians (0 à 512) ;
    - `/tilt_controller/state` sert à donner l'état du servo sous la forme de messages de type `dynamixel_msgs/JointState` :

            Header header
            string name         # joint name
            int32[] motor_ids   # motor ids controlling this joint
            int32[] motor_temps # motor temperatures, same order as motor_ids

            float64 goal_pos    # commanded position (in radians)
            float64 current_pos # current joint position (in radians)
            float64 error       # error between commanded and current positions (in radians)
            float64 velocity    # current joint speed (in radians per second)
            float64 load        # current load
            bool is_moving      # is joint currently in motion

    - `/motor_states/pan_tilt_port` est une liste de messages de type `dynamixel_msgs/MotorState` :

            float64 timestamp   # motor state is at this time
            int32 id            # motor id
            int32 goal          # commanded position (in encoder units)
            int32 position      # current position (in encoder units)
            int32 error         # difference between current and goal positions
            int32 speed         # current speed (0.111 rpm per unit)
            float64 load        # current load - ratio of applied torque over maximum torque
            float64 voltage     # current voltage (V)
            int32 temperature   # current temperature (degrees Celsius)
            bool moving         # whether the motor is currently in motion

- Pour bouger le servo, il faut alors publier une commande

		rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -1.5

  Remarque : les `--` sont obligatoires pour une valeur négative afin de différentier une valeur négative d'un argument de type `-r`. Pour les valeurs positives, les `--` peuvent être omis.

### Controleur pour plusieurs servo
Source : <http://wiki.ros.org/dynamixel_controllers/Tutorials/Creatingmetacontroller>

On fait cette fois appel au [`joint_trajectory_action_controller`](https://github.com/arebgun/dynamixel_motor/blob/master/dynamixel_controllers/src/dynamixel_controllers/joint_trajectory_action_controller.py). Il permet de définir un ensemble de liaisons qui peuvent alors recevoir des consignes de trajectoires qui sont ensuite exécutées. Il n'y a pas d'interpolation.

Description rapide de l'auteur :

> It was meant to control multiple joints through the FollowJointTrajectoryAction that plugs-in nicely into the whole ROS Arm Trajectory planning/execution pipeline. Of course it doesn't have to be an arm, you should be able to control any set of related joints.

- Création d'un fichier de configuration des servos

		$ roscd my_dynamixel_tutorial
		$ vim tilt2.yaml

	Écrire :

    ```yaml
    joint14_controller:
        controller:
            package: dynamixel_controllers
            module: joint_position_controller
            type: JointPositionController
        joint_name: toxa_2
        joint_speed: 1.0
        motor:
            id: 14
            init: 512
            min: 300
            max: 700

    joint16_controller:
        controller:
            package: dynamixel_controllers
            module: joint_torque_controller
            type: JointTorqueController
        joint_name: femur_2
        joint_speed: 1.0
        motor:
            id: 16
            init: 512
            min: 200
            max: 830

    joint18_controller:
        controller:
            package: dynamixel_controllers
            module: joint_position_controller
            type: JointPositionController
        joint_name: tibia_2
        joint_speed: 1.0
        motor:
            id: 18
            init: 512
            min: 170
    		max: 700
    ```

- Création d'un ficher de configuration de tous les paramètres

		$ vim joints_trajectory_controller.yaml

	Écrire

    ```yaml
    f_arm_controller:
        controller:
            package: dynamixel_controllers
            module: joint_trajectory_action_controller
            type: JointTrajectoryActionController
        joint_trajectory_action_node:
            min_velocity: 0.1
            constraints:
                goal_time: 0.25
    ```

- Créer un fichier launch

		$ vim launch/start_meta_controller.launch

	Écrire

    ```xml
    <launch>
    <!-- Start tilt joint controller -->
        <rosparam file="$(find my_dynamixel_tutorial)/tilt2.yaml" command="load"/>
        <node name="controller_spawner" pkg="dynamixel_controllers" type="controller_spawner.py"
              args="--manager=dxl_manager
                    --port dxl_USB0
                    joint14_controller
                    joint16_controller
                    joint18_controller
                    "
              output="screen"/>

      <!-- Start joints trajectory controller controller -->
        <rosparam file="$(find my_dynamixel_tutorial)/joints_trajectory_controller.yaml" command="load"/>
        <node name="controller_spawner_meta" pkg="dynamixel_controllers" type="controller_spawner.py"
              args="--manager=dxl_manager
                    --type=meta
                    f_arm_controller
                    joint14_controller
                    joint16_controller
                    joint18_controller
                    "
               output="screen"/>
    </launch>
    ```

- Lancement du controleur créé précédemment

		$ roslaunch my_dynamixel_tutorial controller_manager.launch

- Lancement du fichier `launch` qui vient d'être créé (on récupère la main sur le terminal après, c'est normal)

		$ roslaunch my_dynamixel_tutorial start_meta_controller.launch

	Remarque : cela n'a pas marché pour moi, j'ai alors créé le fichier `launch/start_tilt2_manager.launch` :

    ```xml
    <?xml version='1.0' ?>
    <launch>
        <!-- Start tilt joint controller -->
        <rosparam file="$(find my_dynamixel_tutorial)/tilt2.yaml" command="load"/>
        <node name="tilt_controller_spawner" pkg="dynamixel_controllers" type="controller_spawner.py"
              args="--manager=dxl_manager
                    --port pan_tilt_port
                    joint14_controller
                    joint16_controller
                    joint18_controller"
              output="screen"/>
    </launch>
    ```

    Puis je l'ai lancé avec :

		$ roslaunch my_dynamixel_tutorial start_tilt2_controller.launch

    Et cela a fonctionné.

- On vérifie que cela a bien fonctionné avec

		$ rostopic list

	Qui doit alors renvoyer :

        /diagnostics
        /f_arm_controller/command
        /f_arm_controller/follow_joint_trajectory/cancel
        /f_arm_controller/follow_joint_trajectory/feedback
        /f_arm_controller/follow_joint_trajectory/goal
        /f_arm_controller/follow_joint_trajectory/result
        /f_arm_controller/follow_joint_trajectory/status
        /f_arm_controller/state
        /joint14_controller/command
        /joint14_controller/state
        /joint16_controller/command
        /joint16_controller/state
        /joint18_controller/command
        /joint18_controller/state
        /motor_states/pan_tilt_port
        /rosout
        /rosout_agg

### Création d'un client
Source : <http://wiki.ros.org/dynamixel_controllers/Tutorials/Creatingdynamixelactionclient>

On écrit le code Python suivant :

```Python
#!/usr/bin/env python
import roslib
roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Joint:
    def __init__(self, motor_name):
        # motor_name sera f_arm
        self.name = motor_name
        # Cree un client pour le topic
        self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')


    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()
        #char = self.name[0]
        #goal.trajectory.joint_names = ['joint_1'+char, 'joint_2'+char,'joint_3'+char]
        # On donne le nom des articulations
        # Conformement au fichier de config.yaml
        goal.trajectory.joint_names = ['joint_14', 'joint_16','joint_18']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

def main():
    arm = Joint('f_arm')
    arm.move_joint([0.5,1.5,1.0])
    arm.move_joint([6.28,3.14,6.28])

if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
```

### Services de dynamixel_controllers
Le paquet `dynamixel_controllers` propose également les services suivants :

    RestartController
    SetComplianceMargin
    SetCompliancePunch
    SetComplianceSlope
    SetSpeed
    SetTorqueLimit
    StartController
    StopController
    TorqueEnable

Donc pour désactiver le couple sur un servo :

    $ rosservice call /dynamixel_controller/j_c1_lf_position_controller/torque_enable False

### Commandes utiles
État actuel de tous les moteurs :

    rostopic echo /phantomx/motor_states/phantomx_dxl_port

Commande de position sur un moteur :

    rostopic pub -1 /phantomx/j_tibia_rm_position_controller/command std_msgs/Float64 -- -1.5

Connaître la valeur d'un paramètre :

    rosparam get /phantomx/j_tibia_rm_position_controller/joint_speed

Modifier la valeur d'un paramètre (souvent inutile car les paramètres sont généralement lus qu'au lancement du noeud) :

    rosparam set /phantomx/j_tibia_rm_position_controller/joint_speed 8.0

Appeler un service pour modifier la vitesse :

    rosservice call /phantomx/j_tibia_rm_position_controller/set_speed 4.0

### Commentaires sur le fichier de configuration
La configuration de chaque moteur est précisée dans un fichier YAML qui est chargé dans le serveur de paramètres avant le lancement du noeud `controller_spawner`. Dans ce fichier sont précisés :

- Le nom de la liaison (`joint_name`) ;
- La vitesse de la liaison (`joint_speed`) (aucun effet pour l'instant...) ;
- L'id du moteur (`id`) ;
- La position du zéro en ticks pour les commandes en radians (`init`) ;
- L'angle minimal admissible (bridage de sécurité) en ticks (`min`) ;
- L'angle maximal admissible (bridage de sécurité) en ticks (`max`).

*Remarque : si un servo est monté à l'envers, il suffit d'inverser les valeurs de `min` `max`.*

D'après l'étude du [code source](https://github.com/arebgun/dynamixel_motor/blob/master/dynamixel_controllers/src/dynamixel_controllers/joint_controller.py), il semble possible de renseigner les paramètres suivants dans le fichier de configuration :

- `joint_compliance_slope`
- `joint_compliance_margin`
- `joint_compliance_punch`
- `joint_torque_limit`

### Fork
Il existe un [fork](https://github.com/k-okada/dynamixel_motor) de `dynamixel_motor` de k-okada qui ajoute le JointStatePublisher dans le paquet et l'option `--dummy` qui permet de visualiser les changements dans RViz sans avoir les Dynamixels de physiquement connectés ([changements](https://github.com/k-okada/dynamixel_motor/commit/4f701becc7f2ebaa4a02873d5e9a460e0b46a482)).

### Application à la stack Hexapod de KevinO
#### Topics

    /base
    /body
    /cmd_vel
    /diagnostics
    /dynamixel_controller/coxa_joint_LF_position_controller/command
    /dynamixel_controller/coxa_joint_LF_position_controller/state
    /dynamixel_controller/coxa_joint_LM_position_controller/command
    /dynamixel_controller/coxa_joint_LM_position_controller/state
    /dynamixel_controller/coxa_joint_LR_position_controller/command
    /dynamixel_controller/coxa_joint_LR_position_controller/state
    /dynamixel_controller/coxa_joint_RF_position_controller/command
    /dynamixel_controller/coxa_joint_RF_position_controller/state
    /dynamixel_controller/coxa_joint_RM_position_controller/command
    /dynamixel_controller/coxa_joint_RM_position_controller/state
    /dynamixel_controller/coxa_joint_RR_position_controller/command
    /dynamixel_controller/coxa_joint_RR_position_controller/state
    /dynamixel_controller/femur_joint_LF_position_controller/command
    /dynamixel_controller/femur_joint_LM_position_controller/command
    /dynamixel_controller/femur_joint_LM_position_controller/state
    /dynamixel_controller/femur_joint_LR_position_controller/command
    /dynamixel_controller/femur_joint_LR_position_controller/state
    /dynamixel_controller/femur_joint_RF_position_controller/command
    /dynamixel_controller/femur_joint_RM_position_controller/command
    /dynamixel_controller/femur_joint_RR_position_controller/command
    /dynamixel_controller/femur_joint_RR_position_controller/state
    /dynamixel_controller/motor_states/phantomx_dxl_port
    /dynamixel_controller/tibia_joint_LF_position_controller/command
    /dynamixel_controller/tibia_joint_LM_position_controller/command
    /dynamixel_controller/tibia_joint_LR_position_controller/command
    /dynamixel_controller/tibia_joint_RF_position_controller/command
    /dynamixel_controller/tibia_joint_RM_position_controller/command
    /dynamixel_controller/tibia_joint_RR_position_controller/command
    /head
    /imu/data
    /imu_override
    /joint_states
    /rosout
    /rosout_agg
    /sounds
    /state
    /tf

#### Paramètres

    /ALL_FOOT_INIT_Z
    /COXA_LENGTH
    /CYCLE_LENGTH
    /FEMUR_LENGTH
    /FRONT_COXA_ANGLE
    /FRONT_COXA_TO_CENTER_X
    /FRONT_COXA_TO_CENTER_Y
    /FRONT_INIT_FOOT_POS_X
    /FRONT_INIT_FOOT_POS_Y
    /LEG_LIFT_HEIGHT
    /MID_COXA_ANGLE
    /MID_COXA_TO_CENTER_X
    /MID_COXA_TO_CENTER_Y
    /MID_INIT_FOOT_POS_X
    /MID_INIT_FOOT_POS_Y
    /OFFSET_ANGLE
    /REAR_COXA_ANGLE
    /REAR_COXA_TO_CENTER_X
    /REAR_COXA_TO_CENTER_Y
    /REAR_INIT_FOOT_POS_X
    /REAR_INIT_FOOT_POS_Y
    /TARSUS_LENGTH
    /TIBIA_LENGTH
    /dynamixel_controller/coxa_joint_LF_position_controller/controller/module
    /dynamixel_controller/coxa_joint_LF_position_controller/controller/package
    /dynamixel_controller/coxa_joint_LF_position_controller/controller/type
    /dynamixel_controller/coxa_joint_LF_position_controller/joint_name
    /dynamixel_controller/coxa_joint_LF_position_controller/joint_speed
    /dynamixel_controller/coxa_joint_LF_position_controller/motor/id
    /dynamixel_controller/coxa_joint_LF_position_controller/motor/init
    /dynamixel_controller/coxa_joint_LF_position_controller/motor/max
    /dynamixel_controller/coxa_joint_LF_position_controller/motor/min
    /dynamixel_controller/coxa_joint_LM_position_controller/controller/module
    /dynamixel_controller/coxa_joint_LM_position_controller/controller/package
    /dynamixel_controller/coxa_joint_LM_position_controller/controller/type
    /dynamixel_controller/coxa_joint_LM_position_controller/joint_name
    /dynamixel_controller/coxa_joint_LM_position_controller/joint_speed
    /dynamixel_controller/coxa_joint_LM_position_controller/motor/id
    /dynamixel_controller/coxa_joint_LM_position_controller/motor/init
    /dynamixel_controller/coxa_joint_LM_position_controller/motor/max
    /dynamixel_controller/coxa_joint_LM_position_controller/motor/min
    /dynamixel_controller/coxa_joint_LR_position_controller/controller/module
    /dynamixel_controller/coxa_joint_LR_position_controller/controller/package
    /dynamixel_controller/coxa_joint_LR_position_controller/controller/type
    /dynamixel_controller/coxa_joint_LR_position_controller/joint_name
    /dynamixel_controller/coxa_joint_LR_position_controller/joint_speed
    /dynamixel_controller/coxa_joint_LR_position_controller/motor/id
    /dynamixel_controller/coxa_joint_LR_position_controller/motor/init
    /dynamixel_controller/coxa_joint_LR_position_controller/motor/max
    /dynamixel_controller/coxa_joint_LR_position_controller/motor/min
    /dynamixel_controller/coxa_joint_RF_position_controller/controller/module
    /dynamixel_controller/coxa_joint_RF_position_controller/controller/package
    /dynamixel_controller/coxa_joint_RF_position_controller/controller/type
    /dynamixel_controller/coxa_joint_RF_position_controller/joint_name
    /dynamixel_controller/coxa_joint_RF_position_controller/joint_speed
    /dynamixel_controller/coxa_joint_RF_position_controller/motor/id
    /dynamixel_controller/coxa_joint_RF_position_controller/motor/init
    /dynamixel_controller/coxa_joint_RF_position_controller/motor/max
    /dynamixel_controller/coxa_joint_RF_position_controller/motor/min
    /dynamixel_controller/coxa_joint_RM_position_controller/controller/module
    /dynamixel_controller/coxa_joint_RM_position_controller/controller/package
    /dynamixel_controller/coxa_joint_RM_position_controller/controller/type
    /dynamixel_controller/coxa_joint_RM_position_controller/joint_name
    /dynamixel_controller/coxa_joint_RM_position_controller/joint_speed
    /dynamixel_controller/coxa_joint_RM_position_controller/motor/id
    /dynamixel_controller/coxa_joint_RM_position_controller/motor/init
    /dynamixel_controller/coxa_joint_RM_position_controller/motor/max
    /dynamixel_controller/coxa_joint_RM_position_controller/motor/min
    /dynamixel_controller/coxa_joint_RR_position_controller/controller/module
    /dynamixel_controller/coxa_joint_RR_position_controller/controller/package
    /dynamixel_controller/coxa_joint_RR_position_controller/controller/type
    /dynamixel_controller/coxa_joint_RR_position_controller/joint_name
    /dynamixel_controller/coxa_joint_RR_position_controller/joint_speed
    /dynamixel_controller/coxa_joint_RR_position_controller/motor/id
    /dynamixel_controller/coxa_joint_RR_position_controller/motor/init
    /dynamixel_controller/coxa_joint_RR_position_controller/motor/max
    /dynamixel_controller/coxa_joint_RR_position_controller/motor/min
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/degrees_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/encoder_resolution
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/encoder_ticks_per_degree
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/encoder_ticks_per_radian
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/max_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/max_torque
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/max_velocity
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/min_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/model_name
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/model_number
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/radians_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/radians_second_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/range_degrees
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/range_radians
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/torque_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/1/velocity_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/degrees_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/encoder_resolution
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/encoder_ticks_per_degree
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/encoder_ticks_per_radian
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/max_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/max_torque
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/max_velocity
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/min_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/model_name
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/model_number
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/radians_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/radians_second_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/range_degrees
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/range_radians
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/torque_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/2/velocity_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/degrees_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/encoder_resolution
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/encoder_ticks_per_degree
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/encoder_ticks_per_radian
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/max_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/max_torque
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/max_velocity
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/min_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/model_name
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/model_number
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/radians_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/radians_second_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/range_degrees
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/range_radians
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/torque_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/3/velocity_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/degrees_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/encoder_resolution
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/encoder_ticks_per_degree
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/encoder_ticks_per_radian
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/max_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/max_torque
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/max_velocity
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/min_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/model_name
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/model_number
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/radians_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/radians_second_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/range_degrees
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/range_radians
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/torque_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/4/velocity_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/degrees_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/encoder_resolution
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/encoder_ticks_per_degree
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/encoder_ticks_per_radian
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/max_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/max_torque
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/max_velocity
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/min_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/model_name
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/model_number
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/radians_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/radians_second_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/range_degrees
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/range_radians
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/torque_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/5/velocity_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/degrees_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/encoder_resolution
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/encoder_ticks_per_degree
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/encoder_ticks_per_radian
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/max_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/max_torque
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/max_velocity
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/min_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/model_name
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/model_number
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/radians_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/radians_second_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/range_degrees
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/range_radians
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/torque_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/6/velocity_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/degrees_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/encoder_resolution
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/encoder_ticks_per_degree
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/encoder_ticks_per_radian
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/max_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/max_torque
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/max_velocity
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/min_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/model_name
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/model_number
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/radians_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/radians_second_per_encoder_tick
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/range_degrees
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/range_radians
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/torque_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/7/velocity_per_volt
    /dynamixel_controller/dynamixel/phantomx_dxl_port/8/max_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/8/min_angle
    /dynamixel_controller/dynamixel/phantomx_dxl_port/8/model_name
    /dynamixel_controller/dynamixel/phantomx_dxl_port/8/model_number
    /dynamixel_controller/dynamixel_manager/namespace
    /dynamixel_controller/dynamixel_manager/serial_ports/phantomx_dxl_port/baud_rate
    /dynamixel_controller/dynamixel_manager/serial_ports/phantomx_dxl_port/max_motor_id
    /dynamixel_controller/dynamixel_manager/serial_ports/phantomx_dxl_port/min_motor_id
    /dynamixel_controller/dynamixel_manager/serial_ports/phantomx_dxl_port/port_name
    /dynamixel_controller/dynamixel_manager/serial_ports/phantomx_dxl_port/update_rate
    /dynamixel_controller/femur_joint_LF_position_controller/controller/module
    /dynamixel_controller/femur_joint_LF_position_controller/controller/package
    /dynamixel_controller/femur_joint_LF_position_controller/controller/type
    /dynamixel_controller/femur_joint_LF_position_controller/joint_name
    /dynamixel_controller/femur_joint_LF_position_controller/joint_speed
    /dynamixel_controller/femur_joint_LF_position_controller/motor/id
    /dynamixel_controller/femur_joint_LF_position_controller/motor/init
    /dynamixel_controller/femur_joint_LF_position_controller/motor/max
    /dynamixel_controller/femur_joint_LF_position_controller/motor/min
    /dynamixel_controller/femur_joint_LM_position_controller/controller/module
    /dynamixel_controller/femur_joint_LM_position_controller/controller/package
    /dynamixel_controller/femur_joint_LM_position_controller/controller/type
    /dynamixel_controller/femur_joint_LM_position_controller/joint_name
    /dynamixel_controller/femur_joint_LM_position_controller/joint_speed
    /dynamixel_controller/femur_joint_LM_position_controller/motor/id
    /dynamixel_controller/femur_joint_LM_position_controller/motor/init
    /dynamixel_controller/femur_joint_LM_position_controller/motor/max
    /dynamixel_controller/femur_joint_LM_position_controller/motor/min
    /dynamixel_controller/femur_joint_LR_position_controller/controller/module
    /dynamixel_controller/femur_joint_LR_position_controller/controller/package
    /dynamixel_controller/femur_joint_LR_position_controller/controller/type
    /dynamixel_controller/femur_joint_LR_position_controller/joint_name
    /dynamixel_controller/femur_joint_LR_position_controller/joint_speed
    /dynamixel_controller/femur_joint_LR_position_controller/motor/id
    /dynamixel_controller/femur_joint_LR_position_controller/motor/init
    /dynamixel_controller/femur_joint_LR_position_controller/motor/max
    /dynamixel_controller/femur_joint_LR_position_controller/motor/min
    /dynamixel_controller/femur_joint_RF_position_controller/controller/module
    /dynamixel_controller/femur_joint_RF_position_controller/controller/package
    /dynamixel_controller/femur_joint_RF_position_controller/controller/type
    /dynamixel_controller/femur_joint_RF_position_controller/joint_name
    /dynamixel_controller/femur_joint_RF_position_controller/joint_speed
    /dynamixel_controller/femur_joint_RF_position_controller/motor/id
    /dynamixel_controller/femur_joint_RF_position_controller/motor/init
    /dynamixel_controller/femur_joint_RF_position_controller/motor/max
    /dynamixel_controller/femur_joint_RF_position_controller/motor/min
    /dynamixel_controller/femur_joint_RM_position_controller/controller/module
    /dynamixel_controller/femur_joint_RM_position_controller/controller/package
    /dynamixel_controller/femur_joint_RM_position_controller/controller/type
    /dynamixel_controller/femur_joint_RM_position_controller/joint_name
    /dynamixel_controller/femur_joint_RM_position_controller/joint_speed
    /dynamixel_controller/femur_joint_RM_position_controller/motor/id
    /dynamixel_controller/femur_joint_RM_position_controller/motor/init
    /dynamixel_controller/femur_joint_RM_position_controller/motor/max
    /dynamixel_controller/femur_joint_RM_position_controller/motor/min
    /dynamixel_controller/femur_joint_RR_position_controller/controller/module
    /dynamixel_controller/femur_joint_RR_position_controller/controller/package
    /dynamixel_controller/femur_joint_RR_position_controller/controller/type
    /dynamixel_controller/femur_joint_RR_position_controller/joint_name
    /dynamixel_controller/femur_joint_RR_position_controller/joint_speed
    /dynamixel_controller/femur_joint_RR_position_controller/motor/id
    /dynamixel_controller/femur_joint_RR_position_controller/motor/init
    /dynamixel_controller/femur_joint_RR_position_controller/motor/max
    /dynamixel_controller/femur_joint_RR_position_controller/motor/min
    /dynamixel_controller/joint_state_controller/publish_rate
    /dynamixel_controller/joint_state_controller/type
    /dynamixel_controller/tibia_joint_LF_position_controller/controller/module
    /dynamixel_controller/tibia_joint_LF_position_controller/controller/package
    /dynamixel_controller/tibia_joint_LF_position_controller/controller/type
    /dynamixel_controller/tibia_joint_LF_position_controller/joint_name
    /dynamixel_controller/tibia_joint_LF_position_controller/joint_speed
    /dynamixel_controller/tibia_joint_LF_position_controller/motor/id
    /dynamixel_controller/tibia_joint_LF_position_controller/motor/init
    /dynamixel_controller/tibia_joint_LF_position_controller/motor/max
    /dynamixel_controller/tibia_joint_LF_position_controller/motor/min
    /dynamixel_controller/tibia_joint_LM_position_controller/controller/module
    /dynamixel_controller/tibia_joint_LM_position_controller/controller/package
    /dynamixel_controller/tibia_joint_LM_position_controller/controller/type
    /dynamixel_controller/tibia_joint_LM_position_controller/joint_name
    /dynamixel_controller/tibia_joint_LM_position_controller/joint_speed
    /dynamixel_controller/tibia_joint_LM_position_controller/motor/id
    /dynamixel_controller/tibia_joint_LM_position_controller/motor/init
    /dynamixel_controller/tibia_joint_LM_position_controller/motor/max
    /dynamixel_controller/tibia_joint_LM_position_controller/motor/min
    /dynamixel_controller/tibia_joint_LR_position_controller/controller/module
    /dynamixel_controller/tibia_joint_LR_position_controller/controller/package
    /dynamixel_controller/tibia_joint_LR_position_controller/controller/type
    /dynamixel_controller/tibia_joint_LR_position_controller/joint_name
    /dynamixel_controller/tibia_joint_LR_position_controller/joint_speed
    /dynamixel_controller/tibia_joint_LR_position_controller/motor/id
    /dynamixel_controller/tibia_joint_LR_position_controller/motor/init
    /dynamixel_controller/tibia_joint_LR_position_controller/motor/max
    /dynamixel_controller/tibia_joint_LR_position_controller/motor/min
    /dynamixel_controller/tibia_joint_RF_position_controller/controller/module
    /dynamixel_controller/tibia_joint_RF_position_controller/controller/package
    /dynamixel_controller/tibia_joint_RF_position_controller/controller/type
    /dynamixel_controller/tibia_joint_RF_position_controller/joint_name
    /dynamixel_controller/tibia_joint_RF_position_controller/joint_speed
    /dynamixel_controller/tibia_joint_RF_position_controller/motor/id
    /dynamixel_controller/tibia_joint_RF_position_controller/motor/init
    /dynamixel_controller/tibia_joint_RF_position_controller/motor/max
    /dynamixel_controller/tibia_joint_RF_position_controller/motor/min
    /dynamixel_controller/tibia_joint_RM_position_controller/controller/module
    /dynamixel_controller/tibia_joint_RM_position_controller/controller/package
    /dynamixel_controller/tibia_joint_RM_position_controller/controller/type
    /dynamixel_controller/tibia_joint_RM_position_controller/joint_name
    /dynamixel_controller/tibia_joint_RM_position_controller/joint_speed
    /dynamixel_controller/tibia_joint_RM_position_controller/motor/id
    /dynamixel_controller/tibia_joint_RM_position_controller/motor/init
    /dynamixel_controller/tibia_joint_RM_position_controller/motor/max
    /dynamixel_controller/tibia_joint_RM_position_controller/motor/min
    /dynamixel_controller/tibia_joint_RR_position_controller/controller/module
    /dynamixel_controller/tibia_joint_RR_position_controller/controller/package
    /dynamixel_controller/tibia_joint_RR_position_controller/controller/type
    /dynamixel_controller/tibia_joint_RR_position_controller/joint_name
    /dynamixel_controller/tibia_joint_RR_position_controller/joint_speed
    /dynamixel_controller/tibia_joint_RR_position_controller/motor/id
    /dynamixel_controller/tibia_joint_RR_position_controller/motor/init
    /dynamixel_controller/tibia_joint_RR_position_controller/motor/max
    /dynamixel_controller/tibia_joint_RR_position_controller/motor/min
    /robot_description
    /rosdistro
    /roslaunch/uris/host_192_168_0_102__39203
    /roslaunch/uris/host_192_168_0_103__43311
    /rosversion
    /run_id

#### Services

    /Hexapod_Locomotion/get_loggers
    /Hexapod_Locomotion/set_logger_level
    /dynamixel_controller/coxa_joint_LF_position_controller/set_compliance_margin
    /dynamixel_controller/coxa_joint_LF_position_controller/set_compliance_punch
    /dynamixel_controller/coxa_joint_LF_position_controller/set_compliance_slope
    /dynamixel_controller/coxa_joint_LF_position_controller/set_speed
    /dynamixel_controller/coxa_joint_LF_position_controller/set_torque_limit
    /dynamixel_controller/coxa_joint_LF_position_controller/torque_enable
    /dynamixel_controller/coxa_joint_LM_position_controller/set_compliance_margin
    /dynamixel_controller/coxa_joint_LM_position_controller/set_compliance_punch
    /dynamixel_controller/coxa_joint_LM_position_controller/set_compliance_slope
    /dynamixel_controller/coxa_joint_LM_position_controller/set_speed
    /dynamixel_controller/coxa_joint_LM_position_controller/set_torque_limit
    /dynamixel_controller/coxa_joint_LM_position_controller/torque_enable
    /dynamixel_controller/coxa_joint_LR_position_controller/set_compliance_margin
    /dynamixel_controller/coxa_joint_LR_position_controller/set_compliance_punch
    /dynamixel_controller/coxa_joint_LR_position_controller/set_compliance_slope
    /dynamixel_controller/coxa_joint_LR_position_controller/set_speed
    /dynamixel_controller/coxa_joint_LR_position_controller/set_torque_limit
    /dynamixel_controller/coxa_joint_LR_position_controller/torque_enable
    /dynamixel_controller/coxa_joint_RF_position_controller/set_compliance_margin
    /dynamixel_controller/coxa_joint_RF_position_controller/set_compliance_punch
    /dynamixel_controller/coxa_joint_RF_position_controller/set_compliance_slope
    /dynamixel_controller/coxa_joint_RF_position_controller/set_speed
    /dynamixel_controller/coxa_joint_RF_position_controller/set_torque_limit
    /dynamixel_controller/coxa_joint_RF_position_controller/torque_enable
    /dynamixel_controller/coxa_joint_RM_position_controller/set_compliance_margin
    /dynamixel_controller/coxa_joint_RM_position_controller/set_compliance_punch
    /dynamixel_controller/coxa_joint_RM_position_controller/set_compliance_slope
    /dynamixel_controller/coxa_joint_RM_position_controller/set_speed
    /dynamixel_controller/coxa_joint_RM_position_controller/set_torque_limit
    /dynamixel_controller/coxa_joint_RM_position_controller/torque_enable
    /dynamixel_controller/coxa_joint_RR_position_controller/set_compliance_margin
    /dynamixel_controller/coxa_joint_RR_position_controller/set_compliance_punch
    /dynamixel_controller/coxa_joint_RR_position_controller/set_compliance_slope
    /dynamixel_controller/coxa_joint_RR_position_controller/set_speed
    /dynamixel_controller/coxa_joint_RR_position_controller/set_torque_limit
    /dynamixel_controller/coxa_joint_RR_position_controller/torque_enable
    /dynamixel_controller/dxl_manager/meta/restart_controller
    /dynamixel_controller/dxl_manager/meta/start_controller
    /dynamixel_controller/dxl_manager/meta/stop_controller
    /dynamixel_controller/dxl_manager/phantomx_dxl_port/restart_controller
    /dynamixel_controller/dxl_manager/phantomx_dxl_port/start_controller
    /dynamixel_controller/dxl_manager/phantomx_dxl_port/stop_controller
    /dynamixel_controller/dynamixel_manager/get_loggers
    /dynamixel_controller/dynamixel_manager/set_logger_level
    /dynamixel_controller/femur_joint_LF_position_controller/set_compliance_margin
    /dynamixel_controller/femur_joint_LF_position_controller/set_compliance_punch
    /dynamixel_controller/femur_joint_LF_position_controller/set_compliance_slope
    /dynamixel_controller/femur_joint_LF_position_controller/set_speed
    /dynamixel_controller/femur_joint_LF_position_controller/set_torque_limit
    /dynamixel_controller/femur_joint_LF_position_controller/torque_enable
    /dynamixel_controller/femur_joint_LM_position_controller/set_compliance_margin
    /dynamixel_controller/femur_joint_LM_position_controller/set_compliance_punch
    /dynamixel_controller/femur_joint_LM_position_controller/set_compliance_slope
    /dynamixel_controller/femur_joint_LM_position_controller/set_speed
    /dynamixel_controller/femur_joint_LM_position_controller/set_torque_limit
    /dynamixel_controller/femur_joint_LM_position_controller/torque_enable
    /dynamixel_controller/femur_joint_LR_position_controller/set_compliance_margin
    /dynamixel_controller/femur_joint_LR_position_controller/set_compliance_punch
    /dynamixel_controller/femur_joint_LR_position_controller/set_compliance_slope
    /dynamixel_controller/femur_joint_LR_position_controller/set_speed
    /dynamixel_controller/femur_joint_LR_position_controller/set_torque_limit
    /dynamixel_controller/femur_joint_LR_position_controller/torque_enable
    /dynamixel_controller/femur_joint_RF_position_controller/set_compliance_margin
    /dynamixel_controller/femur_joint_RF_position_controller/set_compliance_punch
    /dynamixel_controller/femur_joint_RF_position_controller/set_compliance_slope
    /dynamixel_controller/femur_joint_RF_position_controller/set_speed
    /dynamixel_controller/femur_joint_RF_position_controller/set_torque_limit
    /dynamixel_controller/femur_joint_RF_position_controller/torque_enable
    /dynamixel_controller/femur_joint_RM_position_controller/set_compliance_margin
    /dynamixel_controller/femur_joint_RM_position_controller/set_compliance_punch
    /dynamixel_controller/femur_joint_RM_position_controller/set_compliance_slope
    /dynamixel_controller/femur_joint_RM_position_controller/set_speed
    /dynamixel_controller/femur_joint_RM_position_controller/set_torque_limit
    /dynamixel_controller/femur_joint_RM_position_controller/torque_enable
    /dynamixel_controller/femur_joint_RR_position_controller/set_compliance_margin
    /dynamixel_controller/femur_joint_RR_position_controller/set_compliance_punch
    /dynamixel_controller/femur_joint_RR_position_controller/set_compliance_slope
    /dynamixel_controller/femur_joint_RR_position_controller/set_speed
    /dynamixel_controller/femur_joint_RR_position_controller/set_torque_limit
    /dynamixel_controller/femur_joint_RR_position_controller/torque_enable
    /dynamixel_controller/tibia_joint_LF_position_controller/set_compliance_margin
    /dynamixel_controller/tibia_joint_LF_position_controller/set_compliance_punch
    /dynamixel_controller/tibia_joint_LF_position_controller/set_compliance_slope
    /dynamixel_controller/tibia_joint_LF_position_controller/set_speed
    /dynamixel_controller/tibia_joint_LF_position_controller/set_torque_limit
    /dynamixel_controller/tibia_joint_LF_position_controller/torque_enable
    /dynamixel_controller/tibia_joint_LM_position_controller/set_compliance_margin
    /dynamixel_controller/tibia_joint_LM_position_controller/set_compliance_punch
    /dynamixel_controller/tibia_joint_LM_position_controller/set_compliance_slope
    /dynamixel_controller/tibia_joint_LM_position_controller/set_speed
    /dynamixel_controller/tibia_joint_LM_position_controller/set_torque_limit
    /dynamixel_controller/tibia_joint_LM_position_controller/torque_enable
    /dynamixel_controller/tibia_joint_LR_position_controller/set_compliance_margin
    /dynamixel_controller/tibia_joint_LR_position_controller/set_compliance_punch
    /dynamixel_controller/tibia_joint_LR_position_controller/set_compliance_slope
    /dynamixel_controller/tibia_joint_LR_position_controller/set_speed
    /dynamixel_controller/tibia_joint_LR_position_controller/set_torque_limit
    /dynamixel_controller/tibia_joint_LR_position_controller/torque_enable
    /dynamixel_controller/tibia_joint_RF_position_controller/set_compliance_margin
    /dynamixel_controller/tibia_joint_RF_position_controller/set_compliance_punch
    /dynamixel_controller/tibia_joint_RF_position_controller/set_compliance_slope
    /dynamixel_controller/tibia_joint_RF_position_controller/set_speed
    /dynamixel_controller/tibia_joint_RF_position_controller/set_torque_limit
    /dynamixel_controller/tibia_joint_RF_position_controller/torque_enable
    /dynamixel_controller/tibia_joint_RM_position_controller/set_compliance_margin
    /dynamixel_controller/tibia_joint_RM_position_controller/set_compliance_punch
    /dynamixel_controller/tibia_joint_RM_position_controller/set_compliance_slope
    /dynamixel_controller/tibia_joint_RM_position_controller/set_speed
    /dynamixel_controller/tibia_joint_RM_position_controller/set_torque_limit
    /dynamixel_controller/tibia_joint_RM_position_controller/torque_enable
    /dynamixel_controller/tibia_joint_RR_position_controller/set_compliance_margin
    /dynamixel_controller/tibia_joint_RR_position_controller/set_compliance_punch
    /dynamixel_controller/tibia_joint_RR_position_controller/set_compliance_slope
    /dynamixel_controller/tibia_joint_RR_position_controller/set_speed
    /dynamixel_controller/tibia_joint_RR_position_controller/set_torque_limit
    /dynamixel_controller/tibia_joint_RR_position_controller/torque_enable
    /robot_state_publisher/get_loggers
    /robot_state_publisher/set_logger_level
    /rosout/get_loggers
    /rosout/set_logger_level

### Remarques
#### Réflexion sur les temps
Après chaque `write`, il y a un délai de 1,3 ms pour attendre la réponse du servo (`dynamixel_io.py` l. 177). En réalité, d'après la [datasheet de l'AX-18A](http://support.robotis.com/en/product/dynamixel/ax_series/ax-18f.htm), la valeur par défaut est la maximale et elle correspond à 0.5 ms.

Chaque paquet de commande de position comprend : `OxFF` (1), `OxFF` (1), `servo_id` (1), `length` (1), `WRITE_DATA` (1), `address`(1), `data` (2), `checksum` (1) = 9 octets soit (9 * 8 ) 72 bits. À 1000000 bauds, soit 1000000 bits par seconde en comptant les bits de contrôle (les bits de start et stop sont inclus), il faut au moins 0,072 ms soit 72 µs pour transmettre une instruction.

Le pacquet de retour de status contient au moins : `OxFF` (1), `OxFF` (1), `servo_id` (1), `length` (1), `error` (1), `checksum` (1) = 6 octets soit (6 * 8) 48 bits. Il faut donc 48 µs pour les transmettre.

Pour un envoi de position sur un servo on a alors 0,072 + 1,3 + 0,048 = 1,42 ms.

Pour 18 servomoteurs, on a alors (18 * 1,42) 25,56 ms, soit une fréquence maximale de 39,12 Hz en négligeant tous les temps de traitement.

Pour une commande de position avec l'instruction `sync_write` sur 18 servos : `OxFF` (1), `OxFF` (1), `servo_id` (1), `length` (1), `SYNC_WRITE` (1), `address`(1), `data` (3*18), `checksum` (1) = 61 octets soit 488 bits de donnée + 61 * . Il faut alors 488 µs à 1000000 bauds. Mais il n'y a pas de réponse pour un ordre de broadcast. On peut alors monter à une fréquence max de 2049 Hz !

#### Lecture de l'état
- Lecture de 17 registres pour le feedback : `OxFF` (1), `OxFF` (1), `servo_id` (1), `length` (1), `READ_DATA` (1), `address`(1), `data` (1), `checksum` (1) = 8 octets soit 64 bits. 64 µs pour l'envoi.
- Attente de 1,3 ms le temps de recevoir la réponse :
- Réponse : `OxFF` (1), `OxFF` (1), `servo_id` (1), `length` (1), `error` (1), `parameter` (17) , `checksum` (1) = 23 octets soit 184 bits. Il faut donc 184 µs pour l'envoi.

Cela fait don un total de 0,016 + 1,3 + 0,184 = 1,5 ms.

Pour 18 moteurs : (1,5 * 18) 27 ms. Soit une fréquence max de 37 Hz.

Sources :

- Document sur la communication UART : <https://www.freebsd.org/doc/en_US.ISO8859-1/articles/serial-uart/>
- Datasheet du AX-18A : <http://support.robotis.com/en/product/dynamixel/ax_series/ax-18f.htm>
- Documentation sur la communication Dynamixel : <http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm> et <http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm>

#### Découvertes dans le code source
La méthode `sync_write` (`dynamixel_io.py` l. 185) est implémentée et utilisée dans toutes les méthodes du type `set_multi`.

Les valeurs de conversion en système SI sont notées dans le fichier `dynamixel_const.py`. Pour l'AX 18A :

    18: { 'name':               'AX-18',
           'encoder_resolution': 1024,
           'range_degrees':      300.0,
           'torque_per_volt':    1.8 / 12.0,                       #  1.8 NM @ 12V
           'velocity_per_volt':  (97 * RPM_TO_RADSEC) / 12.0,      #  97 RPM @ 12V
           'rpm_per_tick':       0.111,
           'features':           []
         },

La lecture des capteurs pour la publication des messages `/state` se fait via la méthode `get_feedback` du fichier `dynamixel_io.py`. Elle lit les 17 octets consécutifs de statut du dynamixel via la méthode `read` du même fichier. Elle est appelée dans une boucle sur le nombre de moteurs par la méthode `__update_motor_states` du fichier `dynamixel_serial_proxy.py`. Cette méthode est lancée dans un thread lors du lancement du Dynamixel manager, et tourne avec la période indiquée par le paramètre `update_rate`.

La méthode `read`, comme `write`, attend 1,3 ms pour recevoir la réponse. Attention, la méthode `read` renvoie toute la trame reçue sous la forme d'une liste et lui ajoute un timestamp à la fin.

#### Mofification du code pour augmenter la vitesse

## Utilisation du paquet ROS de HR pour les Dynamixels
Dans cette partie, nous utilisons le paquet ROS développé par HumaRobotics, disponible à l'adresse suivente : <https://github.com/HumaRobotics/dynamixel_hr_ros>. Pour fonctionner, il a besoin que la bibliothèque Python de HR pour le pilotage des Dynamixels soit installée. On peut la trouver à cette adresse : <https://github.com/HumaRobotics/dynamixel_hr>.

### Installation
Il suffit de télécharger les fichiers et de mettre le dossier dans le dossier `src` du workspace ROS.

		$ cd ~/catkin_ws/src
		$ git clone https://github.com/HumaRobotics/dynamixel_hr_ros.git

Il faut ensuite lancer les commandes `rosmake` et `catkin_make` pour installer le paquet :

		$ cd dynamixel_hr_ros
		$ rosmake
		$ cd ../../
		$ catkin_make

### Utilisation
Pour lancer le noeud `/dxl`, il suffit de lancer la commande suivante (avec une instance de `roscore` déjà en cours)

		$ rosrun dynamixel_hr_ros expose.py --device /dev/ttyUSB0 --baudrate 1000000 --rate 10

L'argument `--rate 10` donne la fréquence de publication des messages.

Deux scripts sont fournis en exemple : `record.py` et `replay.py`. Comme leurs noms l'indiquent, ils servent à enregistrer une séquence de mouvement des moteurs sur 5 secondes en les enregistrant dans un ficher dans le répertoire courant, puis les rejouer (attention, les vitesses ne sont pas préservées).

Enregistrer un mouvement de 5 secondes :

		$ rosrun dynamixel_hr_ros record.py

Rejouer le mouvement :

		$ rosrun dynamixel_hr_ros replay.py

### Activation/Désactivation de la chaîne
Pour désactiver la chaîne de Dynamixel, il faut envoyer un message avec un booléen à faux sur le topic `/dxl/enable` :

		$ rostopic pub -1 /dxl/enable std_msgs/Bool -- 'false'

De même pour l'activation :

		$ rostopic pub -1 /dxl/enable std_msgs/Bool -- 'true'

### Description de la bibliothèque
Description donnée dans le `README` du paquet :

> The ROS bindings are provided by a node called /dxl. The three following topics are provided:
> * `/dxl/chain_state`: provides the current position of the motors
> * `/dxl/command_position`: listens for position (and optional speed) commands
> * `/dxl/enable`: listens for activation/deactivation messages
>
> The `/dxl/chain_state` topic will provide messages about the current position of the motors in radians. The message structure is the following:
>
> 		int8[] id
> 		float32[] angle
>
> The `/dxl/enable` topic listens to Bool messages that allows to activate (True) or deactivate (False) the entire motor chain.
>
> The `/dxl/command_position` topic listens for messages that specifies angles to be reached by the motors and optionally a set of speeds::
>
> 		int8[] id
> 		float32[] angle
> 		float32[] speed
>
> If no speed is provided then the last speed settings are used. Otherwise all speeds will be set at the same time as the goal position. To maximize performance,  synchronized write commands are used on the serial link.

*Remarque* : Les angles valent 0 radian pour 0 tick.

## Utilisation de Joystick avec ROS
Source : <http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick>

Dans Virtualbox, le joystick (manette de PS3 USB) est déjà bien reconnue puisque Windows l'a déjà reconnu comme un joystick USB.

Donner le numéro du joystick sur le serveur de paramètre de ROS :

		$ rosparam set joy_node/dev "/dev/input/js2"

Lancer le noeud `joy_node` :

		$ rosrun joy joy_node

Le réglage peut être ajouté dans un fichier launch :

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="joy" type="joy_node" name="joystick">
      <param name="dev" value="/dev/input/js2"/>
  </node>
</launch>
```

Utilisation du code du turtlebot disponible [ici](https://github.com/turtlebot/turtlebot/blob/indigo/turtlebot_teleop/src/turtlebot_joy.cpp).

## Piloter l'hexapode avec un joystick
Les paquets du **PhantomX** de [HumaRobotics](https://github.com/HumaRobotics) permettent de simuler le robot PhantomX hexapod de Trossen Robotics. Une marche de type tripod est implémentée avec un code d'exemple. Le robot sosucrit au topic `/phantomx/cmd_vel` qui reçoit des messages de type `geometry_msgs/Twist`. Il est alors possible d'utiliser le noeud de téléopération du paquet `turtlebot_teleop` qui publie ce type de message et gère deux types d'entrées, le clavier et un jostick. L'avantage du joystick, en plus d'être intuitif, c'est qu'il ne nécessite pas de conserver le focus sur le terminal du noeud de téléopération.

Dans la machine virtuelle, le joystick de PS3 est reconnu comme `/dev/input/js2`, il faut donc donner ce paramètre au noeud `joy_node`. Pour cela, on utilise un fichier `launch` :

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="joy" type="joy_node" name="joystick">
      <param name="dev" value="/dev/input/js2"/>
  </node>
</launch>
```

À ce fichier `launch`, on ajoute l'appel au noeud de téléopération au joystick du turtlebot, on lui passe des arguments pour donner des dacteurs sur les axes et le numéro des axes dépendants du joystick utilisé. Enfin, on remappe le topic. En effet, dans le fichier source `turtlebot_teleop_joy.cpp`, le `nodeHandle` est construit avec le paramière `'~'` qui correspond au namespace privé. Dans ce cas, on a donné le nom `phantomx_teleop` au noeud de gestion de téléopération donc le topic publié est nommé : `phantomx_teleop/cmd_vel`, or le robot simulé de HR attend un topic nommé `phantomx/cmd_vel`, il faut donc le remappé :

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="turtlebot_teleop" type="turtlebot_teleop_joy" name="phantomx_teleop">
    <param name="scale_angular" value="1.5"/>
    <param name="scale_linear" value="1.5"/>
    <param name="axis_deadman" value="2"/>
    <param name="axis_linear" value="1"/>
    <param name="axis_angular" value="0"/>
    <remap from="phantomx_teleop/cmd_vel" to="phantomx/cmd_vel"/>
  </node>

  <node pkg="joy" type="joy_node" name="joystick">
      <param name="dev" value="/dev/input/js2"/>
  </node>

</launch>
```

## Gazebo headless
Le simulateur Gazebo est en fait décomposé en deux programmes distincts, `gzserver` et `gzclient`, respectivement le serveur de simulation et l'interface graphique de visualisation. Il est possible de ne lancer que le serveur pour limiter la puissance graphique nécessaire, les mouvements pouvant alors être visualisés via RViz.

Pour ne lancer que le serveur, il faut modifier le fichier `launch` pour passer l' argument `headless` à `true` et `gui` à `false`.
On part donc de l'extrait du `launch` suivant :

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/empty.world"/>
    <arg name="paused" value="true"/>
</include>
```

Et on ajoute deux lignes pour obtenir :

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/empty.world"/>
    <arg name="paused" value="true"/>
    <arg name="headless" value="true"/>
    <arg name="gui" value="false"/>
</include>
```

Le problème c'est que le simulateur est en pause et qu'il faut attendre que les contrôlleurs soient actifs pour lancer la simulation. Pour l'instant, je n'ai pas trouvé mieux que de lancer le client avec la commande :

    $ gzclient

De cliquer sur le bouton lecture, puis de quitter le client, le serveur restant alors actif.

Pour visualiser le robot, il faut lancer RViz :

    $ rosrun rviz rviz

Puis il faut définir la `base_link` comme **Fixed Frame** car il n'y a pas de frame `map`. Ensuite, il faut ajouter un *display* pour **Robot Model** et **tf**.

Remarque, dans RViz, vu qu'il n'y a pas de frame `map` de définie, le robot est immobile par rapport au sol, RViz n'est qu'un outil de visualisation.

## Hexapodes sous ROS
### Crab
#### Installation
Problème avec orocos_kdl et catkin : <http://answers.ros.org/question/107346/could-not-find-a-configuration-file-for-package/>

Ajouter la ligne

		find_package(orocos_kdl REQUIRED)

Dans le fichier CMakeList.txt des paquets posant problème (`crab_gait`, `crab_leg_kinematics`, `crab_body_kinematics`).

Pour que `crab_msgs` soit déjà compilé au moment de compiler les paquets utilisant ces messages, d'abord compiler que ce paquet :

		catkin_make --pkg crab_msgs
		catkin_make

#### Téléopération avec Joystick PS3
Source : <http://gameimps.com/ps3-controller-linux-usb-290>

- Pour configurer le Joystick, installer `xboxdrv` :

		$ sudo apt-get install xboxdrv

- Bancher le Joystick de PS3 en USB
- Appuyer sur le bouton PS pour l'activer (les LED continuent de clignoter)
- Lancer `xboxdrv` :

		$ sudo xboxdrv --led 2

	* S'il y a une erreur du type `IBUSB_ERROR_ACCESS`,  ajouter `--detach-kernel-drive` à la commande.
	* `--led 2` permet de mettre la led 1 fixe
	* Lorqu'on est sûr que ça marche, on peut utiliser `--silent` pour éviter tout l'affichage verbeux dans le terminal

### Rhoeby
- Wiki : <http://wiki.ros.org/Robots/Rhoeby>
- Sources : <https://github.com/Rhoeby/hexapod_ros>

### Erle-Spider
Annoncée en août 2015 : <http://erlerobotics.com/spider/>

## ROS sur le réseau
Pour faire communiquer deux PC entre-eux avec ROS, il suffit de renseigner deux variables d'environnement. Le plus simple est de les placer dans le fichier `~/.bashrc`.

    export ROS_HOSTNAME=adresse_ip_du_pc
    export ROS_MASTER_URI=http://adresse_ip_pc_master:11311

Attention à bien ajouter le `http://` et le port `:11311` pour le master.

Source : <http://wiki.ros.org/ROS/NetworkSetup>

## Notes générales sur ROS
Source : Le livre [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/)

### Changer le nom d'un noeud
Lors du lancement d'un noeud avec `rosrun`, il est possible de spécifier le nom du noeud :

		$ rosrun package-name executable-name __name:=node-name

Cette possibilité est intéressante puisque qu'avec ROS, chaque noeud doit avoir un nom unique. Si on a besoin de le faire à chaque fois, il vaut mieux utiliser un fichier `launch`.

### Lancer un noeud avec un namespace
Pour lancer manuellement (en dehors d'un fichier launch) un noeud avec un namespace.

    $ export ROS_NAMESPACE=my_namespace
    $ rosrun my_package my_node

Source : <http://answers.ros.org/question/28090/namespace-renaming/>

### Informations sur un noeud

		$ rosnode info node-name

### Tuer un noeud
On peut utiliser la commande `Ctrl + c` dans le terminal exécutant le noeud. Cependant, le Master peut toujours voir ce noeud. Pour supprimer les noeuds morts du master :

		$ rosnode cleanup

Une autre façon de tuer un noeud est la commande suivante :

		$ rosnode kill node-name

### Fréquence et bande passante d'un topic
La fréquence de publication d'un topic (messages par seconde) peut être affichée avec la commande :

		$ rostopic hz topic-name

De même, la bande passante utilisée (en octets par seconde) est calculée avec la commande :

		$ rostopic bw topic-name

### Publication manuelle de message
Il est possible de publier manuellement un message avec la commande suivante :

		$ rostopic pub -r rate-in-hz topic-name message-type message-content

Exemple :

		$ rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist ’[2, 0, 0]’ ’[0, 0, 0]’

Une autre solution est d'utiliser le formalisme YAML. Cependant, celui-ci demande des sauts de ligne dans la commande à écrire dans le terminal. Pour cela, il faut utiliser la commande de complétion avec la touche `Tab`.
Pour cela, écrire :

		$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "<Tab>

Et la ligne suivante va apparaître :

		$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
		  x: 0.0
		  y: 0.0
		  z: 0.0
		angular:
		  x: 0.0
		  y: 0.0
		  z: 0.0"

Il suffit alors de l'éditer avant de la valider.

### Vérification de problèmes
La commande `roswtf` effectue tout un tas de tests pour vérifier la présence d'éventuels problèmes.

### Inclure un fichier launch dans un fichier launch
Source : <http://wiki.ros.org/roslaunch/XML/include>

Il suffit d'utiliser la balise `<include>` et le lui spécifier le chemin.

```xml
<include file="$(find pkg-name)/launch/filename.launch" />
```

### Convertir un bag en fichiers CSV
Clearpath Robotics fournis un [script Python](https://support.clearpathrobotics.com/hc/en-us/articles/201644379-Converting-ROS-bag-to-CSV) permettant d'extraire les topics d'un bag en fichiers CSV, un par topic.

## Inclure la bibliothèque Eigen3
Pour utiliser la bibliothèque C++ de calcul matriciel [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) dans ROS, il faut l'indiquer dans le fichier `CMakeLists.txt` et ajouter la dépendance à `cmake_modules`.

Dans le `CMakeLists.txt` :

    find_package(Eigen3)
    if(NOT EIGEN3_FOUND)
      # Fallback to cmake_modules
      find_package(cmake_modules REQUIRED)
      find_package(Eigen REQUIRED)
      set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
      set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
      # Possibly map additional variables to the EIGEN3_ prefix.
    endif()

Dans `package.xml` :

    <build_depend>cmake_modules</build_depend>

Sources :
- <http://wiki.ros.org/jade/Migration>
- <https://github.com/ros/cmake_modules/blob/0.3-devel/README.md#usage>

## AccelStamped pcDuino
Attention, le package `hexapod_locomotion` utilise le message `geometry_msgs/AccelStamped` qui a été introduit dans ROS Indigo. Il ne peut donc pas s’exécuter sur le pcDuino tournant avec ROS Hydro.

Un message personnalisé `hexapod_msgs/AccelStamped` a donc été créé.

## Profiling ROS
Pour faire du profiling avec ROS, il est possible d'ajouter un préfix au lancement d'un noeud dans le fichier launch.

Par exemple avec Python, on écrirait `python -m cProfile -o profile.txt mon_script`. Ici, il suffit alors d'ajouter l'attribut `launch-prefix="python -m cProfile -o profile.txt"`.

Sources :

- Ajouter des prefix aux fichiers launch : <http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB>
- Utiliser `cProfile` pour les scripts Python : <http://stackoverflow.com/questions/582336/how-can-you-profile-a-python-script>

Pour analyser le fichier de sortie de `cProfile` : [runsnake](http://www.vrplumber.com/programming/runsnakerun/)

    $ sudo easy_install RunSnakeRun

## Debug ROS C++
Avec Gdb et Qtcreator

- Ajouter le flag de Debug au CMakeLists.txt de tout le workspace :

    set(CMAKE_BUILD_TYPE Debug)

- Lancer QtCreator
- Menu Debug/Start Debugging/Attach to Running Application...

Source : <http://answers.ros.org/question/34966/debugging-ros-package-with-qtcreator/>

Lors de la première utilisation, un message d'erreur va apparaître. Afin de pouvoir faire du débug *à la volée*, entrer la commande suivante :

    $ echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope

Pour rendre se réglage permanent, éditer le fichier `/etc/sysctl.d/10-ptrace.conf` et remplacer la ligne :

    kernel.yama.ptrace_scope = 1

Par :

    kernel.yama.ptrace_scope = 0

Source : <http://askubuntu.com/questions/41629/after-upgrade-gdb-wont-attach-to-process>

## Activer les warnings de GCC
Ajouter les flags : `-W -Wall` à la ligne :

        CMAKE_CXX_FLAGS:STRING=

Dans le fichier `build/CMakeCache.txt`.

## Couleurs sortie de catkin_make
Catkin est basé sur CMake qui crée des Makefile afin d'appeler `g++` pour la compilation. Avec l'utilisation des flags `-W` et `-Wall`, la sortie de `g++` peut être chargée à cause de paramètres non utilisés dans la bibliothèque `tf` ou encore avec l'utilisation de messages de types `Empty` qui par définition, ne sont pas utilisés.

Depuis la version 4.9 de GCC, il est possible de colorer la sortie en passant le flag `-fdiagnostics-color=auto` lors de la compilation. On ajoute donc ce flag dans le fichier `catkin_ws/build/CMakeCache.txt` à la ligne commençant par `CMAKE_CXX_FLAGS:STRING=` qui devient alors :

  CMAKE_CXX_FLAGS:STRING=-W -Wall -fdiagnostics-color=auto

Mais avec Ubuntu 14.04, c'est la version 4.8 de GCC qui est installée. Pour installer la version 4.9, il faut utiliser le dépôt des développeurs Ubuntu.

    $ sudo apt-get install build-essential
    $ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    $ sudo apt-get update
    $ sudo apt-get install gcc-4.9 g++-4.9 cpp-4.9

Ensuite, pour indiquer au système d'utiliser cette version au lieu de la 4.8, enter :

    $ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 40
    $ sudo update-alternatives --config gcc
    $ sudo update-alternatives --install /usr/bin/cpp cpp /usr/bin/cpp-4.9 40
    $ sudo update-alternatives --config cpp
    $ sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 40
    $ sudo update-alternatives --config g++

Source : <http://askubuntu.com/questions/428198/getting-installing-gcc-g-4-9-on-ubuntu>

## Erreur Thread Python

    Exception in thread Thread-34:
    Traceback (most recent call last):
      File "/usr/lib/python2.7/threading.py", line 551, in __bootstrap_inner
        self.run()
      File "/usr/lib/python2.7/threading.py", line 504, in run
        self.__target(*self.__args, **self.__kwargs)
      File "/opt/ros/hydro/lib/python2.7/dist-packages/rospy/impl/tcpros_pubsub.py", line 431, in _run
        with self._cond:
      File "/opt/ros/hydro/lib/python2.7/dist-packages/rospy/impl/tcpros_pubsub.py", line 390, in __getattr__
        return getattr(self._connection, name)
    AttributeError: 'TCPROSTransport' object has no attribute '_cond'

Cette erreur apparaît suite à un bug dans ROS Hydro et Indigo (voir [cet Issue sur le Github de ros-comm](https://github.com/ros/ros_comm/issues/369). Il semble avoir été corrigé dans la version 1.10.2 de ROS Hydro et 1.11.0 dans ROS Indigo. Selon le fichier `Packages` du dépôt ROS Hydro pour ubuntu ARM 12.04 (Precise) ([ici](http://packages.namniart.com/repos/ros/dists/precise/main/binary-armhf/Packages)) et le fichier `/opt/ros/hydro/share/ros_comm/package.xml` sur le pcDuino, c'est la version `1.9.53-0precise-20141104-0927-+0000` (version publiée le 15 janvier 2014) qui est installée, le bug n'est donc pas corrigée.

Selon [ce commit](https://github.com/ros/ros_comm/commit/60cdce9e4193ce5e2a4b5bbdda405641f676faf7), il suffit de changer la ligne 431 du fichier `/opt/ros/hydro/lib/python2.7/dist-packages/rospy/impl/tcpros_pubsub.py` :

    with self._cond:

Par :

    with self._lock:

## Erreur bad_alloc
Bizarrement, le fait de stocker le type de gait utilisé dans une string C++ (`std::string`), créait des erreurs de type `std::bad_alloc` qui intervenaient dans le getteur d'après la sortie de GBD :

		terminate called after throwing an instance of 'std::bad_alloc'
		  what():  std::bad_alloc

		Program received signal SIGABRT, Aborted.
		0x00007ffff5cf7cc9 in __GI_raise (sig=sig@entry=6) at ../nptl/sysdeps/unix/sysv/linux/raise.c:56
		56	../nptl/sysdeps/unix/sysv/linux/raise.c: No such file or directory.
		(gdb) backtrace
		#0  0x00007ffff5cf7cc9 in __GI_raise (sig=sig@entry=6) at ../nptl/sysdeps/unix/sysv/linux/raise.c:56
		#1  0x00007ffff5cfb0d8 in __GI_abort () at abort.c:89
		#2  0x00007ffff632c78d in __gnu_cxx::__verbose_terminate_handler() () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
		#3  0x00007ffff632a7f6 in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
		#4  0x00007ffff632a841 in std::terminate() () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
		#5  0x00007ffff632aa58 in __cxa_throw () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
		#6  0x00007ffff632af5c in operator new(unsigned long) () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
		#7  0x00007ffff6369f69 in std::string::_Rep::_S_create(unsigned long, unsigned long, std::allocator<char> const&) ()
		   from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
		#8  0x00007ffff636ad0b in std::string::_Rep::_M_clone(std::allocator<char> const&, unsigned long) ()
		   from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
		#9  0x00007ffff636b3ec in std::basic_string<char, std::char_traits<char>, std::allocator<char> >::basic_string(std::string const&) () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
		#10 0x00007ffff6da8c78 in Control::getGait (this=0x7fffffffc160)
		    at /home/rr244992/hexapod_ws/src/hexapod_locomotion/src/control.cpp:121
		#11 0x0000000000409cfe in main (argc=1, argv=0x7fffffffd348)
		    at /home/rr244992/hexapod_ws/src/hexapod_locomotion/src/locomotion.cpp:128

Pour corriger cela, le gait est maintenant stocké dans un `int` dont la valeur est donnée par l'énumération `Control::Gait::TRIPOD/WAVE`. Le programme modifié n'a pas été beacoup testé donc à voir si cela empêche bien le phénomène, sinon, modifier le type de message employé par ROS et revenir sur un type plus simple comme un `uint8_t`.

# Gazebo
## Unité
Gazebo suit les recommandation deu [REP 103](http://www.ros.org/reps/rep-0103.html) pour les unités donc les distances sont exprimées en mètre et les masses en kilogramme.

## Adaptation de l'URDF
Gazebo n'utilise pas directement le format URDF qui manque de fonctionnalités, mais le SDF. Cependant, il convertit lui même le fichier URDF de notre robot en SDF. Pour cela, il lui faut obligatoirement un élément `<inertial>` pour chaque `<link>`. Il est également possible (optionnel) d'ajouter un élément `<gazebo>` pour les `<link>`, `<joint>` et `<robot>`.

Source : <http://gazebosim.org/tutorials/?tut=ros_urdf>

## Erreur au lancement
Sur ROS Indigo, avec Gazebo 2, l'erreur suivante peut apparaître au démarrage :

    Error [Param.cc:181] Unable to set value [1,0471975511965976] for key[horizontal_fov]
    Error [Param.cc:181] Unable to set value [0,100000001] for key[near]

Pour éviter cela, il suffit d'ajouter la ligne suivante au `~/.bashrc` :

    export LC_NUMERIC=C

Puis de sourcer le `.bashrc` avant de relancer Gazebo :

    $ source ~/.bashrc

Source : <http://answers.gazebosim.org/question/5518/i-have-installed-gazebo-1-9-and-there-is-some/>

# pcDuino
## Ré-installer Ubuntu sur pcDuino3
Attention, le pcDuino3 et pcDuino3B n'utilisent pas les mêmes images puisque le pcDuino3B (connecteur Ethernet plus gros et condensateur entre la prise Ethernet et la prise HMDI) utilise une autre puce Ethernet pour offrir du Gigabit, il s'agit donc des mêmes images que le pcDuino3 Nano. Les images du pcDuino3 fonctionnent donc à l'exception du port Ethernet.

### Ubuntu 12
L'installation de Lubuntu 12.04 se fait en deux étapes, l'installation du noyau puis l'installation d'Ubuntu.

Pour installer le noyau :

- Télécharger l'image sur la page suivante : <http://www.linksprite.com/?page_id=855>. Il faut choisir l'image pouvant être utilisée avec l'utilitaire `dd` soit la version du 21/07/2014 au moment d'écrire ces lignes. Il faut bien prendre le *Kernel* mais sans le support du LVDC screen qui est l'écran tactile. Le lien est [celui-ci](https://s3.amazonaws.com/pcduino/Images/v3/20140721/pcduino3_a20_kernel_dd_20140721.img).
- Insérer la carte microSD dans le PC et repérer son nom dans `/dev` (`ls /dev` avant et après l'insertion de la carte). Elle devrait être soit `/dev/sdb` ou `/dev/mmcblk0p`.
- Démonter les partitions de la carte SD. Soit avec le logo *Eject* dans un gestionnaire de fichiers ou dans le terminal :

			$ sudo umount /dev/sdb1

  ou

			$ sudo umount /dev/mmcblk0p1

  Le répéter avec l'éventuelle deuxième partition de la carte.
- Copier l'image sur la carte SD avec l'utilitaire `dd` :

			$ sudo dd if=~/Downloads/pcduino3_a20_kernel_dd_20140721.img of=/dev/sdb bs=1M

  (ne pas préciser de numéro de partition).
- À la fin de la copie, entrer `sync` pour vider le buffer d'écriture puis éjecter la carte :

			$ sudo eject /dev/sdb

- Insérer la carte microSD dans le pcDuino3 et le mettre sous-tension.
- Attendre que les LED TX/RX cessent de clignoter (quelques minutes).

Il faut ensuite installer Ubuntu. Cette fois-ci, il suffit de copier-coller le fichier sur la carte SD. En effet, le noyau fraîchement flashé attend la présence du fichier `update.sh` sur une carte SD.

- Télécharger l'installation d'Ubuntu 12.04 toujours sur la même page, la dernière version est [celle-ci](https://s3.amazonaws.com/pcduino/Images/v3/20140430/pcduino3_ubuntu_20140430.7z).
- Extraire l'archive :

			$ 7za e ~/Downloads/pcduino3_ubuntu_20140430.7z

  Si les outils de fichiers 7z ne sont pas installés :

			$ sudo apt-get install p7zip

- Formater la carte SD en FAT32 avec GParted par exemple.
  Si GParted n'est pas installé :

			$ sudo apt-get install gparted

- Éjecter et insérer la carte SD dans le pcDuino3. Des informations sont affichées sur l'écran HDMI au cours de l'installation qui dure environ 8 minutes.
- Redémarrer le pcDuino3 et tout devrait être installé.

Pour les réglages de l'OS, entrer la commande suivante dans un terminal :

    $ sudo board-config

Le login est `ubuntu`, le mot de passe est `ubuntu` et le nom de la machine (hostname) est `ubuntu`.

Pour se connecter en SSH, il faudra donc écrire :

    $ ssh ubuntu@ubuntu.local

Pour le pcDuino3B, la procédure est la même, mais la page de téléchargement est la suivante : <http://www.linksprite.com/?page_id=858>.

### Ubuntu 14
Pour Ubuntu 14, une seule opération est nécessaire, celle avec `dd`.

Pour cela, télécharger l'image se trouvant plus bas dans la même page que précédemment (différente pour le pcDuino3 et le pcDuino3B).

Il s'agit en fait de Linaro LXDE basé sur Ubuntu et non pas un Lubuntu total mais il y a peu de changements.

Problème de réseau car géré avec wicd. Je n'ai pas réussi à utiliser le client DHCP pour se connecter au réseau de Marcoule.

Ubuntu 14 fonctionne avec wicd mais il est parfois capricieux... Les mises à jour fonctionnent. Toutefois, éviter de forcer la mise à jour du noyau (`sudo apt-get dist-upgrade`), on ne sait jamais.

Le login est `linaro`, le mot de passe est `linaro` et le nom de la machine (hostname) est `linaro-alip`.

Pour se connecter en SSH, il faut donc écrire :

    $ ssh linaro@linaro-alip.local

## Paquets installés lors d'une fresh install
Liste non exhaustive

### Paquets généralistes

    screen
    byobu
    vim
    htop
    tree
    i2c-tools
    libi2c-dev
    python-smb
	ntpdate

### Paquets ROS
Pour la procédure pour installer ROS Hydro sur ARM, suivre ce tuto <http://wiki.ros.org/hydro/Installation/UbuntuARM>
En plus de l'installation ROS de base :

    ros-hydro-ros-base
    ros-hydro-nodelet-core
    ros-hydro-dynamixel-motor
    ros-hydro-common-msgs
    ros-hydro-xacro
    ros-hydro-robot-state-publisher
    ros-hydro-cmake-modules

## Installer les sources Ubuntu 14.04 LTS sur le pcDuino
Non testé, mais ce serait possible :
<http://learn.linksprite.com/pcduino/linux-applications/use-ubuntu-14-04-lts-trusty-source-on-pcduino-ubuntu-12-04lts-precise/>

## Connexion au réseau de Marcoule
Pour se connecter au réseau intra de Marcoule afin d'avoir une connexion internet, il faut préciser le **DHCP client ID** dans les réglages réseau Ethernet.

1. Aller dans les réglages réseau (ou taper `sudo nm-connection-editor` dans un terminal, le `sudo` semble necessaire, sinon aucune connexion n'est listée)
2. Sélectionner la connexion ethernet (*wired*), puis **Edit**
3. Dans l'onglet **IPv4 Settings**, renseigner le champ **DHCP client ID** avec **Dhcp_M@r!**.
4. **Save**
5. Redémarrer

*Remarque : il est possible de faire cette opération en SSH si le X11 forwarding a été activé (option `-Y` dans la commande ssh).*

Source : Julien Favrichon ;)

## Changer message d'accueil
Il s'agit du message informatif afficher au début d'une connexion SSh par exemple.

Créer le fichier `/etc/motd.tail` et y inscrire le message souhaité.

Exemple :

		* Penser a regler la date.
			 - Manuellement :
					 sudo date MMJJHHmmAAAA
			 - Ou plus precisement :
					 sudo ntpdate 192.168.0.103

Source : <http://serverfault.com/questions/407033/how-can-i-edit-the-welcome-message-when-ssh-start>

## Point d'accès Wifi sur le pcDuino
Afin de simplifier la procédure de connexion, il parraît plus judicieux d'utiliser le module Wifi du pcDuino comme point d'accès wifi. Ainsi, n'importe quel ordinateur peut se connecter au pcDuino. Il faudra en revanche utiliser un modem USB pour se connecter à internet si besoin, or le noyau utilisé ne prend pas en charge les modem USB.

### Installer le driver et hostapd
**ATTENTION : Ne fonctionne pas !**

Installer les `linux-headers`

				$ sudo apt-get install pcduino-linux-headers-3.4.79+

Pour éviter que l'interface réseau (`wlanX`) ne soit modifiée toute seule, commenter toutes les lignes sauf la première dans le fichier `/etc/udev/rules.d/70-persistent-net.rules` :

				$ sudo vim /etc/udev/rules.d/70-persistent-net.rules

Dans la première ligne, remplacer l'adresse mac (`ATTR{address}==`) par une étoile :

        SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="*", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="wlan*", NAME=wlan0"

La suite n'est pas expliquée ici puisque cela ne focntionne pas.

Source : <http://learn.linksprite.com/pcduino/pcduino-embedded-linux-development/enable-ap-mode-for-pcduino2-green-wifi-module/> adapté au pcDuino3.

## La date sur le pcDuino
Le pcDuino ne dispose pas de module horloge RTC (Real Time Clock) avec une pile permettant de conserver l'information de date.

Il se connecte alors à un serveur NTP lorsqu'il est conecté à internet. Mais sur le robot, il n'a pas accès à internet, il conserve alors une date par défaut (janvier 2010) mais cela pose problème à ROS qui utilise la date à de nombreuses reprises (logs...).

### ntpdate
La façon la plus fiable est d'utiliser `ntpdate` pour récupérer la date d'un ordianteur distant.

- Installer `ntpdate` sur les deux PC :

				$ sudo apt-get install ntpdate

- Installer **en plus** `ntp` sur le PC fixe :

				$ sudo apt-get install ntpdate ntp

- Ensuite il suffit de lancer la commande suivante après le démarrage sur le pcDuinop :

				$ sudo ntpdate adresse.ip.pc.fixe.

### En SSH
Une solution est d'utiliser la date du PC fixe et l'envoyer par SSH. Pour cela, il faut être connecté sur le pcDuino et établir une connexion SSH vers le PC fixe qui doit donc avoir un serveur SSH opérationnel.

    $ sudo date --set="$(ssh user@server 'date -u')"

Source : <http://www.commandlinefu.com/commands/view/9153/synchronize-date-and-time-with-a-server-over-ssh>

### Manuellement
Régler la date manuellement avec le format : "MMJJHHmmYYYY" :

    $ sudo date 071412302006

Source : <http://www.aide-ubuntu.com/date-visualiser-et-modifier-la>

## Connexion SSH sans mot de passe
Afin de se connecter au pcDuino en SSH sans qu'il nous demande à chaque fois le mot de passe, il faut configurer le SSH. Il faut donc avoir un jeu de clés sur le PC fixe (supposé vu qu'on utilise Git). Ensuite, il faut ajouter notre clé publique dans les clés autorisées du pcDuino :

    $ cat ~/.ssh/id_rsa.pub | ssh ubuntu@192.168.0.102 'cat >> .ssh/authorized_keys'

Sur le PC fixe, la passphrase de la clé SSH sera demandée à chaque fois. Pour éviter cela, on utilise le gestionnaire de mots de passe **Gnome Keyring**.

## Connexion SSH avec X11 forwarding
Le problème de la connexion SSH, c'est que l'on ne peut exécuter que des programmes en ligne de commande et non des utilitaires avec des interfaces graphiques. C'est en fait possible avec la méthode du X11 forwarding. Cela consiste à rediriger les flux X11 donc graphiques sur la machine de l'utilisateur. Pour cela, il suffit de lancer la connexion SSH avec l'option `-Y` :

    ssh -Y utilisateur@adresse_ip

En réalité, le X11 forwarding s'utilise avec l'option `-X` mais cette connexion est soumise à des règles de sécurité qui sont évitées avec `-Y`, ce qui suffit pour un connexion locale.

*EDIT : pas forcement utile :*

Pour utiliser le X11 forwarding dans GNU Screen, il faut reporter la valeur de la variable d'environnement `DISPLAY`. Pour cela, suite à la connexion SSH avec X11 forwarding, afficher la valeur de cette variable :

    echo $DISPLAY

Noter cette valeur et lancer Gnu Screen :

    screen

Dans chaque terminal de Gnu Screen, exporter la variable `DISPLAY` avec la valeur notée précédemment :

    export DISPLAY=localhost:10.0

## Batterie pour pcDuino
![Connecteur de batterie sur le pcDuino](ressources/pcduino_battery_connector.jpg "Connecteur de batterie sur le pcDuino")

Le pcDuino intègre le compsant [**AXP209**](http://linux-sunxi.org/AXP209), c'est un *PMU* (*Power Management Unit*) qui sait utiliser les batteries LiPo de 3,7 V et aussi les recharger grâce au [connecteur 5 broches JST SH du pcDuino](http://learn.linksprite.com/pcduino/schematics/chargeable-battery-connector-for-pcduino3/), dont la [datasheet](http://learn.linksprite.com/wp-content/uploads/2014/05/bat-conn.pdf) est disponible (broche 1 marquée par un triangle rouge sur le PCB).

1. VCC
2. VCC
3. NC
4. GND
5. GND

Le câble qui va bien, le [JST SH 5 wire](https://www.sparkfun.com/products/10360). Mais les petites baterries LiPo de 3,7 V utilisent des connecteurs JST PH (pas de 2 mm) à deux fils. Le JST SH a un pas de 1 mm.

- Le connecteur chez Farnell : <http://fr.farnell.com/jst-japan-solderless-terminals/shr-05v-s-b/boitier-de-connecteur-5-voies/dp/1679111?MER=i-9b10-00001460>
- Les terminaisons à sertir qui vont avec : <http://fr.farnell.com/jst-japan-solderless-terminals/ssh-003t-p0-2/terminaison-a-sertir-serie-sh/dp/1679142?MER=i-9b10-00001460>
- Le connecteur mâle et femelle par lot de 10 à 2,78 € sur ebay : <http://www.ebay.fr/itm/10-Sets-Micro-JST-2-0-PH-5-Pin-Connector-plug-Male-Female-Crimps-/181441351414?pt=LH_DefaultDomain_0&hash=item2a3ebf4ef6>

Sinon, possibilité d'utiliser une *PowerBank* comme celle-ci <http://www.generationrobots.com/fr/401927-batterie-nomade-power-bank-5000mah.html>. (La mention de 1 A en sortie alors que le fabricant du pcDuino demande 2 A ne devrait pas poser problème).

Test avec une [batterie LiPo de 3,7 V et 1200 mAh](http://www.generationrobots.com/fr/402085-lithium-ion-polymer-battery-37v-1200mah.html) et le Li-Po Rider de Seeedstudio (voir plus bas) : le pcDuino ne démarre pas. Sûrement pas assez de puissance.

## Seeed Studio Li-Po Rider
Le [Li-Po Rider](http://www.seeedstudio.com/wiki/Lipo_Rider_V1.1) de Seeedstudio est un chargeur de batteries Li-Po de 3,7 V. Il permet de les charger via un panneau solaire ou une prise USB mini B et permet la communication vers le port de sortie USB type A.

## i2c
Pour accéder au bus i2c depuis la ligne de commande, il faut installer les `i2c-tools` :

    $ sudo apt-get install i2c-tools

On peut alors utiliser les outils `i2cdectect`, `i2cget` et `i2cset` pour communiquer avec une périphérique i2c depuis la ligne de commande :

    $ sudo i2cdetect -y 2

Le bus i2c présent sur le connecteur Arduino est le bus n°2. Le bus n°1 est utilisé sur la carte pour communiquer avec le PMU AXP209 (voir paragraphe suivant).

Pour utiliser les [fonctions SMBus](https://www.kernel.org/doc/htmldocs/device-drivers/i2c.html) pour contrôler le port i2c, il convient d'installer le paquet `libi2c-dev` pour le C et `python-smbus` pour le Python :

    $ sudo apt-get install libi2c-dev python-smb

Pour accéder à l'i2c sans être root, il faut ajouter l'utilisateur au groupe `i2c` :

    $ sudo adduser utilisateur i2c

Sources :

- [Article de linksprite](http://learn.linksprite.com/pcduino/arduino-ish-program/i2c/how-to-use-i2c-tools-on-pcduino-3-ubuntu/)
- [Tuto programmation i2c Sparkfun](https://learn.sparkfun.com/tutorials/programming-the-pcduino/i2c-communications)
- [Doc i2c noyau Linux](http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c/dev-interface)
- [Infos i2c Raspberry Pi](https://www.abelectronics.co.uk/i2c-raspbian-wheezy/info.aspx)

## AXP209
Comme vu précédemment, le pcDuino est équipé du PMU [AXP209](http://linux-sunxi.org/AXP209). Celui-ci peut communiquer en i2c (bus n°1) au processus A20 pour donner des informations sur la batterie et l'alimentation.

### Script Python avec GUI
Script Python trouvé sur le [forum de linksprite](http://forum.linksprite.com/index.php?/topic/2980-python-script-to-read-input-voltage-and-current-draw-of-pcduino/) qui affiche la tension et le courant consommés par le pcDuino dans une petite fenêtre.

Le principe consiste simplement à lire la valeur des fichiers `/sys/class/power_supply/ac/voltage_now` pour la tension et `/sys/class/power_supply/ac/current_now` pour le courant.

```Python
import Tkinter as tk
import time

def Draw():
    global text1
    global text2


    frame=tk.Frame(root,width=100,height=100,relief='solid',bd=1)
    text1=tk.Label(root,text='Voltage',pady=10)
    text2=tk.Label(root,text='Current',padx=20)
    text1.pack()
    text2.pack()

def Refresher():
    global text1
    global text2

    AC_PATH='/sys/class/power_supply/ac/voltage_now'
    file = open(AC_PATH,'r')
    txt=file.read()
    file.close
    Volts=int(txt)/1000000.0
    file.close
    text1.configure(text='Volts = '+str(Volts))
    AC_PATH='/sys/class/power_supply/ac/current_now'
    file = open(AC_PATH,'r')
    txt=file.read()
    file.close
    Current=int(txt)/1000.0
    file.close
    text2.configure(text='Current = '+str(Current))
    root.after(1000,Refresher)

root=tk.Tk()
root.title("pcDuino Health")
Draw()
Refresher()
root.mainloop()
```
### Script Python sans GUI
Issu de la [même page](http://forum.linksprite.com/index.php?/topic/2980-python-script-to-read-input-voltage-and-current-draw-of-pcduino/), un exemple sans GUI, mais utilisant directement l'utilitaire `i2cget`, mais en étant obligé de forcer la communication (moins propre) :

```Python
import subprocess

volth=subprocess.check_output(['i2cget','-y','-f','0','0x34','0x56'])
voltl=subprocess.check_output(['i2cget','-y','-f','0','0x34','0x57'])
volts=(int(volth,16)<<4)+(int(voltl,16))
ampsh=subprocess.check_output(['i2cget','-y','-f','0','0x34','0x58'])
ampsl=subprocess.check_output(['i2cget','-y','-f','0','0x34','0x59'])
amps=(int(ampsh,16)<<4)+(int(ampsl,16))
volts=volts*0.0017
amps=amps*0.000625
print 'Volts =',volts
print 'Amps =',amps
```

### Script shell
Sur [un article](http://hardware-libre.fr/2014/11/banana-pi-axp209-battery-power-monitoring/) à propos du Banana Pi qui utilise le même processeur Allwiner A20 et le PMU AXP209, on trouve le script shell suivant (nécessite que l'utilitaire `bc`soit installé) :

```shell
#!/bin/sh
# This program gets the battery info from PMU
# Voltage and current charging/discharging
#
# Nota : temperature can be more than real because of self heating
#######################################################################
# Copyright (c) 2014 by RzBo, Bellesserre, France
#
# Permission is granted to use the source code within this
# file in whole or in part for any use, personal or commercial,
# without restriction or limitation.
#
# No warranties, either explicit or implied, are made as to the
# suitability of this code for any purpose. Use at your own risk.
#######################################################################

# force ADC enable for battery voltage and current
i2cset -y -f 0 0x34 0x82 0xC3

################################
#read Power status register @00h
POWER_STATUS=$(i2cget -y -f 0 0x34 0x00)
#echo $POWER_STATUS

BAT_STATUS=$(($(($POWER_STATUS&0x02))/2))  # divide by 2 is like shifting rigth 1 times
#echo $(($POWER_STATUS&0x02))
echo « BAT_STATUS= »$BAT_STATUS
# echo $BAT_STATUS

################################
#read Power OPERATING MODE register @01h
POWER_OP_MODE=$(i2cget -y -f 0 0x34 0x01)
#echo $POWER_OP_MODE

CHARG_IND=$(($(($POWER_OP_MODE&0x40))/64))  # divide by 64 is like shifting rigth 6 times
#echo $(($POWER_OP_MODE&0x40))
echo « CHARG_IND= »$CHARG_IND
# echo $CHARG_IND

BAT_EXIST=$(($(($POWER_OP_MODE&0x20))/32))  # divide by 32 is like shifting rigth 5 times
#echo $(($POWER_OP_MODE&0x20))
echo « BAT_EXIST= »$BAT_EXIST
# echo $BAT_EXIST

################################
#read Charge control register @33h
CHARGE_CTL=$(i2cget -y -f 0 0x34 0x33)
echo « CHARGE_CTL= »$CHARGE_CTL
# echo $CHARGE_CTL

################################
#read Charge control register @34h
CHARGE_CTL2=$(i2cget -y -f 0 0x34 0x34)
echo « CHARGE_CTL2= »$CHARGE_CTL2
# echo $CHARGE_CTL2

################################
#read battery voltage    79h, 78h    0 mV -> 000h,    1.1 mV/bit    FFFh -> 4.5045 V
BAT_VOLT_LSB=$(i2cget -y -f 0 0x34 0x79)
BAT_VOLT_MSB=$(i2cget -y -f 0 0x34 0x78)

#echo $BAT_VOLT_MSB $BAT_VOLT_LSB

BAT_BIN=$(( $(($BAT_VOLT_MSB << 4)) | $(($(($BAT_VOLT_LSB & 0xF0)) >> 4)) ))

BAT_VOLT=$(echo « ($BAT_BIN*1.1) »|bc)
echo « Battery voltage = « $BAT_VOLT »mV »

###################
#read Battery Discharge Current    7Ah, 7Bh    0 mV -> 000h,    0.5 mA/bit    FFFh -> 4.095 V
BAT_IDISCHG_LSB=$(i2cget -y -f 0 0x34 0x7B)
BAT_IDISCHG_MSB=$(i2cget -y -f 0 0x34 0x7A)

#echo $BAT_IDISCHG_MSB $BAT_IDISCHG_LSB

BAT_IDISCHG_BIN=$(( $(($BAT_IDISCHG_MSB << 4)) | $(($(($BAT_IDISCHG_LSB & 0xF0)) >> 4)) ))

BAT_IDISCHG=$(echo « ($BAT_IDISCHG_BIN*0.5) »|bc)
echo « Battery discharge current = « $BAT_IDISCHG »mA »

###################
#read Battery Charge Current    7Ch, 7Dh    0 mV -> 000h,    0.5 mA/bit    FFFh -> 4.095 V
BAT_ICHG_LSB=$(i2cget -y -f 0 0x34 0x7D)
BAT_ICHG_MSB=$(i2cget -y -f 0 0x34 0x7C)

#echo $BAT_ICHG_MSB $BAT_ICHG_LSB

BAT_ICHG_BIN=$(( $(($BAT_ICHG_MSB << 4)) | $(($(($BAT_ICHG_LSB & 0xF0)) >> 4)) ))

BAT_ICHG=$(echo « ($BAT_ICHG_BIN*0.5) »|bc)
echo « Battery charge current = « $BAT_ICHG »mA »
```
### Script Python pour stocker l'état de la batterie
Pour tester l'autonomie de la batterie Powerbank **EasyAcc** de 5000 mAh, un script Python a été écrit pour stocker les valeurs de tension et courant avec l'heure :

```Python
#!/usr/bin/env python
# coding: utf-8
import time

AC_VOLTAGE_PATH = '/sys/class/power_supply/ac/voltage_now'
AC_CURRENT_PATH = '/sys/class/power_supply/ac/current_now'
OUTPUT_PATH = '/home/ubuntu/battery.dat'

def read_write():
    timestamp = time.time()
    file = open(AC_VOLTAGE_PATH, 'r')
    raw_voltage = file.read()
    file.close()
    voltage = int(raw_voltage)/1000000.0
    file = open(AC_CURRENT_PATH, 'r')
    raw_current = file.read()
    file.close()
    current = int(raw_current)/1000.0
    file = open(OUTPUT_PATH, 'a')
    file.write(str(timestamp) + ',' + str(voltage) + ',' + str(current) + '\n')

while(1):
    read_write()
    time.sleep(120)
```

Et le script pour tracer les courbes avec `matploblib` :

```Python
#!/usr/bin/env python
# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as md
import time
import datetime

data_file = np.genfromtxt('battery.dat', delimiter=',')

timestamp = data_file[:,0]
voltage = data_file[:,1]
current = data_file[:,2]

# Conversion de la date pour matplotlib
datetimestamp = [datetime.datetime.fromtimestamp(element) for element in timestamp]
date_num = [md.date2num(element) for element in datetimestamp]

plt.plot_date(date_num, voltage, '-')

plt.show()
```

## Programmer ADC et GPIO du pcDuino
**ATTENTION, la tension de fonctionnement du pcDuino et donc des GPIO est de 3,3 V !**

**ATTENTION ! Le pcDuino ne dispose que de 6 entrées analogiques dont 2 (adc0 et adc1) 6 bits limitées à 2,0 V !**

### Activation des modules du noyau
Tout d'abord, les modules `adc` et `gpio` du noyau Linux n'étaient pas chargés.

        $ sudo modprobe gpio
        $ sudo modprobe adc

Pour les charger automatiquement au démarrage, il faut modifier le fichier `/etc/modules` et y ajouter les deux modules, un par ligne :

        gpio
        adc

Sources :

- Activer le module adc du pcDuino3 : <http://learn.linksprite.com/pcduino/arduino-ish-program/adc/how-to-add-adc-module-on-pcduino-3/>
- Document sur les modules Linux : <http://doc.ubuntu-fr.org/tutoriel/tout_savoir_sur_les_modules_linux>

### Programmation Python sans bibliothèque
> Dans Linux, tout est fichier.

Effectivement, tout est fichier, donc pour lire l'état d'une entrée GPIO, écrire une valeur de l'ADC, il faut ouvrir le fichier et lire la valeur qu'il contient.

Le pcDuino comporte six entrées analogiques nommées de `adc0` à `adc5`. Les deux premiers sont sur 6 bits (de 0 à 63) et de **0 à 2,0 V**, alors que les trois autres sont sur 12 bits (0 à 4095) de **0 à 3,3 V**. Les fichiers relatifs à l'ADC se trouvent dans le dossier `/proc` et se nomment très logiquement `adc0`, ... `adc5`.

```Python
#!/usr/bin/env python

import time, os

# Utilisation de constantes pour stocker le chemin vers les fichiers adc
ADC_PATH= os.path.normpath('/proc/')
ADC_FILENAME = "adc"

# Creation d'une liste des fichiers adc
adcFiles = []
for i in range(6):
  adcFiles.append(os.path.join(ADC_PATH, ADC_FILENAME+str(i)))

# Lecture des fichiers
# Attention, la valeur est lue sous forme de string, il faut
# utiliser la methode int() pour effectuer des calculs sur cette valeur
for file in adcFiles:
  fd = open(file, 'r')
  fd.seek(0)
  print "ADC Channel: " + str(adcFiles.index(file)) + " Result: " + fd.read(16)
  fd.close()
```

De même pour les GPIO, ils se trouvent dans le dossier `/sys/devices/virtual/misc/gpio`. En fait dans ce répertoire se trouvent deux dossiers, un pour le mode des broches (entrée ou sortie) et un pour l'état des broches :

        /sys/devices/virtual/misc/gpio/mode/
        /sys/devices/virtual/misc/gpio/pin/

Pour le `mode`, **'0'** met la pin en entrée, **'1'** en sortie et **'8'** en entrée avec résistance de pull-up.

Pour `pin`, il suffit de lire (attention, sous forme de string) ou d'écrire 1 ou 0. Attention, il faut les droit administrateur pour modifier le ficher.

Exemple basique (gpio18 est la LED TX) :

        $ echo '1' > /sys/devices/virtual/misc/gpio/pin/gpio18

Exemple de Sparkfun :

```Python
#!/usr/bin/env python

import time, os

## For simplicity's sake, we'll create a string for our paths.
GPIO_MODE_PATH= os.path.normpath('/sys/devices/virtual/misc/gpio/mode/')
GPIO_PIN_PATH=os.path.normpath('/sys/devices/virtual/misc/gpio/pin/')
GPIO_FILENAME="gpio"

## create a couple of empty arrays to store the pointers for our files
pinMode = []
pinData = []

## Create a few strings for file I/O equivalence
HIGH = "1"
LOW =  "0"
INPUT = "0"
OUTPUT = "1"
INPUT_PU = "8"

## First, populate the arrays with file objects that we can use later.
for i in range(0,18):
  pinMode.append(os.path.join(GPIO_MODE_PATH, 'gpio'+str(i)))
  pinData.append(os.path.join(GPIO_PIN_PATH, 'gpio'+str(i)))

## Now, let's make all the pins outputs...
for pin in pinMode:
  file = open(pin, 'r+')  ## open the file in r/w mode
  file.write(OUTPUT)      ## set the mode of the pin
  file.close()            ## IMPORTANT- must close file to make changes!

## ...and make them low.
for pin in pinData:
  file = open(pin, 'r+')
  file.write(LOW)
  file.close()

## Next, let's wait for a button press on pin 2.
file = open(pinMode[2], 'r+') ## accessing pin 2 mode file
file.write(INPUT_PU)          ## make the pin input with pull up
file.close()                  ## write the changes

temp = ['']   ## a string to store the value
file = open(pinData[2], 'r') ## open the file
temp[0] = file.read()       ## fetch the pin state

## Now, wait until the button gets pressed.
while '0' not in temp[0]:
  file.seek(0)      ## *MUST* be sure that we're at the start of the file!
  temp[0] = file.read()   ## fetch the pin state
  print "Waiting for button press..."
  time.sleep(.1)  ## sleep for 1/10 of a second.

file.close()  ## Make sure to close the file when you're done!

## Now, for the final trick, we're going to turn on all the pins, one at a
##   time, then turn them off again.
for i in range(3,17):
  file = open(pinData[i], 'r+')
  file.write(HIGH)
  file.close()
  time.sleep(.25)

for i in range(17,2, -1):
  file = open(pinData[i], 'r+')
  file.write(LOW)
  file.close()
  time.sleep(.25)

```

Source : <https://learn.sparkfun.com/tutorials/programming-the-pcduino>

### Programmation des GPIO avec le module python-pcduino
Linksprite, le concepteur du pcDuino fournit un module Python pour gérer les GPIO à la manière d'Arduino, `python-pcduino`.

Il faut le télécharger à partir de Github :

        $ git clone https://github.com/pcduino/python-pcduino.git
        $ cd python-pcduino
        $ sudo python setup.py install

Il fournis alors des fonctions suivantes :

- `pin_mode(channel, mode)` : mode prend les valeurs INPUT ou OUTPUT
- `digital_read(channel)`
- `digital_write(channel, value)` : value prend les valeurs HIGHT ou LOW
- `analog_read(channel)`
- `analog_write(channel, value)` : PWM avec une valeur entre 0 et 255

Une écriture de la fonction delay() :

```Python
def delay(ms):
    sleep(1.0*ms/1000)
```

Source : <http://learn.linksprite.com/pcduino/arduino-ish-program/adc/use-python-to-read-adc-of-pcdunio/>

### Programmation en Python avec pingo
Il existe aussi la bibliothèque [pingo](https://github.com/pcduino/pingo). Elle a l'avantage d'être dispnible sur Raspberry Pi, Beagle Bone Black et pcDuino, ce qui peut apporter une certaine portabilité. Cependant elle un support limité, on ne l'utilisera donc pas ici.

## Problème de lancement Arduino IDE
**En fait, malgré la solution ci-dessous, il y a toujours un soucis :(** (mis de côté pour l'instant puisque le code utilisé sera en Python pour être compatible ROS).

Suite à l'installation de ROS, j'ai modifié les *locales*. Mais depuis, l'IDE Arduino ne se lance plus à cause d'un problème de *locale*.

        Exception in thread "main" java.util.MissingResourceException : can't find bundle for base name processing.app.i18n.Resources, locale en_us

La solution consiste à lancer l'utilitaire de configuration du pcDuino :

        $ sudo board-config.sh

Et de choisir l'option **set_locale** pour revenir à la configuration anglaise par défaut.

Source : <http://www.electrodragon.com/w/Version0531_pcduino>

## Avahi
Pour bénéficier des services ZeroConf, il faut installer Avahi :

        $ sudo apt-get install avahi-utils

Activation :

        $ sudo service avahi-daemon restart

Pour modifier le nom de la machine :

        sudo hostname pcduino

Pour une modification durable, il fautmodifier les fichiers `/etc/hosts` et `/etc/hostname`. Attention, le nom ne doit contenir que des lettres, chiffres et le symbole '-', pas de '_'.

Pour que le démon avahi se lance au démarrage, il convient d'exécuter :

        $ sudo update-rc.d avahi-daemon defaults

Sources : <http://www.pobot.org/Connexion-au-robot-sur-pcDuino.html?lang=fr> et <http://serverfault.com/questions/124774/have-a-service-start-on-startup-with-ubuntu>

## ADC 12 bits ADS1015
L'[ADS1015](http://www.ti.com/product/ads1015) de Texas Instruments est un convertisseur analogique numérique 12 bits avec 4 entrées. Il est commercialisé sous forme d'une breakout board par [Adafruit](http://www.adafruit.com/product/1083) et revendu en Franche chez [Génération Robots](http://www.generationrobots.com/fr/401467-convertisseur-analogique-vers-num%C3%A9rique-amplificateur.html).

**Attention** cependant, les 12 bits ne sont utilisables qu'en mode différentiel. En mode *single-ended*, c'est-à-dire que l'on mesure la différence de tension entre une entrée et la masse (comme utilisé ici sur le robot), on perd la moité de la plage de mesure donc il devient un convertisseur 11 bits (0 à 2048).

### Adressage
Il est possible de modifier l'adresse i2c du module pour utiliser plusieurs modules sur le même bus i2c. L'adresse par défaut est `Ox48`.

Pour modifier l'adresse il suffit de relier la broche `ADDR` à une des broches suivantes :

- ADDR->GND : `Ox48`
- ADDR->VDD : `Ox49`
- ADDR->SDA : `Ox4A`
- ADDR->SCL : `Ox4B`

### Différentiel
Il est possible d'utiliser l'ADC en mode différentiel avec alors 2 entrées possibles, `A0&A1` et `A2&A3`.

### Bibliothèque Python
Adafruit fournit une bibliothèque pour Raspberry Pi parmi sont ensemble de [codes Python pour Raspberry Pi](https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code) et un [tutoriel](https://learn.adafruit.com/adafruit-4-channel-adc-breakouts) pour l'utiliser avec un Arduino.

#### Installation
Télécharger la bibliothèque en clonant le [dépôt Github](https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code) :

    $ git clone https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code.git

La partie de gestion de l'ADS1015 s'appuie sur `python-smbus` et `Adafruit_I2C`. Il faudra alors simplement modifier le fichier `Adafruit_I2C.py` si besoin pour les accès bas niveau à l'i2c.

Il convient donc d'installer `python-smbus` pour utiliser cette bibliothèque :

    $ sudo apt-get install python-smbus

Ensuite, pour faire fonctionner la bibliothèque `Adafruit_ADS1x15`, il suffit de modifier le driver i2c dans le fichier `Adafuit_I2C.py` pour renseignier le numéro du bus i2c utilisé, en l'occurence, `i2c-2` sur le pcDuino.

Pour le vérifier, il suffit d'exécuter :

    $ i2cdetect -y <numero_du_bus>

Et de modifier le numéro du bus pour voir apparaître les adresses des périphériques connectés.

Dans le fichier `Adafruit_I2C.py`, il suffit alors de commenter la méthode `getRevision()` qui est normalement utilisée pour connaître la révision du Raspberry Pi utilisé car les premières révisions utilisaient le bus 0 et les suivantes le bus 1. Ensuite, on modifie la méthode `getPiI2CBusNumber()` pour quelle retourne toujours 2 :

```Python
def getPiI2CBusNumber():
    # Gets the I2C bus number /dev/i2c#
    return 2
```

#### Utilisation
Il est alors maintenant possible d'exécuter l'exemple `ads1x15_ex_singleended.py` en ayant pris soin de modifier la version utilisée :

    adc = ADS1x15(ic=ADS1015)

Pour afficher les valeurs de 2 ADC sur le même bus, mais ayant des adresses différentes, il est possible d'utiliser le code suivant :

```python
adc = [ADS1x15(0x48, ic=ADS1015), ADS1x15(0x49, ic=ADS1015)]

for board_num in range(2):
    for adc_num in range(4):
        volts = adc[board_num].readADCSingleEnded(adc_num, gain, sps) / 1000
        print "Board num.%d, ADC%d: %.6f V" % (board_num+1, adc_num, volts)
```

### Bibliothèque C++
Une bibliothèque C++ a été développée pour éviter les bugs/lenteurs des threads Python dans ROS Hydro sur le pcDuino. Il est possible d'atteindre une fréquence de publication des messages dans ROS d'environ 115 Hz pour les lecture de 6 valeurs analogiques avec 2 ADS1015.

Cette bibliothèque est basée sur [la bibliothèque Arduino d'Adafruit](https://github.com/adafruit/Adafruit_ADS1X15) mais adaptée pour utiliser le port i2c de la pcDuino sous Linux. En plus, la possibilité de choisir la fréquence des mesures a été ajoutée.

Elle diponible sur le Gitlab : <https://agita/hexapod/adafruit_ads1015_linux>

## Pololu MinIMU-9 v3
Centrale inertielle 9 axes produite par [Pololu](https://www.pololu.com/product/2468) et vendue par [Génération Robots](http://www.generationrobots.com/fr/401886-capteur-minimu-9-v3-gyroscope-accelerometre-et-boussole.html).

Elle comprend un gyroscope 3 axes, le [ST L3GD20H](http://www.st.com/web/catalog/sense_power/FM89/SC1288/PF254039) (adresse i2c : `0x6B`) et un accéléromètre 3 axes et magnétomètre 3 axes, le [ST LSM303D](http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF253884) (adresse i2c : `0x1D`).

### Bibliothèque Arduino
Pololu fournit une [bibliothèque Arduino](https://github.com/pololu/minimu-9-ahrs-arduino) avec un code de test Python utilisant pyserial pour lire les données et visualiser l'orientation en 3D. Elle utilise les bibliothèques Pololu [L3G-arduino](https://github.com/pololu/l3g-arduino) pour le gyromètre et [LSM303-arduino](https://github.com/pololu/lsm303-arduino) pour l'accéléromètre et magnétomètre.

### Bibliothèque pour PC en i2c
#### minimu-9-ahrs
David Grayson, un employé de Pololu a créer un programme pour utiliser la MinIMU-9 avec un Raspberry Pi en i2c. Il y a en fait un programme pour faire l'acquisition des données et le calcul de fusion, [minimu-9-ahrs](https://github.com/DavidEGrayson/minimu9-ahrs) qui devrait pouvoir être utilisable sur un autre PC comme la pcDuino, et un programme de visualisation se basant sur OpenGL ES 2.0, [ahrs-visualizer](https://github.com/DavidEGrayson/ahrs-visualizer). Ce dernier en revanche, a été écrit en fonction des possibilités du GPU du Raspberry Pi et devra être modifié pour fonctionner sur une autre plateforme.

L'inconvénient de `minimu-9-ahrs`, c'est qu'il a été écrit comme un programme standalone et il renvoir tout sur la sortie standart qu'il faut alors "piper" vers un autre programme.

#### RTIMULib
##### Présentation
En revanche, la [RTIMULib](https://github.com/richards-tech/RTIMULib) de *richards-tech* est bien une bibliothèque C++ avec des bindings Python. Elle fonctionne sur les Linux embarqués et des versions [Arduino](https://github.com/richards-tech/RTIMULib-Arduino) et [Teensy](https://github.com/richards-tech/RTIMULib-Teensy) ont même été crées par l'auteur.

Cette bibliothèque propose deux algorithmes de fusion de capteurs, RTQF et Kalman. La bibliothèque poropose 5 programmes :

- RTIMULibDrive est un programme de test affichant l'orientation sous forme des angles d'Euler et sert d'exemple pour utliser la bibliothèque en C++.
- RTIMULibDrive10 : idem que RTIMULibDrive mais pour ces centrales inertielles à 10 axes de liberté, donc avec un baromètre pour l'altitude en plus.
- RTIMULibCal est un programme de calibration de la centrale inertielle. Elle permet une calibration de l'accéléromètre et du magnétomètre et stocke les valeurs dans un fichier de configuration `RTIMULib.ini` qui est lu par les autres programmes s'ils sont exécutés dans le même répertoire.
- RTIMUDemo est un programme en GUI Qt permettant de visualiser les valeurs des différents capteurs et le résultat de la fusion. Il permet de changer d'algorithme de fusion à la volée et d'effectuer les opérations de calibrage.
- RTIMUDemoGL permet les mêmes choses que RTIMUDemo mais avec une partie de visualisation 3D OpenGL.

##### Python binding
Pour le binding Python, un exemple `Fusion.py` est fournit dans le dossier `Linux/python/tests` :

```Python
import sys, getopt
import RTIMU
import os.path
import time
import math

sys.path.append('.')
SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters
imu.setSlerpPower(0.02) # Ratio entre donnees gyro et acc/magn
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

while True:
  if imu.IMURead():
    # x, y, z = imu.getFusionData()
    # print("%f %f %f" % (x,y,z))
    data = imu.getIMUData()
    fusionPose = data["fusionPose"]
    print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]),
        math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
    time.sleep(poll_interval*1.0/1000.0)
```

Un second script d'exemple est donné, `InjectIMU.py`, qui permet d'injecter des valeurs d'IMU au programme de fusion, donc pour ne pas utiliser la partie acquisition de la bibliothèque.

##### Correction de montage de l'IMU
D'après [un article sur le sute richard-tech](https://richardstechnotes.wordpress.com/2014/11/20/using-quaternions-to-correct-imu-installation-offsets/), il est simple de corriger les erreurs de positionnement de l'IMU.

> In order to do this, the offset orientation must be captured by putting the device in its reference position and storing the fused quaternion – fusionQPose. Let’s call this referenceQPose. Then, every time a fused result from the IMU is used, simply pre-multiply the fusionQPose by the conjugate of the referenceQPose:

```cpp
correctedQPose = referenceQPose.conjugate() * fusionQPose;
correctedQPose.toEuler(correctedPose);
```

Qui s'implémenterai comme ceci dans l'application RTIMULibDrive :

```cpp
int i = 0;
while (imu->IMURead()) {
    RTIMU_DATA imuData = imu->getIMUData();
    RTQuaternion referenceQPose;
    RTQuaternion correctedQPose;
    RTVector3 correctedPose;

    sampleCount++;
    now = RTMath::currentUSecsSinceEpoch();

    // display 10 times per second
    if ((now – displayTimer) > 100000) {
        if (i == 0) {
            //the offset orientation must be captured by putting the device in its reference position and storing the fused quaternion – fusionQPose
            referenceQPose = imuData.fusionQPose;
            i++;
        }

        // Then, every time a fused result from the IMU is used,
        //simply pre-multiply the fusionQPose by the conjugate of the referenceQPose:
        correctedQPose = referenceQPose.conjugate() * imuData.fusionQPose;
        correctedQPose.toEuler(correctedPose);
        printf(“Sample rate %d: %s\r”, sampleRate, RTMath::displayDegrees(“”, correctedPose));
    }

    // update rate every second
    if ((now – rateTimer) > 1000000) {
        sampleRate = sampleCount;
        sampleCount = 0;
        rateTimer = now;
    }
}
```

##### Notes
- En python, les quaternions retounés par la méthode `getMeasuredQPose()` sont des tuples.
- En python, les valeurs retournées par la méthode `getIMUData()` sont des dictionnaires.
- En C++, l'attribut `m_data` est une liste de `RTFLOAT` qui est soit `double`, soit `float` en fonction d'un `#define` donc défini à la compilation.
- La méthode `RTQuaternion::conjugate()` n'est pas définie en Python. Mais son implémentation C++ étant assez simple, nous pouvons la reproduire en Python.

    ```c++
    RTQuaternion RTQuaternion::conjugate() const
    {
        RTQuaternion q;
        q.setScalar(m_data[0]);
        q.setX(-m_data[1]);
        q.setY(-m_data[2]);
        q.setZ(-m_data[3]);
        return q;
    }
    ```
    Ce qui donne :

    ```python
    def conjugate(q):
        return q[0], -q[1], -q[2], -q[3]
    ```
- De même pour la méthode `RTQuaternion::toEuler()` :
    ```cpp
    void RTQuaternion::toEuler(RTVector3& vec)
    {
        vec.setX(atan2(2.0 * (m_data[2] * m_data[3] + m_data[0] * m_data[1]),
                1 - 2.0 * (m_data[1] * m_data[1] + m_data[2] * m_data[2])));

        vec.setY(asin(2.0 * (m_data[0] * m_data[2] - m_data[1] * m_data[3])));

        vec.setZ(atan2(2.0 * (m_data[1] * m_data[2] + m_data[0] * m_data[3]),
                1 - 2.0 * (m_data[2] * m_data[2] + m_data[3] * m_data[3])));
    }
    ```

    ```Python
    def to_euler(q):
        x = math.atan2(2.0 * (q[2] * q[3] + q[0] * q[1]),
                1 - 2.0 * (q[1] * q[1] + q[2] * q[2])))
        y = math.asin(2.0 * (q[0] * q[2] - q[1] * q[3])))

        z = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]),
                1 - 2.0 * (q[2] * q[2] + q[3] * q[3])))
        return x, y, z
    ```

#### Adafruit
À noter que les [codes Python pour Raspberry Pi](https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code) d'Adafruit comprennent un module pour le [LSM303](https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code/tree/master/Adafruit_LSM303) et qu'un clône de ce dépôt a ajouté le support pour le [L3GD20](https://github.com/fuzzycode/Adafruit-Raspberry-Pi-Python-Code/tree/master/Adafruit_L3GD20_Unified), ce qui permet de faire l'acquisition des données des capteurs par la même bibliothèque que celle utilisée pour l'ADS1015. La fusion peut alors ensuite être réalisée par une autre bibliothèque, comme avec la méthode `setExtIMUData()` de la RTLibIMU.

### Noeud ROS
Il existe un noeud ROS pour lire les mesures traitées par un Arduino sur la liaison série (utilise rosserial) : <https://github.com/pkok/pololu_imu>

Ce noeud a était écrit pour la MinIMU-9 v2 mais devrait pouvoir être adapté en multipliant les valeurs par 16 (voir la modification lors du passage de la MinIMU-9 v2 (12 bits) à la MinIMU-9 v3 (16 bits)).

## Flags de compilation ROS
Pour modifier les flags de compilation des fichiers C++ de Catkin afin d'optimiser les binaires pour ARMv7 :

> Inside: [workspace]/build/CMakeCache.txt
> Find: //Flags used by the compiler during all build types.
> Change: CMAKE_CXX_FLAGS:STRING=-O3 -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a7

Source : <http://forums.trossenrobotics.com/showthread.php?7411-ROS-enabled-PhantomX-Hexapod&p=67401#post67401>

En réalité, il convient d'accompagner le flag `-mfpu=neon-vfpv4` de `-funsafe-math-optimizations` car NEON n'est pas totalement compatible avec la norme IEEE 754 qui décrit les nombres à virgule flottante. Cela peut donc conduire à des imprécisions sur certains calculs donc par défaut, GCC désactive NEON si on n'utilise pas le flag `-funsafe-math-optimizations`. Pour éviter d'utiliser NEON qui emploie les SIMD, il est possible d'utiliser le flag `-mfpu=vfpv4-d16`.

Cependant, le processeur ARM Cortex A7 n'est pas pris en charge avec le compilateur GCC 4.6 fournit avec Ubuntu 12.04. Les programmes vont bien compiler car l'architecture ARMv7 est bien présente, mais pas les optimisations spécifiques au processeur A7. Cette prise en charge a été apportée par la version 4.7 de GCC et des améliorations ont été faites avec la version 4.8. De plus, la version 4.8 prend en charge la majorité des ajouts de C++11.

Source : <http://wits-hep.blogspot.fr/2013/11/arm-cortex-a7-gcc-flags-allwinner-a20.html>

Pour installer le compilateur GCC version 4.8 il faut utiliser le PPA de la [Toolchain Ubuntu](https://launchpad.net/~ubuntu-toolchain-r/+archive/ubuntu/test).

    $ sudo apt-get install python-software-properties
    $ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    $ sudo apt-get update
    $ sudo apt-get install gcc-4.8 g++-4.8
    $ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50
    $ sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 50

Pour changer de version :

    $ sudo apt-get install gcc-4.7 g++-4.7
    $ sudo update-alternatives --remove gcc /usr/bin/gcc-4.8
    $ sudo update-alternatives --remove g++ /usr/bin/g++-4.8
    $ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.7 50
    $ sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.7 50

Source : <http://askubuntu.com/questions/271388/how-to-install-gcc-4-8>

Cependant, le passage à GCC 4.8 ou 4.7 ne permet plus de compiler les paquets ROS à cause d'une incompatibilité de la bibliothèque Boost.

Une des nombreuses erreurs :

    error: use of deleted function 'boost::shared_ptr<ros::Subscriber::Impl>::shared_ptr(const boost::shared_ptr<ros::Subscriber::Impl>&)'

Hélas, il ne semble pas exister de solution viable, en dehors de passer à Ubuntu 14.04 ou de revenir à GCC 4.6.

En réalité GCC 4.6 supporte tout de même quelques fonctionnalités de C++11, avec le flag `-std=c++0x` (il ne supporte pas `-std=c++11`), comme les `array`.

Pour l'optimisation, bien que l'on ne puisse bénéficier de celles spécifiques au Cortex-A7, il est possible d'utiliser les optimisations NEON et celles de l'architecture `armv7-a`. Les flags de compilations utilisés sont alors finalement :

    -std=c++0x -Wall -O3 -mfloat-abi=hard -mfpu=neon-vfpv4 -funsafe-math-optimizations -mcpu=cortex-a7

# Le Robot du CEA
## Dynamixel
**Impératif** : Régler le `return delay` (EEPROM) des servomoteurs à 0 pour permettre une communication rapide.

Cela ne pose pas de problème de fermer la boucle des servomoteurs.

## Performances
- Autonomie : 1 h avec la batterie LiPo 4000 mAh chargée à font, le robot marche en tournant sur lui même
- Hauteur de franchissement : 87 mm (dessous du corps à 100 mm)
- Marche Tripod : 10 sec pour 1 m
- Marche Wave Gait :
    - 1 période = 160 cycles -> 2 min pour 1 m
    - 1 période = 80 cycles -> 1 min 08 our 1 m
- Tous les noeuds sauf joystick intégrés dans le robot : 60 % des 2 coeurs du pcDuino
- Masse du robot complet : 2760 g
- Charge utile max 1500 g

# Markdown
## Conversion Markdown-Html sous Windows
### Avec python-markdown
#### Installation
La distribution Python Anaconda étant installé sur le poste, il est possible d'installer [python-markdown](https://pypi.python.org/pypi/Markdown) en lançant le prompt Anaconda puis :

		pip install markdown

Une fois installée, elle est assez simple d'utilisation et [bien documentée](http://pythonhosted.org//Markdown/index.html).

#### Utilisation
Pour convertir un fichier Markdown en Html, se placer dans le dossier du fichier, maintenir la touche `Maj.` enfoncée en faisant un clic droit. Choisir alors "Ouvrir une fenêtre de commande ici". Il ne reste plus qu'à écrire :

		markdown_py nom_du_fichier_source.md > nom_du_fichier_de_sortie.html

Attention, il y a des soucis d'encodage avec l'unicode. Dans le navigateur, changer l'encodage pour utf-8.

#### Utiliser l'extension TOC
Pour afficher une table des matières, on peut utiliser l'[extension TOC](http://pythonhosted.org//Markdown/extensions/toc.html) incluse dans python-markdown.

Pour cela, il suffit de placer `[TOC]` à l'endroit du fichier où l'on souhaite afficher la table des matières.

Pour générer le fichier Html, il faut alors spécifier l'utilisation de l'extension, dans ce cas, on utilise un autre syntaxe :

		python -m markdown -x markdown.extensions.toc notes_romain.md > notes_romain.html

### Avec grip
Une autre solution est [grip](https://github.com/joeyespo/grip). Il permet de rendre les pages avec le CSS de Github (et de prendre en compte la syntaxe Markdown de Github).

#### Installation
Simplement avec `pip`

		pip install grip

#### Utilisation
Pour exporter le fichier localement dans un fichier Html :

		grip fichier_source.md --export ficher_de_destination.html

## Ascii art dans Sublime Text
Utilisation du plugin [Ascii Decoration](https://github.com/viisual/ASCII-Decorator) disponible dans le Package Control.

## Dictionnaire français dans Sublime Text

- Clôner le dépôt Github : <https://github.com/titoBouzout/Dictionaries>
- Copier les fichiers `French.aff` et `French.dic` dans un nouveau paquet de Sublime Text dans `~/.config/sublime-text-2/Packages/User/Language-French`
- Activer la vérification avec `<F6>` et choisir la langue dans le menu `View -> Dictionnary`.

Source : <http://blog.smarchal.com/correcteur-orthographique-francais-sublime-text/>

# LaTeX
## Glossaire avec makeindex
Dans TeXnicCenter, les arguments à passer à la commande `makeindex` sont les suivants :

	"%tm".glo -t "%tm".glg -s "%tm".ist -o "%tm".gls

Source : <http://tex.stackexchange.com/questions/41360/how-do-i-make-makeindex-work-fully-on-my-winxp-system-works-on-a-different-s>

Problème : Pas de glossaire de généré avec le type `report` ou `scrreprt`.

# Git
## Fin de lignes
### Réglage
Pour configurer git afin qu'il gère des fin de ligne à la mode UNIX :

    $ git config --global core.autocrlf input

À la mode Windows :

    $ git config --global core.autocrlf true

Source : <https://help.github.com/articles/dealing-with-line-endings/>

### Problème avec scripts Python
Un mauvais réglage des fins de ligne peut conduire à des erreurs lors de l'exécution de scripts Python.
En effet, lorsque l'on lance un script comme un exécutable `./mon_script.py` ou par un fichier launch de ROS, c'est la première ligne qui est lue, le **hashbang** :

    #!/usr/bin/env python

Sous Linux, si le fichier est encodé avec des fins de ligne DOS, le `#` est mal lu, le script ne s'exécute pas et la sortie est la suivante :

    : No such file or directory

Source : <http://stackoverflow.com/questions/19764710/python-script-gives-no-such-file-or-directory>

## Quelques commandes utiles
### Comparer deux branches

    $ git diff master..branche

### Comparer un fichier dans deux branches

    $ git diff branche1 branche2 -- fichier.txt

Source : <http://stackoverflow.com/questions/4099742/how-can-i-compare-files-from-two-different-branches>

### Supprimer une branche :

    $ git branch -d ma_branche

Source : <http://makandracards.com/makandra/621-git-delete-a-branch-local-or-remote>

### Obtenir les branches distantes localement
Suite au clonage d'un répertoire, les branches sont bien rapatriées, mais compresées. Pour lister toutes les branches :

    $ git branch -a

Pour obtenir la version locale d'une branche présente sur `origin` :

    $ git checkout -b ma_branche origin/mas_branche

Source : <http://stackoverflow.com/questions/67699/clone-all-remote-branches-with-git>

## Insertion d'un depot Git dans un nouveau avec conservation de l'historique
Le module Kinematics (repris de Martin) a été développé dans `Documents/gitPython`, puis, il a été déplacé dans le paquet ROS dans le workspace catkin. Le problème, c'est que copier-coller le fichier manuellement supprime tout l'historique git. Pour éviter cela, le principe est d'ajouter l'ancien dépôt comme source du nouveau dans une nouvelle branche puis fusionner cette nouvelle branche avec l'ancienne.

        git remote add other ~/Documents/gitPython
        git fetch other
        git checkout -b newBranch other/master
        git mv Kinematics.py scripts
        git commit -m "Ajout du module Kinematics."
        git checkout master
        git merge newBranch
        git commit
        git remote rm other
        git branch -d newBranch

*Remarque : petit soucis avec un double .gitignore lors du merge.*

Source : <http://stackoverflow.com/questions/1683531/how-to-import-existing-git-repository-into-another>

# Robots Hexapodes
## Au sujet des hexapodes
- Avantages des robots hexapodes : <http://hexapodrobots.weebly.com/advantages-of-hexapod-gait.html>

> To maintain the balance only three legs are required, however for walking four are necessary. Additional two legs allow some leeway in walking and increase the reliability of the robot movements.
>
> Construction of the robot’s leg is based on the classical kinematic scheme of the insect. This pattern was chosen for its versatility and possibility to apply for investigations of the gait of both arthropods and reptiles. Moreover, chosen construction enables also to overcome wider range of the obstacles than in case of application of reptile or mammal kinematics.
> -- Six-Legged Robot Gait Analysis, B. Stańczyk  and J. Awrejcewicz

> In fact, simulation tests performed with a dynamic model of hexapod robot revealed that, without postural control, walking efficiency is low and just obstacles significantly shorter than the robot itself can be overcome.(p.2)
>
> Simulation experiments showed that the robot was able to overcome just obstacles significantly shorter than the robot itself,i.e. up to 50% of its center of mass (CoM) height. Main problem was that the robot fell on its back when climbing obstacles. (p.3)
>
> Basically, rear legs are angled posteriorly, [...] Robot performance improved significantly: the robot was now able to overcome obstacles up to 90% of its center of mass; nevertheless, this result is still far from insect performance. (p.3-4)
>
> Leg pairs with different length provide superior agility. (p.4)
>
> with the proposed structure and control architecture the hexapod is now able to successfully overcome obstacles with height up to 140 % of its CoM height. (p.16)
>
> Anyway, just to have an idea of other robot performance, robot RHex [20] is able to negotiate obstacles high 130% of front part height, while Sprawlita [18] is able to negotiate obstacles high 100% of front part height (RHex and Sprawlita are to date among the most efficient six-legged runners). (p.16)
> -- Climbing Obstacle in Bio-robots via CNN and Adaptive Attitude Control, M. Pavone, P. Arena, L. Fortuna, M. Frasca, L. Patane

Définition de la stabilité d'un multipode : McGhee, R. B.: Vehicular legged locomotion, in: Advances in Automation and Robotics, edited by: Saridis, G. N., JAI Press, New York, 1, 259–284, 1985.

- Article des Les Echos sur le robot Toshiba à Fukushima : <http://www.lesechos.fr/21/11/2012/lesechos.fr/0202401576131_toshiba-invente-un-robot-pour-explorer-fukushima.htm>
- Article sur les robots à Fukushima : <http://en.akihabaranews.com/126794/robot/japans-robot-renaissance-fukushimas-silver-lining>
- Blog posts sur l'entrainement avec robots iRobots à Fukushima : <http://spectrum.ieee.org/automaton/robotics/industrial-robots/fukushima-robot-operator-diaries>

## Hexapodes existants
### Hexapodes grand public
#### PhantomX Hexapod MkI

#### PhantomX Hexapod MkII
- Coxa = 52 mm
- Femur = 66 mm
- Tibia = 130 mm

Étude mécanique : <http://www.overware.fr/cours/2013/10/Etude-m%C3%A9canique-robot-PhantomX-AX-Mark-II/?PHPSESSID=iajpip92atagp1p9gq9m80r8o0>

#### Robotis Bioloid Spider

#### Lynxmotion Heaxapod Robot
- Coxa = 20 mm
- Femur = 70 mm
- Tibia = 100 mm

Servomoteurs Hitec HS-475HB, 4,4 kg.cm, 0.23 sec pour 60°.
3,18 mm Lexan

### Hexapodes DIY
#### Crab (ROS)

#### Rhoeby (ROS)

#### TKSPIDER1
Robot de l'article "Design of Six Legged Spider Robot and Evolving Walking Algorithms". 3 DDL par patte.

- Coxa = 75 mm
- Femur = 65 mm
- Tibia = 85 mm

#### HF08
Hexapode à imprimer en 3D, équipé de 18 mini servomoteurs 9g.
Le projet est présenté à cette adresse : <http://www.heliumfrog.com/hf08robot/hf08blog.html>

Les sources pour les pièces 3D, mais aussi le code Arduino et PC sont dispos sur Grabcad : <https://grabcad.com/library/helium-frog-hf08-hexapod-robot>


Il utilise un pseudo-Gcode pour communiquer entre le PC et l'Arduino en USB. Les commandes de déplacement sont envoyées du PC. L'Arduino génère le commandes des servos en calculant la cinématique inverse en interne.

#### Golem
<http://forums.trossenrobotics.com/showthread.php?6725-ROS-Hexapod-project-Golem-MX-64-4dof/page9>

### Hexapodes de recherche
#### Rhex
Par Boston Dynamics. 1 DDL par patte.

#### LauronIII
Par FZI (Forschungszentrum Informatik). 3 DDL par patte, capteur de force 3 axes et capteur de courant sur chaque moteur.

Dr. Karsten Berns

Voir Gaßmann et al. 1991

#### Genghis
Par le MIT, 1 DDL par patte donc 6 moteurs.

#### Snake Monster

#### Tarry I et II
Engineering Mechnacis at the University of Duisberg

- Foot contact switches
- Ultrason à l'avant
- jauges de déformation sur les jambes
- inclinomètre pour le corps

Dr. Martin Frik

Buschmann 2000

## Stack ROS Hexapod
Le concepteur du robot Hexapod [Golem](http://forums.trossenrobotics.com/showthread.php?6725-ROS-Hexapod-project-Golem-MX-64-4dof), KevinO qui sévit sur le forum Trossen Robotics a conçu se robot pour découvrir ROS. Ainsi, il développe toute une pile ROS pour contrôler un hexapode.

Maintenant que la téléopération d'une marche de type Tripod Gait est fonctionnelle, il s'intéresse d'avantage à l'aspect navigation et SLAM.

C'est en partie sur ce code que sera basée la marche en terrain chaotique.

# Divers
## Niveaux de maturité technologique (TRL)
Les niveaux TRL vont de 1 à 9.

- 0 à 3 : LIRM (labo de robotique à Montpellier)
- 3 à 6 : Labo du CEA
- 6 à 9 : CMET

Source : <http://fr.wikipedia.org/wiki/Technology_Readiness_Level>

## Génération de la clé ssh pour Gitlab
La clé générée dans Putty n'est pas reconnue sur Gitlab. À la place, elle a été générée au format unix dans Git Bash avec la commande :

		ssh-keygen -t rsa

## Fonction de pas plus smooth
    period_height = 3*pow( sin( cycle_period_ * PI / CYCLE_LENGTH ), 2 ) - 2*pow( sin( cycle_period_ * PI / CYCLE_LENGTH ), 3 );

Plot : <http://fooplot.com/#W3sidHlwZSI6MCwiZXEiOiIzKnNpbih4KV4yLTIqc2luKHgpXjMiLCJjb2xvciI6IiMwMDAwMDAifSx7InR5cGUiOjAsImVxIjoic2luKHgpIiwiY29sb3IiOiIjMDAwMDAwIn0seyJ0eXBlIjoxMDAwLCJ3aW5kb3ciOlsiLTEuMTcwOTQzMDQwMTE0MTc4IiwiMy41OTc0Mjg1NDE5MTcwNTg2IiwiLTEuNTU5MDIwOTk2MDkzNzQ0MiIsIjMuMjA5MzUwNTg1OTM3NDkyNSJdfV0->

Ce qui s'en rapproche le plus: <https://en.wikipedia.org/wiki/Smoothstep>

# Pour la suite
- Possibilité de RESET de Dynamixel en cas d'erreur (surchauffe) automatique ou au moins service. Ne semble pas possible logiciellement :(
- [Odroid XU4](http://www.hardkernel.com/main/products/prdt_info.php) (pour lequel il existe une [image d'Ubuntu avec ROS pré-installé](http://forum.odroid.com/viewtopic.php?f=95&t=7446)) ou équivalent, voir même plutot mini PC genre NUC avec processeur Intel car plus de support logiciel sur x86.
- Manette Bluetooth pour pouvoir tout embarquer
- La forme des pieds pour éviter qu'ils ne s'accrochent sur les côtés et que le capteur ne se déclenche pas, exemple l'hexapode de RoMeLa : <http://www.romela.org/main/File:MARS2.jpg>

![Hexapode de RoMeLa](http://www.romela.org/wiki/images/thumb/4/46/MARS2.jpg/800px-MARS2.jpg "Hexapode de RoMeLa")

- Publier à un taux très lent la tension de la batterie en lisant la valeur sur un Dynamixel pour éviter la décharge trop importante de la LiPo et pourquoi pas faire un tableau de bord un peu comme sur [le Kobuki](http://wiki.ros.org/kobuki_dashboard).

![Tableau de bord Kobuki](http://wiki.ros.org/kobuki_dashboard?action=AttachFile&do=get&target=kobuki_dashboard.png "Tableau de bord Kobuki")

- Pour les seuils, faire la moyenne, écretter (retirer les quelques valeurs max et min) puis multiplier par coefficient de sécurité
- Faire en sorte de pouvoir appliquer le soulèvement du corps à la manette en wave gait (`body_z_override`)
- Utiliser `dynamic_reconfigure` pour régler les gains de compliance des servos à la volée
- Tester plus amplement pour voir si le problème `std::bad_alloc` persiste (voir paragraphe [Erreur bad_alloc](#erreur-bad_alloc))
- 1 cm de différence entre hauteur du corps/pattes réel et software
- Dans `hexapod_dynamixel` et `hexapod_force_feedback`, n'afficher des erreurs que s'il y en a plusieurs consécutivement. Il est "normal" d'avoir quelques erreurs de temps en temps, mais pas tout le temps.
