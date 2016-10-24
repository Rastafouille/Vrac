# Installation de ROS Hydro sur un pcDuino
Source : <http://wiki.ros.org/hydro/Installation/UbuntuARM>

## Configuration des dépots Ubuntu
Il faut ajouter les dépots *restricted*, *universe* et *multiverse*.

Après les avoir ajoutés, `apt` indique des doublons, donc je les ai commentés dans `/etc/apt/sources.list`. Affaire à suivre...

## Régler les locales
Boost et ROS requièrent que les locales système soient définies (pas en français, sino ça cause des problèmes).

		$ sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGE=POSIX

## Ajout des dépots de ROS
Réglage pour ajouter les dépots de ROS ARM à Ubuntu :

		$ sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros precise main" > /etc/apt/sources.list.d/ros-latest.list'

## Réglage des clés
		$ wget http://packages.namniart.com/repos/namniart.key -O - | sudo apt-key add -

## Installation
Mise à jour de la liste des paquets :

		$ sudo apt-get update

Installation de la base de ROS (on ne peut pas installer tout automatiquement car tous les paquets ROS ne sont pas disponibles pour ARM).

		$ sudo apt-get install ros-hydro-ros-base

Pour installer un paquet individuellement :

		$ sudo apt-get install ros-hydro-PACKAGE

Pour voir la liste des paquets disponible :

		$ apt-cache search ros-hydro

Taille de paquets ROS :

- ros-hydro-ros : 364 Mo
- ros-hydro-pcl-ros : 850 Mo
- ros-hydro-navigation 891 Mo

## Initialisation de rosdep
`rosdep` doit déjà etre installé. Si ce n'est pas le cas :

		$ sudo apt-get install python-rosdep

		$ sudo rosdep init
		$ rosdep update

## Réglage de l'environnement
Chargement automatique des variables d'environnement ROS :

		$ echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
		$ source ~/.bashrc

## Installation de rosinstall
`rosinstall` est un utilitaire indépendant de la distribution ROS mais utile pour installer des paquets.

		$ sudo apt-get install python-rosinstall

## Modifier le nom de l'OS
Ubuntu ARM est défini comme *Linaro*, mais ROS ne reconnait que Ubuntu. Pour cela, mofifier le fichier `/etc/lsb-release` pour qu'il contienne :

		DISTRIB_ID=Ubuntu
		DISTRIB_RELEASE=12.04
		DISTRIB_CODENAME=precise
		DISTRIB_DESCRIPTION="Ubuntu 12.04"

## Test de l'installation
Lancement du master

		$ roscore

Liste des topics

		$ rostopic list

*Remarque : Après l'installation de ROS et la désinstallation de xbmc, scratch, abiword et gnumeric, il reste 1,7 Go de libre sur les 4 Go du pcDuino.*

# Création du Workspace
Création d'un dossier de workspace ROS

		$ mkdir -p ~/catkin_ws/src
		$ cd ~/catkin_ws/src
		$ catkin_init_workspace
		$ cd ../
		$ catkin_make

Ajout du script des variables du workspace au `bashrc` 

		$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
		$ source .bashrc 

# Installation de paquet Dynamixel de HumaRobotics
## Installation de la bibliothèque Python
Il faut que les logiciels suivants soient installés :

		$ sudo apt-get install git python2.7 python-serial python-tk

Clonage du dépot Github

		$ cd ../../Documents/
		$ git clone https://github.com/HumaRobotics/dynamixel_hr.git
		$ cd dynamixel_hr/

Ajout de l'utilisateur au groupe `dialout` pour lui permettre d'accéder aux ports série

		$ sudo usermod -a -G dialout ubuntu

Installation de la bibliothèque

		$ sudo python setup.py install

Test de l'installation avec l'utilisateur `ToolDynamixelLab`

		$ python ToolDynamixelLab.py 

## Installation du paquet ROS
Clonage du dépot Github

		$ cd ~/catkin_ws/src/
		$ git clone https://github.com/HumaRobotics/dynamixel_hr_ros.git
		$ cd dynamixel_hr_ros/

Compilation du paquet

		$ rosmake
		$ cd ../../
		$ catkin_make

Test de l'instalaltion en lançant le noeud `/dxl`

		$ rosrun dynamixel_hr_ros expose.py --device /dev/ttyUSB0 --baudrate 1000000 --rate 10

Dans un nouveau terminal, lancer l'enregistrement d'un mouvement pendant 5 secondes

		$ rosrun dynamixel_hr_ros record.py 

Puis rejeu de ce mouvement

		$ rosrun dynamixel_hr_ros replay.py 

Désactivation de la chaine de servos

		$ rostopic pub -1 /dxl/enable std_msgs/Bool -- 'false'

