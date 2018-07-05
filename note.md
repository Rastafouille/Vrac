
## installation opencv +python3 ubuntu18.04

`sudo apt-get update`

`sudo apt-get upgrade`

`sudo apt install python3`

`sudo apt-get install build-essential cmake unzip pkg-config libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran`

`sudo apt-get install python3-dev`

`wget https://bootstrap.pypa.io/get-pip.py`

`sudo python3 get-pip.py`

`sudo pip install spyder opencv-python opencv-contrib-python`

## installation ROS
<http://www.instructables.com/id/Ros-Indigo-install-on-Ubuntu-1/>

j'ai enregistrer lenvironnement dans Bashrc, mais a chaque shell
`source ~/.bashrc`

After you put the who rplidar_ros into the catkin_ws and compile, you will need to do one command to source the folder
`source ~/catkin_ws/devel/setup.bash`



## Wiimote bluetooth
<http://www.raspberrypi-spy.co.uk/2013/02/nintendo-wii-remote-python-and-the-raspberry-pi/>
<http://linux.arcticdesign.fr/commande-bluetooth-laide-du-pyhton/>

pour activer le bluetooth sur le pc H@ri, a chaque connexion du dongle :
`sudo hciconfig hci0 reset`


## GIT HUB 

dans le repertoire
`git pull`
faire les modifs
`git add .` ou `git add fichier.extension`
`git status` pour verifier
`git commit -m "commentaire"`
`git push`


##Arduino
### probleme sur Ubuntu
`processing.app.SerialException: Error opening serial port '/dev/ttyACM0'`
go to File-> Preferences and at the bottom of the popup, they give you a link for the 'preferences.txt' file. Click that and it'll open the text. MAKE SURE AT THIS POINT TO CLOSE YOUR ARDUINO IDE. Make the change to '9600' Serial debug rate and save and exit. Open up your IDE and it should work now. 

## Sublime Text
### installation
	sudo add-apt-repository ppa:webupd8team/sublime-text-3
	sudo apt-get update
	sudo apt-get install sublime-text-installer
### Package Control
Dans View / Show Console :

	import urllib.request,os,hashlib; 
	h = 'eb2297e1a458f27d836c04bb0cbaf282' + 'd0e7a3098092775ccb37ca9d6b2e4b7d'; 
	pf = 'Package Control.sublime-package'; 
	ipp = sublime.installed_packages_path(); 
	urllib.request.install_opener( urllib.request.build_opener( urllib.request.ProxyHandler()) ); 
	by = urllib.request.urlopen( 'http://packagecontrol.io/' + pf.replace(' ', '%20')).read(); 
	dh = hashlib.sha256(by).hexdigest(); 
	print('Error validating download (got %s instead of %s), please try manual install' % (dh, h)) if dh != h else open(os.path.join( ipp, pf), 'wb' ).write(by) 

### MarkDown
 Dans Tools / Command Palette :

	install package
 Puis installer :

	Markdown Preview
	Markdown extended
	Monokai extended

## clavier numerique au demarrage
créer un fichier (s'il n'existe pas) `/etc/lightdm/lightdm.conf.d/20-lubuntu.conf`
et y mettre :
		
	[SeatDefaults]
	greeter-setup-script=/usr/bin/numlockx on

source : <http://doc.ubuntu-fr.org/numlockx>
il faut prélablement avoir installer numlockx en apt-get

## Changer message d'accueil ubuntu
Il s'agit du message informatif afficher au début d'une connexion SSh par exemple.

Créer le fichier `/etc/motd.tail` et y inscrire le message souhaité.

Exemple :

		* Penser a regler la date.
			 - Manuellement :
					 sudo date MMJJHHmmAAAA
			 - Ou plus precisement :
					 sudo ntpdate 192.168.0.103

Source : <http://serverfault.com/questions/407033/how-can-i-edit-the-welcome-message-when-ssh-start>

## Config video DELL E6530
uninstall bumblebee and nvidia drivers
	
	sudo apt-get purge bumblebee* nvidia*

reinstall nvidia 331 
(the only driver that apparently works correctly with my video card)
	
	sudo apt-get install nvidia-331 nvidia-settings nvidia-prime

restart
	
	sudo reboot

