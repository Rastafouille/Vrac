Sommaire 

- [Réseau](#reseau)
- [Git](#git)
- [Arduino](#arduino)
- [Sublime text](#sublime-text)
- [Markdown](#markdown)
- [ROS](#ros)
- [Linux](#linux)
- [Divers](#divers)

# RESEAU
### SSH
Connexion
```bat
ssh odroid@192.168.101.101
```
Transfert de fichier ssh
```bat 
scp /home/lstd/Bureau/en.mkv odroid@192.168.101.101:/home/odroid/Desktop/
```
Transfert de dossier ssh
```bat
scp -r /home/mickael/data/ odroid@192.168.101.101:/var/www/
```
### Traffic sur une connexion
```bat
	sudo vnstsat
	sudo vnstat -i  wlan0 -l
```
### Synchronisation de l'heure avec Chrony entre 2 pc distants

```bat
	sudo apt-get install chrony
	sudo gedit /etc/chrony/chrony.conf
```
sur le master rajouter :
```bash
	# make it serve time even if it is not synced (as it can't reach out)
	local stratum 8
	# allow the IP of your peer to connect (subnet not specific IP)
	allow 192.168.XX
```
sur le client rajouter :
```bash
	# set the servers IP here to sync to it
	server <IP> iburst
	# remove the default servers in the config
```
```bat
	systemctl restart chrony
```
### Changer message d'accueil ubuntu
Il s'agit du message informatif afficher au début d'une connexion SSh par exemple.

Créer le fichier `/etc/motd.tail` et y inscrire le message souhaité.

Exemple :

		* blablabla, lancer motion pour le stream video

Source : <http://serverfault.com/questions/407033/how-can-i-edit-the-welcome-message-when-ssh-start>

### Stream video avec Motion

 <https://motion-project.github.io/motion_config.html>
```bat
	sudo apt-get install motion
	mkdir ~/.motion
	nano ~/.motion/motion.conf
```
mettre :
```bash
videodevice /dev/video7
stream_port 8080
stream_localhost off
output_pictures off
framerate 30
width 640
height 480
stream_quality 50 #up to 100
stream_maxrate 5 #up to 100
ffmpeg_video_codec mpeg4
auto_brightness off
```
lancer
```bat
	motion
```
reboot si necessaire et <http://192.168.101.101:8080/>
possibilité de le mettre en service pour lancement au démarrage
 
# GIT
dans le repertoire
`git pull`
faire les modifs
`git add .` ou `git add fichier.extension`
`git status` pour verifier
`git commit -m "commentaire"`
`git push`

### Annuler les modifications non committées

    $ git checkout -f

# ARDUINO
### probleme sur Ubuntu
`processing.app.SerialException: Error opening serial port '/dev/ttyACM0'`
go to File-> Preferences and at the bottom of the popup, they give you a link for the 'preferences.txt' file. Click that and it'll open the text. MAKE SURE AT THIS POINT TO CLOSE YOUR ARDUINO IDE. Make the change to '9600' Serial debug rate and save and exit. Open up your IDE and it should work now. 

# SUBLIME-TEXT
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

# MARKDOWN
 Dans Tools / Command Palette :

	install package
 Puis installer :

	Markdown Preview
	Markdown extended
	Monokai extended
# ROS
La gestion des paquets se fait avec `rospack`. Pour mettre à jour l'index des paquets :

    $ rospack profile

Déplacement dans le répertoire d'un paquet :

    $ roscd mon_paquet
    
    
# Linux

affichage des variables d'environnement

	$ printenv | grep ROS
	
La liste des processus lancés par l'utilisateur dans le terminal :

    $ ps -a

Tuer un processus avec son `pid` :

    $ kill -9 pid


# DIVERS

### Screen
Donner un nom
	
	$ screen -S nom_du_screen

Donner une nom et lancer une commande
	
	$ screen -S <nom_du_screen> <commande_qui_lance_le_programme>

Lister les screens

	$ screen -r
	
Lancer un screen

	$ screen -r <nom ou id>

Pour fermer

	$ exit

ctrl+d fermer la fenetre au le screen si une seul fenetre

ctrl+a puis c pour ouvrir d'autre terminaux dans le meme screen

### Clavier numerique au demarrage
créer un fichier (s'il n'existe pas) `/etc/lightdm/lightdm.conf.d/20-lubuntu.conf`
et y mettre :
		
	[SeatDefaults]
	greeter-setup-script=/usr/bin/numlockx on

source : <http://doc.ubuntu-fr.org/numlockx>
il faut prélablement avoir installer numlockx avec apt

### Wiimote bluetooth
<http://www.raspberrypi-spy.co.uk/2013/02/nintendo-wii-remote-python-and-the-raspberry-pi/>
<http://linux.arcticdesign.fr/commande-bluetooth-laide-du-pyhton/>

pour activer le bluetooth sur le pc H@ri, a chaque connexion du dongle :

	$ sudo hciconfig hci0 reset`

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
	
	$ sudo apt-get purge bumblebee* nvidia*

reinstall nvidia 331 
(the only driver that apparently works correctly with my video card)
	
	$ sudo apt-get install nvidia-331 nvidia-settings nvidia-prime

restart

	$ sudo reboot

