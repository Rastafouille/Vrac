##installation ROS
<http://www.instructables.com/id/Ros-Indigo-install-on-Ubuntu-1/>

j'ai enregistrer lenvironnement dans Bashrc, mais a chaque shell
`source ~/.bashrc`

After you put the who rplidar_ros into the catkin_ws and compile, you will need to do one command to source the folder
`source ~/catkin_ws/devel/setup.bash`



##Wiimote bluetooth
<http://www.raspberrypi-spy.co.uk/2013/02/nintendo-wii-remote-python-and-the-raspberry-pi/>
<http://linux.arcticdesign.fr/commande-bluetooth-laide-du-pyhton/>

pour activer le bluetooth sur le pc H@ri, a chaque connexion du dongle :
`sudo hciconfig hci0 reset`


##GIT HUB 
Rastafouille/Bibiche0

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
