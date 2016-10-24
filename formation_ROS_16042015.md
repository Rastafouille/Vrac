# Formation ROS du 16/04/2015

[TOC]

## Python
### Commentaires UTF-8
La ligne de spécification de l'encodage utf-8 permet d'écrire des caractères accentués dans les commentaires.

    # -*- coding: utf-8 -*-

La ligne suivante, plus simple, fonctionne également ([source](http://sametmax.com/un-header-dencoding-plus-simple-pour-python/)) :

    # coding: utf-8

## Git
### Annuler les modifications non committées

    $ git checkout -f

### Interface graphique
gitk, gitg

## ROS
### Workspace

Création du workspace :

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace

Construire :

    $ cd ..
    $ catkin_make

Sourcer :

    $ source devel/setup.bash

Pour simplifier l'utilisation, il est possible d'ajouter cette ligne à la fin du `.bashrc` mais ce n'est pas la manière la plus propre si on utilise plusireurs workspaces. Voir script bash de Philippe Capdepuy.

### Paquets
#### Création
Créer un pacquet :

    $ catkin_create_pkg nom_du_paquet dependance1 dependance2

Exemple :

    $ catkin_create_pkg turtle_goal rospy std_msgs geometry_msgs turtlesim

#### Organisation
Les scripts Python executables sont à mettre dans le dossier `scripts`. Ne pas oublier le shebang au début de chaque script exécutable :

    #!/usr/bin/env python

Et de rendre le script exécutable :

    $ chmod +x mon_script.py 

On place dans le dossier `src` les scripts qui sont des modules (non exécutables) et qui peuvent importés de l'extérieur du paquet. Il faut alors les placer dans un dossier portant le même nom que le paquet et ajouter un fichier vide `__init__.py` dans ce dossier. Dans le `CMakeLists.txt`, il faut alors ajouter (ou plutôt décommenter) la ligne suivante :

    catkin_python_setup()

Enfin, il faut créer un fichier `setup.py` à la racine du paquet ([source](http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile#Installing_scripts_and_exporting_modules), contenant :

    #!/usr/bin/env python
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup()
    d['packages'] = ['nom_du_paquet']
    d['package_dir'] = {'': 'src'}

    setup(**d)

#### Outils ROS
La gestion des paquets se fait avec `rospack`. Pour mettre à jour l'index des paquets :

    $ rospack profile

Déplacement dans le répertoire d'un paquet :

    $ roscd mon_paquet

#### Python
##### Classes
Il est intéressant d'utiliser une classe pour coder un noeud. Ainsi, tous les *Publishers* et *Subscribers* sont créés en même temps lors de l'appel du constructeur, ce qui permet d'éviter des problèmes de timing. De plus, lors de la partage d'une ressource entre plusieurs méthodes comme une communication série avec `pyserial`, cela permet de la définir comme un attribut de la classe.

##### Mutex
La liaison série ne peut être utilisée que par un processus à la fois. Dans un noeud ROS, ayant un *Publisher*, l'ajout d'un *Subscriber* conduit à une communication asynchrone. Pour éviter des conflits, il faut alors utiliser des mutex pour bloquer l'accès au port série à une seule méthode à la fois. Pour cela, on importe la méthode `Lock` du module `threading` :

    from threading import Lock

Dans chaque methode, il faut alors utiliser le mot clé `with` :

    def __init__(self):
        self.lock = Lock()

    def lecture(self):
        with self.lock:
            # code de lecture du port serie

    def ecriture(self):
        with self.lock():
            # code d'ecriture sur le port serie

### Topics
Lors de la création d'un *Publisher* ou *Subscriber*, il est possible (et même vivement conseillé depuis ROS Indigo) de spécifier la taille de la file (*queue*) avec le paramètre `queue_size`. Dans le cas du *Publisher*, cela dépend des latences de communication entre les noeuds, alors que pour le *Subscriber*, cela dépend du temps de traitement lors de la réception d'un message. Une file trop grande pour le *Subscriber* peut engendrer des retards dans le traitement, mais garantit de traiter tous les messages. Dans le cas de capteurs publiant régulièrement leur état, une petite taille de file (1) paraît suffisant car garantit de toujours obtenir la dernière mesure. 

### Services
#### Ligne de commande
- Liste des services

        $ rosservice list

- Informations sur un service :

        $ rosservice info /turtle1/teleport_absolute

- Appel d'un service sans arguments :

        $ rosservice call /clear

- Appel d'un service avec arguments :

        $ rosservice call /turtle1/teleport_absolute 5 5 0

#### Python
Sans arguments :

    from std_srvs.srv import Empty
    rospy.wait_for_service("/clear")
    clear = rospy.ServiceProxy("/clear", Empty)
    clear()

Avec des arguments :

    from turtlesim.srv import TeleportAbsolute
    rospy.wait_for_service("/turtle1/teleport_absolute")
    teleport = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)
    teleport(5, 5, 0)

#### Créer un service
##### Fichier srv
Créer un fichier `.srv` dans un dossier `srv` à la racine du paquet.

    float32 x
    float32 x
    ---

##### CMakeLists
Il faut ajouter la dépendance à `genmsg` :

    find_package(catkin REQUIRED COMPONENTS
        [...]
        genmsg
    )

Décommenter la partie sur les fichiers de service :

    add_service_files(
      FILES
      Goal.srv
    )

Et la partie de génération des services :

    generate_messages(
       DEPENDENCIES
       geometry_msgs
       std_msgs
    )

#### Python
Il faut commencer par importer notre nouveau service :

    from turtle_goal import Goal, GoalRequest

*Remarque : le `GoalRequest` est généré automatiquement.*

On crée le service à la manière d'un subscriber :

    self.service=rospy.Service("/turtle1/goal", Goal, self.cb_service)

Et maintenant, on implémente la fonction appelée par le service :

    def cb_service(self, req):
        self.go(req.x, req.y)
        return GoalResponse()

On peut alors tester avec :

    rosservice call /turtle1/goal 2 7

Attention, il faut gérer la concurrence des appels aux services nous-même.

### Paramètres
#### Ligne de commande
    $ rosparam list

#### Python

    r = rospy.get_paral("/background")
    rospy.set_param("/background_r", 0)

### Boucles while
Dans les boucles `while`, toujours ajouter une condition sur le bon fonctionnement du noeud :

    while not rospy.is_shutdown()

Dans ce cas, l'interruption par un `Ctrl+c` est bien prise en compte et le script s'arrête. Sinon, il reste dans la boucle while.

### Divers
Pour visualiser un message de type `sensor_msgs/Imu`, il faut installer un plugin RViz :

    sudo apt-get install ros-indogo-rviz-imu-plugin

## Arduino
### Réception de commande série
Pour traiter des commandes du type `avance` à partir de la liaison série, on peut utiliser le code suivant :

    if(strcmp(commande, "avance") == 0) {
        [...]
    }

Pour traiter des commandes du type `avance 153` à partir de la liaison série, on peut utiliser le code suivant :

    if(strncmp(commande, "avance", 6) == 0) {
        int v = atoi(commande+7);
        [...]
    }

Côté Python, pour lire des données avec pyserial.

    line = ser.readline()
    # line="128 238\n"
    trame = line.strip() # Supprime le '\n' a la fin
    # trame = "128 238"
    values = trame.split(' ') # separe les donnees separees par un espace et les stocke dans une liste
    # values = ['128','238']

## Linux
### Les processus
La liste des processus lancés par l'utilisateur dans le terminal :

    ps -a

Tuer un processus avec son `pid` :

    kill -9 pid

### PS1 ROS
Crée un joli PS1 indiquant la distribution ROS actuellement chargée et l'adresse IP du MASTER :

    PS1=\[\033[0;31m\]ROS \[\033[0;34m\]$ROS_DISTRO\[\033[0;32m\]@$MASTER\[\033[0;m\]:\[\033[0;36m\]\w\[\033[0;m\]>

Mais cela demande la définition de la variable `$MASTER`. Par exemple dans le `.bashrc` :

    export MASTER=localhost
    export ROS_MASTER_URI=http://$MASTER:11311

### Recherche de fichiers
Recherche de tous les fichiers Python dans le répertoire courant :

    find . -name "*.py"

### Redémarrer le service réseau

    sudo service networking restart

## Vim
### Insérer un fichier

    :r /path/to/my/file.py

### Insérer certaines lignes d'un fichier
En utilisant `sed` comme utilitaire externe :

    :r! sed -n 38p path/to/my/file.py

Ou en pure Vimscript :

    :put =readfile('/path/to/foo/foo.c')[146:226]
