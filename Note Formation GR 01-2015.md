Notes Formation Gazebo/Rviz sur Darwin et Hexapode PhantomX de Janvier 2015

# INITIALISATION

## INSTALL DEPENDENCIES

    sudo apt-get install git
    sudo apt-get install ros-hydro-desktop-full
    sudo apt-get install ros-hydro-gazebo-plugins
    sudo apt-get install ros-hydro-gazebo-ros
    sudo apt-get install ros-hydro-gazebo-ros-control
    sudo apt-get install ros-hydro-hector-gazebo
    sudo apt-get install ros-hydro-hector-gazebo-plugins
    sudo apt-get install ros-hydro-effort-controllers
    sudo apt-get install ros-hydro-joint-state-controller
    sudo apt-get install ros-hydro-joint-state-publisher
    sudo apt-get install ros-hydro-turtlebot-teleop`

## CREATE WORKSPACE
    cd ~
    mkdir -p ros-gazebo/src
    cd ros-gazebo/src
    source /opt/ros/hydro/setup.bash
    catkin_init_workspace
    cd ..
    catkin_make

## AJOUTER ROSBASH POUR LES RACCOURCIS

`git clone git@dev.humarobotics.com:rosbash`
modifier `/home/"utilisateur"/.bashrc`
ajouter: `source ~/rosbash/rosbash.bash`
redémarrer les shells
`devel` pour initialiser
`rkill`, `rosrun`, `roslaunch`

## GIT
Pour le premier
`git init .
`git add *
`git commit` (`-a` pour pas faire le `git add *`)
`git remote add origin git@dev.humarobotics.com:tourelle_description` (apres un git clone pas besoin, il crée automatiquement un origin)
`git push origin master`
pour push
    `git add file` si nouveau fichier
    `git commit`
    `git push origin master`
pour pull
`git pull origin master`
`git status` pour voir s'il y a des modifs entre dernier pull ou commit avec tes modifs en cours, et de nouveaux fichiers
`git merge` pour comparer des modifs de ton commit avec un version en cours
`git diff` montrer les modif entre copie de travail et dernier commit
`git log` montre tout l'historique des commit
dossier .git cacher avec tout de stocké
`git checkout xxxxx` pour charger un ancien commit et git chaeckout master pour recharger le dernier commit

## CLONE REPOSITORIES
    cd ros-gazebo/src
    git clone git@dev.humarobotics.com:darwin_description
    git clone git@dev.humarobotics.com:darwin_gazebo
    git clone git@dev.humarobotics.com:darwin_control
    git clone git@dev.humarobotics.com:phantomx_description
    git clone git@dev.humarobotics.com:phantomx_control
    git clone git@dev.humarobotics.com:phantomx_gazebo
    git clone git@dev.humarobotics.com:tourelle_description
 
## LAUNCHERS
`gazebo` pour lancer gazebo hors ROS
OU
`roscore` dans un shell pour lancer gazebo ROS
`rosrun gazebo_ros gazebo` dans un autre shell

`roslaunch darwin_gazebo darwin_gazebo.launch` pour ouvrir un modele

`rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/darwin/cmd_vel`
 

# CREATION MODELE 

`.sdf` spécifique Gazebo et `.urdf` pour ROS mais Gazebo les gere aussi
créer dossier `tourelle_description`
importer le `.stl` dedans
modifier avec blender, echelle, position, orientation, origine
pour echelle et orientaiton, modifier directement les parametres dans la table de droite
modife origine :
- en bas, edit mode
- touche "C" pour selection
- "shift + S" puis "cursor to selected", le curseur se met sur le barycentre des points selectionnés
- en bas, objet mode
- "origin to 3D cursor"
- mettre position a 0 0 0
modif du zoom, tip "N" et passer le start clip a 0.001 dans la table de droite
file/export/STL pour enregister
si probleme d'affichage dans meshlab : "Normalize Face Normals"

# MODELE DANS GAZEBO 
##URDF
définition des links et joins dans le fichier URDF
visualisation URDF dans RVIZ pour validation de l'assemblage
`roslaunch urdf_tutorial display.launch model:=tourelle.urdf gui:=true`

creer tourelle_gazebo.launch pour charger le world et le .urdf et le controleur (control.launch) :       
Exposer le model dans ROS : creation du fichier control.LAUNCH qui appelle un autre fichier de config .YAML
Ajouter les transmissions dans URDF
Créer les fichiers de contrôle:
- `tourelle_control.yaml`
- `tourelle_control.launch`

## PHANTOM + TOURELLE AVEC XACRO 
xacro pour la generation automatique urdf ou la defetion de macro
il nous faut une xacro pour definir un macro de creation tourelle, une idem pour phantomX et une pour faire un urdf d'assemblage.
On peut faire des xacro de creation urdf pour plusieurs types d'assemblages, Tourelle+Baselink, PhantomX+Base link ou PhantomX+tourelle
pour lancer la generation de l'urdf a partir du xacro : `rosrun xacro xacro phantomx.xacro >phantomx.urdf`

## AJOUT CAPTEURS
dans l'URDF tag gazebo, en 2 parties, Sensor et plugin

#RVIZ
`rosun rviz rviz` une fois le ros_gazebo lancé
charge le robot model avec `add RobotModel`
`add display` et le topic qui va bien : `Add Image`

## TF
permet de regarder chacun de repere, les matrices de transformation entres eux
- `Add Tf`
- `rosrun tf tf-echo`
- `rosrun static_transform_publsher`
creer  fichier pdf des liens à partir du fichier urdf
on se place dans le dossier urdf
`urdf_to_graphiz phantomx_tourelle.irdf`
       
# SOME COMMANDS
- Refresh packages in ws (if you cannot find it)
    `rospack profile`
    # from workspace root

- Visualize URDF
`roslaunch urdf_tutorial display.launch model:=phantomx.urdf gui:=true`

- Generate URDF from Xacro
`rosrun xacro xacro phantomx.xacro >phantomx.urdf`
 
- Launch Darwin Gazebo
`roslaunch darwin_gazebo darwin_gazebo.launch`
 
- Launch PhantomX Gazebo
`roslaunch phantomx_gazebo phantomx_gazebo.launch`

- Teleoperation on PhantomX
`rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/phantomx/cmd_vel`

- Teleoperation on Darwin
`rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/darwin/cmd_vel`

- pour faire un pdf graphique avec l'arbre
`urdf_to_graphiz files.urdf`

- PROBLEME AFFICHAGE RVIZ:
`export LIBGL_ALWAYS_SOFTWARE=1 et le mettre dans le bashrc dans home/user`


# CONFIG SCITE 
## GENERAL CONFIGURATION
    tabsize=4
    indent.size=4
    use.tabs=0

    # open files in the last directory:
    open.dialog.in.file.directory=1
    check.if.already.open=1

    # show all files:
    open.filter=$(all.files)Source Files (cpp,c,h,mak)|*.cpp;*.c;*.h;*.mak;makefile|Web Files (htm, html, css, xml, shtml, js, pl, asp)|*.html;*.htm;*.css;*.shtml;*.js;*.pl;*.xml;*.asp|Text (txt, doc)|*.txt;*.doc|

    # show status bar:
    statusbar.visible=1
    title.full.path=1
    line.margin.visible=1

    # Set monospaced font:
    font.base=$(font.monospace)
    font.small=$(font.monospace)
    font.comment=$(font.monospace)
    font.text=$(font.monospace)
    font.text.comment=$(font.monospace)
    font.embedded.base=$(font.monospace)
    font.embedded.comment=$(font.monospace)
    font.vbs=$(font.monospace)

    # Wrapping of long lines
    wrap=1

    # prompt me for any suspicous action
    are.you.sure.on.reload=1
    load.on.activate=1
    highlight.current.word=1
    #highlight.current.word.by.style=1
    highlight.current.word.colour=#00D040
    selection.back=#F00000
    command.go.*.py=python -u "$(FileNameExt)" $(1)
    file.patterns.cpp=$(file.patterns.cpp);*.nxc;*.cpp;*.h;*.cc

     

 

#Probleme carte graphique
    sudo apt-get --purge remove nvidia-current
    sudo add-apt-repository ppa:ubuntu-x-swat/x-updates
    sudo add-apt-repository ppa:bumblebee/stable
    sudo apt-get update
    sudo apt-get install bumblebee bumblebee-nvidia linux-headers-generic
