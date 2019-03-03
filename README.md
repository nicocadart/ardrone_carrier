# ardrone_carrier

ROS project for an ardrone landing on mobile platform, for course ROB314 at ENSTA ParisTech.

Autors : N. Cadart, B. Sarthou, P. Antoine

Date : September 2018 - March 2019

- [ardrone_carrier](#ardrone_carrier)
  - [Description du projet](#description-du-projet)
  - [Usage](#usage)
  - [Descriptions des fichiers](#descriptions-des-fichiers)
  - [Ar Track Alvar](#ar-track-alvar)
    - [Mode d'emploi](#mode-demploi)
    - [Observation des détections individuelles et *tf*](#observation-des-détections-individuelles-et-tf)
    - [Observation des détections multiples (*bundles*)](#observation-des-détections-multiples-bundles)
  - [Tum Ardrone](#tum-ardrone)
  - [Calibration de la caméra](#calibration-de-la-caméra)

## Description du projet

Ce package ROS a pour but d'utiliser un drone Parrot Ardrone de manière autonome afin de décoller, voler, détecter une cible, la suivre, puis atterrir dessus.

L'interface avec le drone est faite au moyen de [ardrone_autonomy](https://ardrone-autonomy.readthedocs.io/en/latest/). La cible est assimilée à un ensemble de QR codes, détectés par [ar_track_alvar](http://wiki.ros.org/ar_track_alvar).

## Usage

Pour lancer l'interface de communication et de commande du drone, ainsi que la correction des TF erronées :
```
roslaunch ardrone_carrier ardrone_driver.launch
```

Pour lancer la détection de la cible par **ar_track_alvar**
```
roslaunch ardrone_carrier track_alvar_ardrone_bundles.launch
```

Pour lancer l'aperçu avec RViz :
```
roslaunch ardrone_carrier rviz.launch
```

## Descriptions des fichiers

- `config/` : répertoire contenant les fichiers de calibrations des caméras (Ardrone 1 et 2), ainsi que la configuration de RViz pour ce projet.
- `doc/` : répertoire contenant des images pour la documentation ainsi que la définition du bundle utilisé comme cible.
- `launch/` : ROS launch files pour lancer les différents noeuds utilisés
- `msg/` : ROS msgs définis dans ce package
- `src/` : répertoire contenant le code source des noeuds développés pour ce projet.


## Ar Track Alvar

### Mode d'emploi

Pour lancer la détection des QR codes avec ar_track_alvar :
1. créer un répertoire "camera_info" dans les fichiers cachés de ROS : `mkdir ~/.ros/camera_info`
2. copier dans ce répertoire les fichiers de calibration des caméras du drone : `cp <ardrone_carrier>/config/ardrone_front.yaml ~/.ros/camera_info/ardrone_front.yaml` et `cp <ardrone_carrier>/config/ardrone_bottom.yaml ~/.ros/camera_info/ardrone_bottom.yaml`
3. lancer le driver du drone : `roslaunch ardrone_carrier ardrone_driver.launch`
4. lancer ar_track_alvar : `roslaunch ardrone_carrier track_alvar_ardrone.launch` ou `roslaunch ardrone_carrier track_alvar_ardrone_bundles.launch`
5. pour visualiser les détections, lancer `rviz` et ajouter les "Marker" correspondant au topic "/visualization_marker"

![individual detections using ar_track_alvar](doc/imgs/ar_track_alvar_individual.png)


### Observation des détections individuelles et *tf*

Launchfile à utiliser : `roslaunch ardrone_carrier track_alvar_ardrone.launch`.
Lorsque détectés, les markers sont publiés sous forme de liste dans le topic `/ar_pose_marker`. On peut passer en paramètre "*output_frame*" du noeud ar_track_alvar le frame_id à utiliser comme référence : les positions des markers seront alors dans cette référence. Exemple d'un message repérant les markers 8 et 6, et publiant leur position dans la référence `/ardrone_base_link`
```
header:
  seq: 2685
  stamp:
    secs: 0
    nsecs:         0
  frame_id: ''
markers:
  -
    header:
      seq: 0
      stamp:
        secs: 1542361017
        nsecs: 764956265
      frame_id: "/ardrone_base_link"
    id: 8
    confidence: 0
    pose:
      header:
        seq: 0
        stamp:
          secs: 0
          nsecs:         0
        frame_id: ''
      pose:
        position:
          x: 1.34920739926
          y: 0.357020739996
          z: -0.114796319112
        orientation:
          x: 0.576315838851
          y: -0.243526939676
          z: -0.382980321762
          w: 0.679618096202
  -
    header:
      seq: 0
      stamp:
        secs: 1542361017
        nsecs: 764956265
      frame_id: "/ardrone_base_link"
    id: 6
    confidence: 0
    pose:
      header:
        seq: 0
        stamp:
          secs: 0
          nsecs:         0
        frame_id: ''
      pose:
        position:
          x: 1.30798827365
          y: 0.468062354214
          z: -0.105472469423
        orientation:
          x: 0.444040791619
          y: -0.348176967016
          z: -0.552468994848
          w: 0.613497012829
```

### Observation des détections multiples (*bundles*)

Launchfile à utiliser : `roslaunch ardrone_carrier track_alvar_ardrone_bundles.launch`.
L'idée est de définir un "objet" rigide composé de plusieurs QR codes. On définit dans un fichier XML la position de ces QR codes pa rapport à la position d'un QR code *master*, ou principal. Seront publiés alors sur `/ar_pose_marker` la position du QR code *master* de ce bundle.
La détection multiple permet d'être plus robuste aux occlusions, et de fournir une meilleure estimation de la position de l'objet total.


## Tum Ardrone

Package ROS utilisant l'algorithme PTAM pour une meilleure estimation de la position du drone. Implémente un paquet complet de navigation pour l'Ardrone.
Plus d'infos [ici](http://wiki.ros.org/tum_ardrone).

Note : nécessite l'utilisation de la caméra frontale du drone.


## Calibration de la caméra

Pour que la détection des markers par **ar_track_alvar** soit efficace, il est nécessaire de calibrer les caméras du drone. Voir tutoriel [camera_calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).

Paramètres:
- `size` : taille en mètres des carrés de la mire
- `cam_topic` : nom du topic où sont publiées les images de la caméra à calibrer
- `cam_name` : nom de la caméra à calibrer

```
size=0.1085
cam_topic=/ardrone/bottom/image_raw
cam_name=/ardrone/bottom
rosrun camera_calibration cameracalibrator.py --size 8x6 --square $size image:=$cam_topic camera:=$cam_name
```

1. Lancer les commandes ci-dessus.
2. Bouger la mire dans le champ de vision de la caméra jusqu'à obtenir les jauges vertes.
3. Cliquer sur "Calibrate" UNE SEULE FOIS. Le calcul du modèle de transformation peut prendre jusqu'à 1 min.
4. Cliquer sur "Commit" pour enregistrer la calibration dans `/home/<username>/.ros/camera_info/`, ou sur "Save" pour écrire les résultats dans une archive `/tmp/calibrationdata.tar.gz`.
