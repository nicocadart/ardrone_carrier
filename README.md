# ardrone_carrier

ROS project fro an ardrone landing on mobile platform.

## Ar Track Alvar

### Mode d'emploi

Pour lancer la détection des QR codes avec ar_track_alvar :
1. créer un répertoire "camera_info" dans les fichiers cachés de ROS : `mkdir ~/.ros/camera_info`
2. copier dans ce répertoire les fichiers de calibration des caméras du drone : `cp <ardrone_carrier>/config/ardrone_front.yaml ~/.ros/camera_info/ardrone_front.yaml` et `cp <ardrone_carrier>/config/ardrone_bottom.yaml ~/.ros/camera_info/ardrone_bottom.yaml`
3. lancer le driver du drone : `roslaunch ardrone_carrier ardrone_driver.launch`
4. lancer ar_track_alvar : `roslaunch ardrone_carrier track_alvar_ardrone.launch`
5. pour visualiser les détections, lancer `rviz` et ajouter les "Marker" correspondant au topic "/visualization_marker"

![individual detections using ar_track_alvar](doc/imgs/ar_track_alvar_individual.png)


### Observation des détections individuelles et *tf*

Lorsque détectés, les markers sont publiés sous forme de liste dans le topic `/ar_pose_marker`. On peut passer en paramètre "*output_frame*" du noeud ar_track_alvar le frame_id à utiliser comme référence : les positions des markers seront alors dans cette référence. Exemple d'un message repérant les markers 8 et 6, et publiant leur position dans la référence `/ardrone_base_link`
```
---
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

TODO

## Tum Ardrone
