# ardrone_carrier

ROS project fro an ardrone landing on mobile platform.

## Ar Track Alvar

Pour lancer la détection des QR codes avec ar_track_alvar :
1. créer un répertoire "camera_info" dans les fichiers cachés de ROS : `mkdir ~/.ros/camera_info`
2. copier dans ce répertoire les fichiers de calibration des caméras du drone : `cp <ardrone_carrier>/config/ardrone_front.yaml ~/.ros/camera_info/ardrone_front.yaml` et `cp <ardrone_carrier>/config/ardrone_bottom.yaml ~/.ros/camera_info/ardrone_bottom.yaml`
3. lancer le driver du drone : `roslaunch ardrone_carrier ardrone_driver.launch`
4. lancer ar_track_alvar : `roslaunch ardrone_carrier track_alvar_ardrone.launch`
5. pour visualiser les détections, lancer `rviz` et ajouter les "Marker" correspondant au topic "/visualization_marker"
