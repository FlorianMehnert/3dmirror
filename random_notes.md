### sehr spezifisch auf pointclouds im cgv framework

- [x] timer event -in vr rgbd
- [x] no need for vrcontroler code 

- dont save pcl
- no record
- from started if changed -> refresh
  move code from initial frame
- renderer muss in init mit 1 und in clear mit -1 aufgerufen werden:
  `cgv::render::ref_point_renderer(ctx, 1);`
- highlighting in glsl in vscode über entsprechende erweiterung -> einfach Endung der entsprechenden shader eintragen

1. shader programms -> finde das glpr file -> am besten alte shader files dort ersetzen oder so
2. füge deine shader dort hinzu
3. baue den shader in der init function (siehe dazu rgbd_control)
4. überführen von irgendwelchen uniforms in der draw function

### raycasting

- holo raycast (in Examples) ist Startpunkt
- im holo_raycast.cpp -> how to use shader

### längerfristige Hausaufgaben

- scivis anschauen -> Path tracing: nachschauen in der sciview Vorlesung
- benjamin russig wg. spiegel
- paper zum raycasting (main focus)
- das Meiste was wichtig ist, kann man in rgbd_control nachschauen: hier ist auch die Pointcloud construction schon vorimplementiert
- die shader kann man in das build directory packen
- falls nochmal probleme bei cgv kinekt: azure kinekt plugin schauen nach readme
- ebenfalls kinect: zulassen dass desktop apps auf kamera zugreifen
- Keine perfekte pinhole Kamera deshalb keine lineare Verzerrung
- Funktion für Entzerrung ist vorimplementiert

### Hausaufgaben

- mesh construction using marching cubes -> dann auch Mal praktisch
  - 45 kamera render Positionen -> anstelle dessen wird eine depth map generiert und die sichtvektoren werden daraus generiert? (sehr speziell von gumhold zum spiegel und depthbuffer etc.)
  - erst raus bekommen ob sich der Punkt im view trustum befindet und dort schrittweise entzerren
- literature review:
  - wie genau kann man meshes aus einem tiefenbuffer erzeugen
  - wie funktioniert marching cubes
