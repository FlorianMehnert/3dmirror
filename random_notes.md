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

### meeting 19.12.2023
research on marching cubes and mesh from depth buffer
new tasks:
- add slider to filter points
- triangle or rectangle as primitive and think about the data structure simple mesh or halfedge
	- rgbd control only provides small hints about mesh construction
- tianfang will send me something about mesh reconstruction
	- marching cubes is already implemented - not the focus (but it is implemented using an estimation of the normals since the position of the camera is not known)
- further research
- focus is the implementation of mesh reconstruction from depth buffer:
	- convert the depth image to 2.5D
	- get position of camera
	- add slider to filter out further away points

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

### Referenzen:

marching cubes:

- [Video Sebastian Lague - visualisierung](https://www.youtube.com/watch?v=M3iI2l0ltbE)
- [gute Antwort zu marching cubes from pcl](https://stackoverflow.com/questions/56686838/point-cloud-triangulation-using-marching-cubes-in-python-3)
  - [Seite zu marching cubes von ungeordneten Punkten](https://hhoppe.com/proj/recon/) -> [github](https://github.com/hhoppe/Mesh-processing-library) (baut aber erst ein signed distance field und passed das dann zu marching cubes)

### mesh reconstruction

- Alpha shapes [[Edelsbrunner1983\]](http://www.open3d.org/docs/latest/tutorial/reference.html#Edelsbrunner1983) - schnell
  - Eis mit schoko so lange aushölen bis die Schokolinsen zu sehen sind
- Ball pivoting [[Bernardini1999\]](http://www.open3d.org/docs/latest/tutorial/reference.html#Bernardini1999)
- Poisson surface reconstruction [[Kazhdan2006\]](http://www.open3d.org/docs/latest/tutorial/reference.html#Kazhdan2006)

#### marching cubes

> From what I know, [marching cubes](https://en.wikipedia.org/wiki/Marching_cubes) is usually used for extracting a polygonal mesh of an isosurface from a three-dimensional discrete scalar field (that's what you mean by  volume). The algorithm does not work on raw point cloud data.
>
> Hoppe's algorithm works by **first generating a signed distance  function field** (a SDF volume), and then passing it to marching cubes.  This can be seen as an implementation to you `pcd_to_volume` and it's not the only way!
>
> If the raw point cloud is all you have, then the situation is a little bit constrained. As you might see, the [Poisson reconstruction](http://hhoppe.com/proj/poissonrecon/) and [Screened Poisson reconstruction](http://hhoppe.com/proj/screenedpoisson/) algorithm both implement `pcd_to_volume` in their own way (they are highly related). However, they needs  additional point normal information, and the normals have to be  consistently oriented. (For consistent orientation you can read [this question](https://stackoverflow.com/q/60346126/7413964)).
>
> While some [Delaunay](https://en.wikipedia.org/wiki/Delaunay_triangulation) based algorithm (*they do not use marching cubes*) like alphaShape and [this](https://doc.cgal.org/latest/Advancing_front_surface_reconstruction/index.html) may not need point normals as input, for surfaces with complex  topology, it's hard to get a satisfactory result due to orientation  problem. And the [graph cuts method](https://ieeexplore.ieee.org/abstract/document/4408892/) can use visibility information to solve that.

- surface reconstruction from open3D: http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html

<img src="http://www.open3d.org/docs/latest/_images/tutorial_Advanced_surface_reconstruction_3_0.png" style="zoom: 17%;" /> <img src="http://www.open3d.org/docs/latest/_images/tutorial_Advanced_surface_reconstruction_3_2.png" style="zoom:17%;" />

#### marching cubes threejs + video:

- https://threejs.org/examples/webgl_marchingcubes.html
- https://www.youtube.com/watch?v=zY0qnkT1dh8

#### marching cubes easier article

- https://paulbourke.net/geometry/polygonise/

### mesh from depthbuffer

- [Converting depth values to distances from z-buffer](https://forum.unity.com/threads/converting-depth-values-to-distances-from-z-buffer.921929/) -> [original project](https://forum.unity.com/threads/proper-way-to-compare-two-different-depth-values.968456/)

### interessante idee

- https://github.com/andyzeng/3dmatch-toolbox/tree/master

### warum nicht auch in shadertoys

- https://www.shadertoy.com/view/ld3Gz2

### cgal mesh generation

- https://doc.cgal.org/latest/Mesh_3/index.html

## near goals
- only glgs of the derived "point_renderer" is required $\rightarrow$ render quads instead of points (basically surfel renderer)
- find out at which step the meshing has to take place - where is this done in other projects
- store distance of points during the creation of the depth array and pass to the geometry shader or somewhere where it is needed
