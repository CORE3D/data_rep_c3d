#Core3D JSON data representation and parser
This repo is a proof of concept for a JSON data representation of 3D Primitives and other geometry, as well as tools for parsing and recreating the geometry.

#Data Format
The proposed format contains information about the producer, their team, the scene and its geometry. Since this data representation is aimed to assist in evaluating performer results, the file itself keeps track of many details that may be relevant for testing and evaluation, as well as internal debugging and evaluation. The geometric representation aims to be flexible allowing individual teams to opperate how they desire, but allow for a simplicity or compactness measure, as well as a semantic metric to be easily computed.

##Scene level information
Each file will contain information about the complete 3D scene and the producer. This information is primarily to aid in tracking files and identifying them geographically and within the context of evaluation.
Scene level infromation consists of:

**Name** the name of the scene which this file represents, for example 'D0_WPAFB'
**Team** the team that has produced this file, for example 'VSI'
**timestamp**  when the file was produced in UTC for example "2017-11-29T20:20:54+00:00"
**producer-id** the person or process that produced the file for example 'vsi-scott', or 'vsi\_final\_core3d\_pipeline
**coordinate system** the coordinate system of the results with included parameters, for example a local vertical coordinate system (LVCS)

##Geometry information
The file then contains a list of 3D objects which are used to represent the geometry of structures in the scene. These objects have a type and transform as well as parameters specific to their type. There are a variety of ways to express the same geometry, which should yield identical precision in evaluation, however some representations may require fewer parameters to express, and this is also true for various transformation types. Accordingly we have supported a variety of options to allow individual performers to choose whichever is best for their situation.
###Transformation
Each object has a transformation to place it within the coordinates of the scene. We have implemented the following transformations:

**Affine Matrix** a 4x4 matrix expressing a complete affine transformation including translation, rotation, scaling etc (16 parameters)

**Translation** 3D translation from the origin (3 parameters)
**Euler angles** rotation around the x,y, and z axis in radians (3 parameters)
**no transformation** no transformation, either because object is at the origin or object parameters contain positional information (0 parameters)

###3D Object Types
We have implemented a few objects as a basis and will expand this to include other primitives. The types implemented so far are:

**Sphere:** centered at the origin, with a specified radius (1 parameter)
**Spherical cap:** centered at the origin, specified by radius and minimum elevation angle (2 parameters)
**Cylinder:** centered at the origin, axis aligned, defined by a radius and length (2 parameters)
**Sliced cylider:** centered at the origin, axis aligned, defined by a radius and length and a slicing angle (3 parameters)
**Cone:** centered at the origin, axis aligned, defined by a base radius and height (2 parameters)
**Torus:** centered at the origin and axis aligned, defined by a a center and tube radius (2 parameters)
**Rectangular prism:** axis aligned with one vertex at the origin (3 parameters)
**Ortho-extruded polygon:** axis aligned 2D polygon of n vertices extruded along the z axis ((2*n)+1 parameters)
**Mesh:** 3D mesh consisting of 3D vertices and polygonal faces. Faces are described by counterclockwise lists of vertex indices (variable parameters)
**CSG:** 3D solids made using boolean operations of other solids. For example the union of two spheres. Each CSG has an operand and two children, these children are themselves 3D objects which can be primitives or other CSG's (1 parameter + childrens' parameter count) _note:_ CSG will turn all faces into triangular meshes, and requires pymesh and boolean engines to be installed.

##Scene meshing
We have provided a sample program and sample data to reconstruct the geometry encoded in this format. This program reads in the sample JSON file and creates a directory with series of .obj files for loading into a variety of 3D viewers/rendering programs. To execute the sample program python 2.7 along with numpy and pymesh are needed. Once all the dependencies are installed simply execute (from the "scripts" directory)

```
python json2obj.py -i sample.json -o ./output
```

and the program will run on the included sample file.
