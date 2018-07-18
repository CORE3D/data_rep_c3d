import math
import numpy as np
import os
import meshIO

#convert from spherical to 3D Cartesian coordinates
def sph2cart(az, el, r):
  rcos_theta = r * np.cos(el)
  x = rcos_theta * np.cos(az)
  y = rcos_theta * np.sin(az)
  z = r * np.sin(el)
  return x, y, z

#polar to Cartesian coordinates in 2D
def pol2cart(rad,theta):
  x = rad*np.cos(theta)
  y = rad*np.sin(theta)
  return x,y

#this function will generate a spherical mesh of a specified radius
def sphereGen(radius):
  #10 degree steps
  stepSize = (10)*math.pi/180
  ptlist = []
  elevs = np.arange(math.pi/-2,math.pi/2,stepSize)
  azimuths = np.arange(-math.pi,math.pi,stepSize)
  for elev in elevs:
    for az in azimuths:
     [x,y,z] = sph2cart(az,elev,radius)
     ptlist.append([x,y,z])
  xsteps = len(azimuths)-1
  ysteps = len(elevs)-1
  offset = 0
  faces = []
  for i in range(ysteps):
    for j in range(xsteps):
      ind0 = offset
      ind1 = ind0 + 1
      ind2 = ind1 + xsteps
      ind3 = ind2 - 1
      face = [ind0,ind1,ind2,ind3]
      faces.append(face)
      offset = offset + 1
    ind0 = offset
    ind1 = ind0 + 1
    ind2 = ind1 + xsteps
    ind3 = ind2 - 1
    face = [ind0,ind1,ind2,ind3]
    faces.append(face)
    offset = offset + 1
  #point at the top
  offset = len(ptlist)-xsteps-1
  lastInd = len(ptlist)
  [x,y,z] =  sph2cart(0,math.pi/2,radius)
  ptlist.append([x,y,z])
  for i in range(xsteps):
    ind0 = offset
    ind1 = offset +1
    ind2 = lastInd
    faces.append([ind0,ind1,ind2])
    offset = offset + 1
  ind0 = offset
  ind1 = len(ptlist)-xsteps-2
  ind2 = lastInd
  faces.append([ind0,ind1,ind2])
  offset = offset + 1
  return ptlist,faces

#this function generates a spherical cap of with a specified radius and minimum angle of elevation
def sphereCapGen(rad,minelev):
  stepSize = (7.5)*math.pi/180
  #minelev = math.pi/6
  maxelev = math.pi/2
  rad = 5
  ptlist = []
  faces = []
  #file = open("sph2.txt","w")
  elevs = np.arange(minelev,maxelev,stepSize)
  azimuths = np.arange(-math.pi,math.pi,stepSize)
  for elev in elevs:
    for az in azimuths:
      [x,y,z] = sph2cart(az,elev,rad)
      ptlist.append([x,y,z])
  xsteps = len(azimuths)-1
  ysteps = len(elevs)-1
  offset = 0
  for i in range(ysteps):
      for j in range(xsteps):
          ind0 = offset
          ind1 = ind0 + 1
          ind2 = ind1 + xsteps
          ind3 = ind2 - 1
          face = [ind0,ind1,ind2,ind3]
          faces.append(face)
          offset = offset + 1
      ind0 = offset
      ind1 = ind0 + 1
      ind2 = ind1 + xsteps
      ind3 = ind2 - 1
      face = [ind0,ind1,ind2,ind3]
      faces.append(face)
      offset = offset + 1
  #zenith point and top faces
  offset = len(ptlist)-xsteps-1
  lastInd = len(ptlist)
  [x,y,z] =  sph2cart(0,math.pi/2,rad)
  ptlist.append([x,y,z])
  for i in range(xsteps):
      ind0 = offset
      ind1 = offset +1
      ind2 = lastInd
      faces.append([ind0,ind1,ind2])
      offset = offset + 1
  ind0 = offset
  ind1 = len(ptlist)-xsteps-2
  ind2 = lastInd
  faces.append([ind0,ind1,ind2])
  offset = offset + 1
  #bottom face
  largeFace = []
  for i in range(0,xsteps+1):
      largeFace.append(xsteps - i)
  faces.append(largeFace)
  return ptlist,faces

#this function will generate a z axis aligned right cone
def coneGen(baseRad,height):
      angleStep = (5)*math.pi/180
      heightSteps = 2 # probably not super useful for full cones...
      angles = np.arange(0,2*math.pi, angleStep)
      verts = []
      faces = []
      #main conic vertices
      for zStep in range(heightSteps):
        z = (float(zStep)/heightSteps)*float(height)
        rad = (1-(float(zStep)/heightSteps))*baseRad
        for theta in angles:
              [x,y,_]=sph2cart(theta,0,rad)
              verts.append([x,y,z])
      #apex
      verts.append([0,0,height])
      #conic faces
      n = len(angles)
      offset = 0
      for zStep in range(heightSteps-1):
            for i in range(n-1):
                  v0 = offset
                  v1 = offset + 1
                  v2 = offset + n + 1
                  v3 = offset + n
                  faces.append([v0,v1,v2,v3])
                  offset = offset + 1
            v0 = offset
            v1 = offset - n + 1
            v2 = offset + 1
            v3 = offset + n
            faces.append([v0,v1,v2,v3])
            offset = n*(zStep+1)
      #apex faces
      vertLen = len(verts)
      startVert = vertLen - n - 1
      for i in range(len(angles)-1):
            v0 = startVert + i
            v1 = v0 + 1
            v2 = vertLen -1
            faces.append([v0,v1,v2])
      v0 = vertLen - 2
      v1 = startVert
      v2 = vertLen - 1
      faces.append([v0,v1,v2])
      #bottom face
      tface = []
      for i in range(n):
            tface.append(n-1-i)
      faces.append(tface)
      return verts, faces

#this function will generate a rectangular prism at the origin
#assumes axis aligned and length = x direction, width = y direction
# z = height.
def rectPrismGen(length,width,height):
  verts = []
  vert = [0,0,0]#1
  verts.append(vert)
  vert = [0, 0,height]#2
  verts.append(vert)
  vert = [0, width,0]#3
  verts.append(vert)
  vert = [0, width,height]#4
  verts.append(vert)
  vert = [length,0,0]#5
  verts.append(vert)
  vert = [length, 0,height]#6
  verts.append(vert)
  vert = [length, width,0]#7
  verts.append(vert)
  vert = [length, width,height]#8
  verts.append(vert)

  faces = []
  faces.append([0,2,6,4])
  faces.append([4,6,7,5])
  faces.append([2,3,7,6])
  faces.append([0,1,3,2])
  faces.append([0,4,5,1])
  faces.append([1,5,7,3])
  return verts,faces

#this function will generate an extruded polygon
def orthoExPolyGen(verts2D,height):
  verts =[]
  faces = []
  numV = len(verts2D)
  #if the polygon is counterclockwise we need to flip it
  #so the bottom surface has the correct normal.
  if counterClockwiseCheck(verts2D):
    verts2D.reverse()
  face =[]
  #bottom face
  for i in range(numV):
    v = verts2D[i]
    verts.append([v[0],v[1],0])
    face.append(i)
  faces.append(face)
  #top face
  face = []
  for i in range(numV):
    v = verts2D[i]
    verts.append([v[0],v[1],height])
    face.append(i +numV)
  face.reverse()
  faces.append(face)
  #side faces
  for i in range(numV-1):
    v0 = i + numV
    v1 = v0 + 1
    v2 = i + 1
    v3 = i
    faces.append([v0,v1,v2,v3])
  v0 = 2*numV -1
  v1 = numV
  v2 = 0
  v3 = numV-1
  faces.append([v0,v1,v2,v3])
  return verts,faces

#this function will generate the mesh structure of a z axis aligned cylinder of a
#specified radius and axis length
def cylinderGen(rad,length):
  [v,f] =slicedCylGen(rad,length,2*math.pi)
  return v,f

#this function will generate a sliced cylindrical mesh defined as a z axis
# aligned cylinder of a specified radius and axis length defined by an angle
# theta defining the slice
def slicedCylGen(rad,length,theta):
  angleStep = (1)*math.pi/180
  verts =[]
  faces =[]
  angles = np.arange(0,theta,angleStep)
  #bottom vertices
  for theta2 in angles:
    [x,y,_] = sph2cart(theta2,0.0,rad)
    verts.append([x,y,0.0])
  #top vertices
  for theta2 in angles:
    [x,y,_] = sph2cart(theta2,0.0,rad)
    verts.append([x,y,length])
  n = len(angles)-1
  for i in range(0,n):
    v0 = i
    v1 = i+1
    v2 = i + n + 2
    v3 = i + n + 1
    faces.append([v0,v1,v2,v3])
  #last cylindrical face
  v0 = n
  v1 = 0
  v2 = n+1
  v3 = 2*n+1
  faces.append([v0,v1,v2,v3])
  #bottom face
  tface = []
  for i in range(n,-1, -1):
    tface.append(i)
  faces.append(tface)
  #top face
  tface = []
  for i in range(n+1,2*n+2):
    tface.append(i)
  faces.append(tface)
  return verts, faces

#this function will generate a torus with the axis of revolution aligned with the z axis
#it is defined by rad1(the circle that is revolved) and rad2 (distance to axis of revolution)
def torusGen(rad1,rad2):
  angleStep = (3)*math.pi/180
  verts = []
  faces =[]
  angles = np.arange(0,2*math.pi,angleStep)
  #for each step of revolution
  for theta2 in angles:
    tverts = []
    affMat = eulerAnglesToRotationMatrix([0,0,theta2])
    #steps of the revolving circle
    for theta1 in angles:
      [x,y] = pol2cart(rad1,theta1)
      tverts.append([x+rad2,0,y]) #looks funky but...
    rotVerts = applyAffineMatrix(np.array(tverts),affMat)
    verts = verts + rotVerts.tolist()
  #faces
  n = len(angles)
  maxV = len(verts)
  offset = 0
  for j in range(n):
    for i in range(n-1):
      v0 = (offset + i)%maxV
      v1 = (v0 + n)%maxV
      v2 = (v1 + 1)%maxV
      v3 = (v0 + 1)%maxV
      faces.append([v0,v1,v2,v3])
    v0 = (offset + n -1)%maxV
    v1 = (v0 + n)%maxV
    v2 = (v0 + 1)%maxV
    v3 = (offset)%maxV
    faces.append([v0,v1,v2,v3])
    offset = offset + n
  return verts,faces

#this function will make extrusions of polygons that may not be aligned to the xy plane
def fixedZOrthoExPoly(verts3D,zval):
  verts =[]
  faces = []
  numV = len(verts3D)
  #top face
  face = []
  for i in range(numV):
    v = verts3D[i]
    verts.append([v[0],v[1],v[2]])
    face.append(i)
  faces.append(face)
  #bottom face
  face = []
  for i in range(numV):
    v = verts3D[i]
    verts.append([v[0],v[1],zval])
    face.append(i +numV)
  face.reverse()
  faces.append(face)
  #side faces
  for i in range(numV-1):
    v0 = i+1
    v1 = i
    v2 = i + numV
    v3 = v2 + 1
    faces.append([v0,v1,v2,v3])
  v0 = 0
  v1 = numV-1
  v2 = 2*numV -1
  v3 = numV
  faces.append([v0,v1,v2,v3])
  return verts,faces

#this function is used to check to see if a list of 2D vertices are clockwise
def counterClockwiseCheck(vertList):
  sum = 0
  for x in range(1,len(vertList)):
    v1 = vertList[x-1]
    v2 = vertList[x]
    t = (v2[0]-v1[0])*(v2[1]+v1[1])
    sum = sum + t
  v1 = vertList[len(vertList)-1]
  v2 = vertList[0]
  t = (v2[0]-v1[0])*(v2[1]+v1[1])

  sum = sum + t
  return sum < 0

#pads vertices and multiplies the matrices
def applyAffineMatrix(verts,affineMat):
  homoVerts = np.array(padVertsHomogeneous(verts))
  transformedHomoVerts = np.transpose(affineMat.dot(np.transpose(homoVerts)))
  transformedVerts = unpadVerts(transformedHomoVerts)
  return transformedVerts

#returns a matrix with homogeneous coordinates
def padVertsHomogeneous(verts):
  if (verts.ndim!=2) or (verts.shape[1]!=3):
    raise ValueError("Expected vertices of shape Nx3")
  return np.insert(verts,3,1,axis=1)

#for getting rid of the homogeneous coordinates
def unpadVerts(homoVerts):
  if (homoVerts.ndim!=2) or (homoVerts.shape[1]!=4):
    raise ValueError("Expected homogeneous vertices of shape Nx4")
  return homoVerts[:,0:3]

#this function returns a 4x4 affine matrix from a set of euler angles
def eulerAnglesToRotationMatrix(angles) :

  R_x = np.array([[1,         0,                  0                   ],
                  [0,         math.cos(angles[0]), -math.sin(angles[0]) ],
                  [0,         math.sin(angles[0]), math.cos(angles[0])  ]
                  ])

  R_y = np.array([[math.cos(angles[1]),    0,      math.sin(angles[1])  ],
                  [0,                      1,      0                   ],
                  [-math.sin(angles[1]),   0,      math.cos(angles[1])  ]
                  ])

  R_z = np.array([[math.cos(angles[2]),    -math.sin(angles[2]),    0],
                  [math.sin(angles[2]),    math.cos(angles[2]),     0],
                  [0,                     0,                      1]
                  ])


  rotMat = np.dot(R_z, np.dot( R_y, R_x ))
  affMat = np.pad(rotMat,[[0,1],[0,1]],mode='constant',constant_values=0)
  affMat[3,3] = 1
  return affMat

#this function uses pymesh and cork for CSG boolean operations.
def booleanOp(fv0,fv1,opstr,temppath):

  try:
    import pymesh
  except ImportError:
    print("ERROR: CSG requires pymesh, which could not successfully be imported")
    print("returning first mesh only.")
    return fv0
  else:
    file0 = os.path.join(temppath,'file0.obj')
    file1 = os.path.join(temppath,'file1.obj')
    writeFV(file0,fv0)
    writeFV(file1,fv1)

    mesh1 = pymesh.meshio.load_mesh(file0)
    if mesh1.vertex_per_face != 3:
      mesh1 = pymesh.quad_to_tri(mesh1)
    mesh2 = pymesh.meshio.load_mesh(file1)
    if mesh2.vertex_per_face != 3:
      mesh2 = pymesh.quad_to_tri(mesh2)
    meshout = pymesh.boolean(mesh1,mesh2,operation = opstr,engine = 'igl')

  # os.remove(file0)
  # os.remove(file1)

  return {'vertices':meshout.vertices,'faces':meshout.faces}

def writeFV(file,fv):
  with open(file,'w') as fid:

    for vert in fv['vertices']:
      fid.write('v {}\n'.format(
          ' '.join([str(v) for v in vert[0:3]])))

    for face in fv['faces']:
      fid.write('f {}\n'.format(
          ' '.join([str(f+1) for f in face])))


#Scott uses this for debugging mesh generation
#nothing to see here
#if __name__ == "__main__":
#   inverts = [[0,0,1],[2,0,1],[3,1,2],[2,2,2],[0,2,2]]
#   [v,f] = fixedZOrthoExPoly(inverts,-10)
#   meshIO.writeObj(v,f,'temp.obj')