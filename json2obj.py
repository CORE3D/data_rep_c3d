import numpy as np
import primitiveMeshGen
import json
import os
import argparse
import copy

#indicates wheter or not to write a bounds polygon obj file
writeBoundsBool = False

# parse JSON file into face/vertex struct
def parseFile(json_file,**kwargs):
  with open(json_file,'r') as fid:
    data = json.load(fid)
  return parseDict(data,**kwargs)

# parse JSON string into face/vertex struct
def parseString(json_string,**kwargs):
  data = json.loads(json_string)
  return parseDict(data,**kwargs)

# parse dictionary into face/vertex struct
def parseDict(data,temppath=None):

  # check for required top-level metadata
  fields = ('name','team','timestamp','producer_id','scenes')
  fields_missing = [a for a in fields if (a not in data)]

  if fields_missing:
    raise AttributeError('Missing required fields: <{}>',format(
      ','.join(fields_missing)))

  # add face/vertex to objects
  for scene in data['scenes']:
    coordinate_system = scene['coordinate_system']
    bounds = scene.get('bounds')
    if bounds:
      [bv,bf] = parseBounds(bounds)
      scene['sceneVerts'] = bv
      scene['sceneFaces'] = bf
    objects = scene['objects']
    for obj in objects:
      obj['fv'] = parseObject(obj,temppath=temppath)

  # return updated data
  return data

# parse individual object into face/vertex structure
def parseObject(obj,temppath=None):

  # object type
  objtype = obj['type']

  # parse object
  if objtype == 'sphere':
    fv = parseSphere(obj)
  elif objtype == 'sphere_cap':
    fv = parseSphereCap(obj)
  elif objtype == 'rect_prism':
    fv = parseRectPrism(obj)
  elif objtype == 'cylinder':
    fv = parseCylinder(obj)
  elif objtype == 'sliced_cyl':
    fv = parseSlicedCyl(obj)
  elif objtype == 'cone':
    fv = parseCone(obj)
  elif objtype == 'torus':
    fv = parseTorus(obj)
  elif objtype == 'mesh':
    fv = parseMesh(obj)
  elif objtype == 'ortho_extruded_polygon':
    fv = parseOrthoExtrudedPolygon(obj)
  elif objtype == 'extruded_planar_region':
    fv = parsePlanarRegion(obj)
  elif objtype == 'csg':
    fv = parseCSG(obj,temppath=temppath)
  # ensure numpy array
  fv['vertices'] = np.array(fv['vertices'])

  # apply transform (if exists)
  trans = obj.get('transform')
  if trans:
    fv['vertices'] = primitiveMeshGen.applyAffineMatrix(
        fv['vertices'],parseTransform(trans))

  return fv


# parse affine transform
def parseTransform(trans):
  if not trans:
    transtype = 'no_transform'
  else:
    transtype = list(trans.keys())[0]

  if transtype == 'affine_matrix':
    affineList = trans['affine_matrix']
    affMat = np.array(affineList)
  elif transtype =='translation':
    translation = trans['translation']
    affineList = [[1,0,0,translation[0]],
                    [0,1,0,translation[1]],
                    [0,0,1,translation[2]],
                    [0,0,0,1]]
    affMat = np.array(affineList)
  elif transtype == 'euler_angles':
    angles = trans['euler_angles']
    affMat = primitiveMeshGen.eulerAnglesToRotationMatrix(angles)
  elif transtype == 'no_transform':
    affList = [[1,0,0,1],[0,1,0,1],[0,0,1,1],[0,0,0,1]]
    affMat = np.array(affList)
  else:
    raise ValueError(transtype + ' is not a valid transformation')
  return affMat

# parse sphere
def parseSphere(data):
  rad = data['radius']
  [v,f] = primitiveMeshGen.sphereGen(rad)
  return {'vertices':v,'faces':f}

#parse a sphere cap
def parseSphereCap(data):
  rad = data['radius']
  minelev = data['min_elevation']
  [v,f] = primitiveMeshGen.sphereCapGen(rad,minelev)
  return {'vertices':v,'faces':f}

#parse a cone
def parseCone(data):
  baseRad = data['base_rad']
  height = data['height']
  [v,f]= primitiveMeshGen.coneGen(baseRad,height)
  return {'vertices':v,'faces':f}

# parse rectangular prism
def parseRectPrism(data):
  l = data['length']
  w = data['width']
  h = data['height']
  [v,f] = primitiveMeshGen.rectPrismGen(l,w,h)
  return {'vertices':v,'faces':f}

#parse extruded planar region
def parsePlanarRegion(data):
  norm = data['normal']
  xbasis = data['x_basis']
  origin = data['origin']
  polyPts = data['polygon_points']
  gndZ = data['ground_level']
  [v,f] = primitiveMeshGen.planarRegionExt(norm,origin,xbasis,polyPts,gndZ)
  return {'vertices':v,'faces':f}

#parse cylinder
def parseCylinder(data):
  rad = data['radius']
  length = data['length']
  [v,f] = primitiveMeshGen.cylinderGen(rad,length)
  return {'vertices':v,'faces':f}

#parse sliced cylinder
def parseSlicedCyl(data):
  rad = data['radius']
  length = data['length']
  theta = data['theta']
  [v,f] = primitiveMeshGen.slicedCylGen(rad,length,theta)
  return {'vertices':v,'faces':f}

#parse torus
def parseTorus(data):
  rad1 = data['rad1']
  rad2 = data['rad2']
  [v,f] = primitiveMeshGen.torusGen(rad1,rad2)
  return {'vertices':v,'faces':f}

# parse mesh
def parseMesh(data):
  v = data['vertices_3d']
  f = data['faces']
  return {'vertices':v,'faces':f}

# parse ortho extruded polygon
def parseOrthoExtrudedPolygon(data):
  h = data['height']
  v2d = data['vertices_2d']
  [v,f] = primitiveMeshGen.orthoExPolyGen(v2d,h)
  return {'vertices':v,'faces':f}

# parse CSG - requires pymesh
def parseCSG(csgObject,temppath):

  # check for temppath existance
  if not temppath:
    raise ValueError("CSG parsing requires a temporary file path")
  elif not os.path.isdir(temppath):
    raise ValueError("Temporary file path <{}> does not exist".format(temppath))

  if 'csg_union' in csgObject:
    partList = csgObject['csg_union']
    opstr = 'union'
  elif 'csg_intersection' in csgObject:
    partList = csgObject['csg_intersection']
    opstr = 'intersection'
  elif 'csg_difference' in csgObject:
    partList = csgObject['csg_difference']
    opstr = 'difference'
  else:
    raise ValueError('not a valid csg operation')

  if len(partList)!=2:
    raise ValueError('Error, invlaid CSG. CSG must consist of only two objects')

  fv0 = parseObject(partList[0])
  fv1 = parseObject(partList[1])
  fv = primitiveMeshGen.booleanOp(fv0,fv1,opstr,temppath)

  return fv

#parse the coordinate system bounds for generating a validity mask
def parseBounds(boundsdata):
  boundsType = boundsdata['type']
  if boundsType == 'Polygon':
    verts = boundsdata['coordinates']
    for v in verts:
      v.append(0.0)
    faces = [range(len(verts))]
  else:
    print ('WARNING: Unsupported boundary type')
    verts = []
    faces = []
  return verts,faces

# write data to OBJ file
def writeObjFile(data,outpath):

  # standard header for each file
  header = [
    '# OBJ file for IARPA CORE3D program',
    '# written by VSI writeData function',
  ]

  fields = ('name','team','timestamp','producer_id')
  header.extend(['# {}: {}'.format(k,data.get(k)) for k in fields])
  header = '\n'.join(header) + '\n'

  # create output folder
  folder = os.path.join(outpath,data.get('name','unknown'))
  if not os.path.isdir(folder):
    os.makedirs(folder)

  # output files
  files = []

  # iterate through scenes
  for sceneidx,scene in enumerate(data['scenes']):

    # scene identifier
    sceneid = scene.get('id',sceneidx)

    # output file
    fileout = os.path.join(folder,'{}.obj'.format(sceneid))
    files.append(fileout)

    # buffer for top of OBJ scene file
    buf = copy.deepcopy(header)
    buf += '# scene_id: {}\n'.format(sceneid)
    buf += '# coordinate_system: {}\n'.format(
        json.dumps(scene.get('coordinate_system')))

    # objects to write
    objects = scene['objects']

    # write to file
    with open(fileout,'w') as fid:

      # write header to each file
      fid.write(buf)

      # vertex index offset
      offset = 1

      # write each object
      for objidx,obj in enumerate(objects):

        # object identifier
        objid = obj.get('id',objidx)

        # object header
        fid.write('\n')
        fid.write('# object_id: {}\n'.format(objid))
        fid.write('# vertex index 1: {}\n'.format(offset))
        fid.write('g {}\n'.format(objid))

        # write vertices
        for vert in obj['fv']['vertices']:
          fid.write('v {}\n'.format(
              ' '.join([str(v) for v in vert[0:3]])))

        # write faces
        for face in obj['fv']['faces']:
          fid.write('f {}\n'.format(
              ' '.join([str(f+offset) for f in face])))

        # increment vertex offset
        offset += len(obj['fv']['vertices'])

  return files

#writes a single obj file with 2d footprints of valid regions
def writeBounds(data,outputpath):
  if writeBoundsBool:
    folder = os.path.join(outputpath,data.get('name','unknown'))
    if not os.path.isdir(folder):
      os.makedirs(folder)
    fileout = os.path.join(folder,'bounds.obj')

    # standard header for each file
    header = [
      '# valid bounds OBJ file for IARPA CORE3D program',
      '# written by VSI writeBounds function',
    ]
    fields = ('name','team','timestamp','producer_id')
    header.extend(['# {}: {}'.format(k,data.get(k)) for k in fields])
    header = '\n'.join(header) + '\n'

    buf = copy.deepcopy(header)
    with open(fileout,'w') as fid2:
      fid2.write(buf)

      for scene in data['scenes']:
        # write scene group header header
        sceneid = scene.get('id')
        fid2.write('g {}\n'.format(sceneid))
        buf2 = '# coordinate_system: {}\n'.format(
          json.dumps(scene.get('coordinate_system')))
        fid2.write(buf2)

        bv = scene.get('sceneVerts')
        bf = scene.get('sceneFaces')
        if bv and bf:
          for v in bv:
            fid2.write('v ' + str(v[0]) + ' ' + str(v[1]) + ' ' + str(v[2]) + '\n')
          for f in bf:
            fid2.write('f ')
            for fv in f:
              fid2.write(str(fv+1) + ' ')
            fid2.write('\n')

# command line function
def main():

  # parse inputs
  parser = argparse.ArgumentParser()
  parser.add_argument('-i', '--inputfile', dest='inputfile',
      help='Input JSON file location', required=True)
  parser.add_argument('-o', '--outputpath', dest='outputpath',
      help='Output OBJ path', required=True)
  parser.add_argument('-t', '--temppath', dest='temppath',
      help='Temporary path for CSG parsing', required=False)

  args = parser.parse_args()

  # parse input JSON file & write OBJ to outputpath
  data = parseFile(args.inputfile,temppath=args.temppath)
  writeBounds(data,args.outputpath)
  writeObjFile(data,args.outputpath)

if __name__ == "__main__":
  main()
