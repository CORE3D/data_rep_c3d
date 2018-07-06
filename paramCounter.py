import json
import os
import argparse

#this script is used to sum up the parameters of a CORE3D json data_rep file
# it is used to compute complexity metrics.

def parseFile(json_file):
  with open(json_file,'r') as fid:
    data = json.load(fid)
  return parseDict(data)

# parse JSON to sum up overall parameter count
def parseString(json_string,**kwargs):
  data = json.loads(json_string)
  return parseDict(data)


# parse dictionary into face/vertex struct
def parseDict(data):

  # check for required top-level metadata
  fields = ('name','team','timestamp','producer_id','scenes')
  fields_missing = [a for a in fields if (a not in data)]

  if fields_missing:
    raise AttributeError('Missing required fields: <{}>',format(
      ','.join(fields_missing)))

  totalParams = 0
  # add face/vertex to objects
  for scene in data['scenes']:
    coordinate_system = scene['coordinate_system']
    objects = scene['objects']
    for obj in objects:
      paramCount = parseObject(obj)
      totalParams = totalParams + paramCount

  # return updated data
  return totalParams

def parseObject(obj):
  # object type
  objtype = obj['type']

  # parse object
  if objtype == 'sphere':
    count = 1
  elif objtype == 'sphere_cap':
    count = 2
  elif objtype == 'rect_prism':
    count = 3
  elif objtype == 'cylinder':
    count = 2
  elif objtype == 'sliced_cyl':
    count = 3
  elif objtype == 'cone':
    count = 2
  elif objtype == 'torus':
    count = 2
  elif objtype == 'mesh':
    count = parseMesh(obj)
  elif objtype == 'ortho_extruded_polygon':
    count = parseOrthoExtrudedPolygon(obj)
  elif objtype == 'csg':
    count = parseCSG(obj,temppath=temppath)

  # apply transform (if exists)
  trans = obj.get('transform')
  if trans:
    count = count + parseTransform(trans)
  return count

#sum up parameters of transform
def parseTransform(trans):
  if not trans:
    count = 0
  else:
    transtype = list(trans.keys())[0]
  if transtype == 'affine_matrix':
    count = 16
  elif transtype =='translation':
    count = 3
  elif transtype == 'euler_angles':
    count = 3
  elif transtype == 'no_transform':
    count = 0
  else:
    raise ValueError(transtype + ' is not a valid transformation')
  return count

#count the mesh 
def parseMesh(mesh):
  v = mesh['vertices_3d']
  f = mesh['faces']
  count = 3*len(v) 
  for face in f:
      count = count + len(face)
  return count

def parseOrthoExtrudedPolygon(oep):
  v2d = data['vertices_2d']
  count = len(v2d)+1
  return count

def parseCSG(csgobj):
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
def main():

  # parse inputs
  parser = argparse.ArgumentParser()
  parser.add_argument('-i', '--inputfile', dest='inputfile',
      help='Input JSON file location', required=True)

  args = parser.parse_args()
  numParams = parseFile(args.inputfile)
  print("The total number of parameters is " + str(numParams))

if __name__ == "__main__":
  main()