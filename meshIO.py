def writeObj(verts,faces,outFile,commentString = ''):
    file2 = open(outFile,'w')
    file2.write("#mesh written by VSI writeObj function. \n")
    if len(commentString) >0:
        file2.write(commentString + '\n') #emsure that each newline begins with a '#'
    for v in verts:
        file2.write("v " + str(v[0]) + " " + str(v[1]) + " " + str(v[2]) + "\n"  )
    faceBool = False
    for x in range(0,len(faces)):
        f = faces[x]
        file2.write("f ")
        for vi in f:
            file2.write(str(vi+ 1) + " ")
        file2.write("\n")
            