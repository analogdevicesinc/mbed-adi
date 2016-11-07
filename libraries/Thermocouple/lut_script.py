def isFloat(string):
    try:
        float(string)
        return True
    except ValueError:
        return False

with open("thermocouple.cpp","w") as out:
    with open("Thermocouple_core_cpp.txt","r") as inp:
        out.write(inp.read())
        out.write("\n\n\n")
        pass

with open("thermocouple.h","w") as out:
    with open("Thermocouple_core_h.txt", "r") as inp:
        out.write(inp.read())
        out.write("\n\n\n")
    pass

with open("thermocouple_lut.cpp","w") as out:
    with open("Thermocouple_lut_core_cpp.txt","r") as inp:
        out.write(inp.read())
        out.write("\n\n\n")
        pass

with open('tc.txt', 'r') as myfile:
    initstr=myfile.read().replace('\n', ' ')


#print(initstr.split());
splitted = initstr.split();

while splitted:
    i=0

    for word in splitted:
        if(word == 'type'):
            break
        i=i+1
    del splitted[:i+1]
    if not splitted:
        break
    tctype = splitted[0][0].lower()

    class_name = "Thermocouple_Type_" + tctype.capitalize()
    lut_def_name = "TYPE_" + tctype.capitalize() + "_LUT"
    print(lut_def_name)

    i=0
    state=0
    lut = []
    for word in splitted:
        if(word=="************************************"):
            break
        if(word=="mV"):
            state = 1
            continue
        if(state == 1 and isFloat(word)):
            lut.append(word)
        else:
            state=0
        i=i+1
    print(lut)
    del splitted[:i]
    i=0

    globaloffset = lut[0]

    if(float(lut[0]) == float(lut[2])-10):
        del lut[:3]
    else:
        del lut[0]

    j=0
    print lut
    lutfloat=[]
    for val in lut:
        b=1
        if(i%11==0):
            #print "11 " + val
            b=0
        if(i%12==0):
            #print "12 "+ val
            i=0
            b=0
        i=i+1
        j=j+1
        if(b):
            lutfloat.append(float(val))


    #print "unsorted"
    #print lutfloat
    lutfloat.sort()
    #print "sorted"
    #print lutfloat
    i=0
    with open("thermocouple_lut.cpp", "a") as out:
        out.write("#ifdef "+lut_def_name+"\n")
        out.write("const int16_t " + class_name + "::lut_offset = " + str(globaloffset)+";\n")
        out.write("const uint16_t " + class_name + "::lut_size = " + str(len(lutfloat))+";\n")
        out.write("const int32_t "+class_name+"::lut["+str(len(lutfloat))+"] = {\n")
        for val in lutfloat:
            out.write(str(int(val*1000)).rjust(8)+ ", ")
            i=i+1
            if(i==10):
                out.write("\n")
                i=0
        out.write("};\n")
        out.write("#endif\n\n")
    i=0

    ranger = []
    values = []
    exp=[]
    ranger_index = -1 #HACK HACK HACK

    while(splitted[0] != '************************************'):
        for word in splitted:
            if(word=='range:'):
                ranger.append([])
                values.append([])
                ranger_index=ranger_index+1
                break
            if(word=='exponential:'):
                exp.append(splitted[i+3])
                exp.append(splitted[i+6])
                exp.append(splitted[i+9])
                i=i+10
                print exp
                break
            i=i+1
        if(word=='exponential:'):
            print("exponential")
            del splitted[:i + 1]
            i=0
            break
        del splitted[:i+1]
        print splitted
        i=0

        ranger[ranger_index].append(splitted[0])
        ranger[ranger_index].append(splitted[1])
        nr_of_coef = int(splitted[2])+1
        del splitted[:3]
        for i in range(nr_of_coef):
            values[ranger_index].append(splitted[i])
        del splitted[:nr_of_coef]
        print ranger
        print values
        i=0

    with open("Thermocouple.h", "a") as out:
        poly_size = str(len(ranger))

        out.write("class " + class_name + " : public Thermocouple\n{")
        out.write("\npublic:\n\t~" + class_name +"();")
        out.write("\n\tstatic const thermocouple_poly_subrange inv_poly[" + poly_size + "];\n")
        out.write("\tstatic const int inv_poly_size;\n\tfloat convert_inv(float temp);\n\n")

    with open("Thermocouple.cpp", "a") as out:

        out.write("const int " + class_name + "::inv_poly_size = " + str(len(ranger)) + ";\n")
        out.write("const Thermocouple::thermocouple_poly_subrange " + class_name + "::inv_poly[" + poly_size + "] =\n{\n")
        for ran in ranger:
            poly = []
            power = []
            for val in values[i]:
                a = val.split("E")
                poly.append(a[0])
                power.append(int(a[1]))
            print(poly)
            print(power)
            out.write("\t{")
            for r in ran:
                out.write(r.rjust(11))
            out.write("// characteristic curve for temp range between " + ran[0] + " and " + ran[1] + "\n\t{")
            for p in poly:
                out.write(str(p).rjust(11) + ",")
            out.write("},\n")
            out.write("\t{")
            for p in power:
                #    print(p)
                out.write(str(p).rjust(11) + ",")
            out.write("},\n\t")
            out.write(str(len(poly)))
            out.write(" }")

            poly = []
            power = []
            i = i + 1
            if i != len(ranger):
                out.write(",")
            out.write("\n")
        out.write("};\n")

        out.write(
            "\nfloat " + class_name + "::convert_inv(float temp)\n{\n\treturn Thermocouple::convert(temp, inv_poly, inv_poly_size);\n}\n")
        out.write(
            "\nfloat " + class_name + "::lookup_inv(float temp)\n{""\n#ifdef " + lut_def_name + "\n\tif((temp+lut_offset)>lut_size)\n\t\treturn lut[lut_size-1];\n\telse\n\t\treturn lut[(uint16_t)temp+lut_offset];\n#else\n\t/* NOT IMPLEMENTED */\n\treturn 0;\n#endif\n}\n")

        i=0

    for word in splitted:
        if(word == 'Voltage' and splitted[i-1] != "Temperature"):
            break
        i=i+1

    del splitted[:i+1]
    i=0
    #print(splitted)
    ranger = []
    values = []
    for word in splitted:

        if(word == 'Range:'):
            break
        ranger.append([word])
        #print(i)
        i = i + 1

    del splitted[:i+1]
    for j in range(i):
        ranger[j].append(splitted[j])
        values.append([])
    del splitted[:i]
    #print(values)

    i=0
    for word in splitted:
        if(word == "Error"):
            break
        values[i%len(ranger)].append(word)
        i=i+1


    poly=[]
    power = []
    i=0

    with open("Thermocouple.h","a") as out:
        poly_size = str(len(ranger))

        out.write("\tstatic const thermocouple_poly_subrange poly["+poly_size+"];\n")
        out.write("\tstatic const int poly_size;\n\tfloat convert(float voltage);\n")
        out.write("#ifdef "+lut_def_name+"\n")
        out.write("\tstatic const int32_t lut[];\n\tstatic const int16_t lut_offset;\n\tstatic const uint16_t lut_size;\n\tfloat lookup(float voltage);\n\tfloat lookup_inv(float temp);\n#endif\n};\n\n\n")


    with open("Thermocouple.cpp","a") as out:

        out.write("const int "+ class_name + "::poly_size = "+str(len(ranger))+";\n")
        out.write("const Thermocouple::thermocouple_poly_subrange "+ class_name+ "::poly["+poly_size+"] =\n{\n")
        for ran in ranger:
            for val in values[i]:
                a=val.split("E")
                poly.append(a[0])
                power.append(int(a[1]))
            print(poly)
            print(power)
            out.write("\t{")
            for r in ran:
                out.write(r.rjust(11) +", ")
            out.write("// characteristic curve for mV range between " + ran[0] +" and "+ ran[1]+ "\n\t{")
            for p in poly:
                out.write(str(p).rjust(11) + ",")
            out.write("},\n")
            out.write("\t{")
            for p in power:
            #    print(p)
               out.write(str(p).rjust(11) + ",")
            out.write("},\n\t")
            out.write(str(len(poly)))
            out.write(" }")

            poly = []
            power = []
            i=i+1
            if i!=len(ranger):
                out.write(",")
            out.write("\n")
        out.write("};\n")
        out.write("\n" + class_name + "::~"+class_name+"()\n{\n\n}\n")
        out.write("\nfloat "+ class_name +"::convert(float voltage)\n{\n\treturn Thermocouple::convert(voltage, poly, poly_size);\n}\n")
        out.write("\nfloat "+class_name+"::lookup(float voltage)\n{""\n#ifdef "+lut_def_name+"\n\treturn Thermocouple::lookup(lut, voltage, lut_size, lut_offset);\n#else\n\t/* NOT IMPLEMENTED */\n\treturn 0;\n#endif\n}\n")

with open("Thermocouple.h", "a") as out:
    out.write("\n#endif\n")





#print(ranger)
#print(values)





