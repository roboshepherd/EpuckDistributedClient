
def trunc(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    slen = len('%.*f' % (n, f))
    return str(f)[:slen]

def extract_objects(object_list):
    list = []
    for object in object_list:
	val = str(object)
	list.append(eval(val) )
    return  list

def GetDBusPaths(robots_cfg):
    dbus_paths = []
    f = open(robots_cfg, 'r')
    for line in f.readlines():
	if line.endswith('\n'):
	    line = line[:-1]
	if(line[0] == '/'):
	    dbus_paths.append(line)
    f.close()
    return dbus_paths
