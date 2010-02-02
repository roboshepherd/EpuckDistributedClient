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
