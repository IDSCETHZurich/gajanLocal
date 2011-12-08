#!/usr/bin/env python
import xml.dom.minidom
import httplib, urllib

url = 'http://labelme.csail.mit.edu/Annotations/static_houses_boston_2005/IMG_3637.xml'
xmlTmp = urllib.urlopen(url)

detectedObjects = set()
dom = xml.dom.minidom.parse(xmlTmp)
for object in dom.getElementsByTagName("object"):
	name = object.getElementsByTagName("name")
	detectedObjects.add(name[0].childNodes[0].nodeValue)

for objs in detectedObjects:
	print objs

