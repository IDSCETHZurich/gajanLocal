#!/usr/bin/env python
import xml.dom.minidom
import httplib, urllib, urlparse, time
from BeautifulSoup import BeautifulSoup, SoupStrainer

url='http://people.csail.mit.edu/torralba/research/LabelMe/js/LabelMeQueryFast.cgi'
matchString = 'http://labelme.csail.mit.edu/tool.html?collection=LabelMe'
baseXML = 'http://labelme.csail.mit.edu/Annotations/'

# options: kitchen, bathroom, bedroom, 
searchValue = 'living room'
values = {'query': searchValue,'submit': 'Search'}
searchResponse = urllib.urlopen(url, urllib.urlencode(values)).read()

searchList = []
for link in BeautifulSoup(searchResponse, parseOnlyThese=SoupStrainer('a')):
	if link.has_key('href'):
		if link['href'].find(matchString) != -1:
			searchList.append(link['href'])

i = 0
for searchUrl in searchList:
	urlParsed = urlparse.urlparse(searchUrl)
	params = dict([p.split('=') for p in urlParsed[4].split('&')])
	url = baseXML + params['folder'] + '/' + params['image'].replace('jpg','xml')

	print '---'+searchValue+str(i)+'---'
	
	xmlTmp = urllib.urlopen(url)
	dom = xml.dom.minidom.parse(xmlTmp)

	detectedObjects = set()
	for object in dom.getElementsByTagName("object"):
		name = object.getElementsByTagName("name")
		detectedObjects.add(name[0].childNodes[0].nodeValue)

	if len(detectedObjects) > 0:
		print 'place(Room'+str(i)+') = '+searchValue+';'
		for objs in detectedObjects:
			print objs+'(Room'+str(i)+') = True;'

	time.sleep(5)
	i=i+1

