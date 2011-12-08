#!/usr/bin/env python
from BeautifulSoup import BeautifulSoup, SoupStrainer
searchResponse = open('searchResults.html','r')
matchString = 'http://labelme.csail.mit.edu/tool.html?collection=LabelMe';
searchList = []
for link in BeautifulSoup(searchResponse, parseOnlyThese=SoupStrainer('a')):
    if link.has_key('href'):
	tmpTxt = link['href']
	if tmpTxt.find(matchString) != -1:
        	searchList.append(link['href'])
		print tmpTxt




