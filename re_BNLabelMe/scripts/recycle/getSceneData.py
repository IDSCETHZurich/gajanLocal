#!/usr/bin/env python
import httplib, urllib
import urlparse
sceneLink = 'http://labelme.csail.mit.edu/tool.html?collection=LabelMe&folder=static_web_submitted_noa_ofen_jenny_chai_stims&image=Indoor85.jpg'

baseXML = 'http://labelme.csail.mit.edu/Annotations/'

parsed_path = urlparse.urlparse(sceneLink)
params = dict([p.split('=') for p in parsed_path[4].split('&')])

XMLLoc = baseXML + params['folder'] + '/' + params['image'].replace('jpg','xml')
