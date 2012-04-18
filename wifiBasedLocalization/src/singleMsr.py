class singleMsr:
	def __init__(self, address, signalStrength, ESSID):
		self.address = address
		self.signalStrength = signalStrength
		self.ESSID = ESSID
	def __repr__(self):
		return self.address+' '+str(self.signalStrength)
