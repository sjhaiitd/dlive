c = "0x"
def ChangeHex(n):
	x = (n % 16)
	global c
	if (x < 10):
		c += str(x)
	if (x == 10):
		c += "A"
	if (x == 11):
		c += "B"
	if (x == 12):
		c += "C"
	if (x == 13):
		c += "D"
	if (x == 14):
		c += "E"
	if (x == 15):
		c += "F"

	if (n - x != 0):
		return ChangeHex(n / 16) + str(c)
	else:
		return str(c)
	

if __name__ == '__main__':
	x =ChangeHex(100)
	print x