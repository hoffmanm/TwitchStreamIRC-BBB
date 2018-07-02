import socket # Connection for IRC
import serial
import httplib, urllib
from datetime import datetime # Output of current local time
import time # For delays

def joinchat():
	Loading = True
	while Loading:
		readbuffer_join = irc.recv(1024)
		readbuffer_join = readbuffer_join.decode()
		for line in readbuffer_join.split("\n")[0:-1]:
			print(line)
			Loading = loadingComplete(line)
def loadingComplete(line):
	if("End of /NAMES list" in line):
		print("Bot has joined " + CHANNEL + "'s Channel")
		sendMessage(irc,"Chat Room Joined")
		return False
	else:
		return True
def sendMessage(irc,message):
	messageTemp = "PRIVMSG #" + CHANNEL + " :" + message
	irc.send((messageTemp + "\n").encode())
SERVER = "irc.twitch.tv"
PORT = 6667
PASS = "oauth:mh1itzjd22pwml6zcj261koab0muz9"
BOT = "UNLV_Bot"
CHANNEL = "unlv_bot"
OWNER = "unlv_bot"
irc = socket.socket()
irc.connect((SERVER,PORT))
irc.send(("PASS " + PASS + "\n" +
		  "NICK " + BOT + "\n" +
		  "JOIN #" + CHANNEL + "\n").encode())
ser = serial.Serial(port = "/dev/ttyO1", baudrate=115200)
ser.close()
ser.open()
if ser.isOpen():
	joinchat()
	tempMessage = "CpE403 Final Project"
	sendMessage(irc,tempMessage)
	increment = 0
	while True:
		tempMessage = "Carbon Dioxide:"
		sendMessage(irc,tempMessage)
		reading = ser.readline()
		sendMessage(irc,reading)
		print "\nCarbon Dioxide:"
		params = urllib.urlencode({'field1': reading,'key':'1D9OBJ9X1GCH3ZX9'})
		headers = {"Content-type": "application/x-www-form-urlencoded","Accept":"text/plain"}
		conn = httplib.HTTPConnection("api.thingspeak.com:80")
		conn.request("POST", "/update", params, headers)
		res = conn.getresponse()
		print res.status, res.reason
		time.sleep(5)
