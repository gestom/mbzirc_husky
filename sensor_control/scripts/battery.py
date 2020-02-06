#!/usr/bin/env python
import yagmail
import rospy
import time
from husky_msgs.msg import HuskyStatus
import random

on = True

def callback(msg):
    global on
    if float(msg.battery_voltage) < 23.8 and on:
        on = False
        sendEmail(msg.battery_voltage)

def sendEmail(vltge):
    print("sending battery warning email")
    yag_smtp_connection = yagmail.SMTP( user="husky.chrono@gmail.com", password="huskyhusky", host='smtp.gmail.com')
    
    subject = 'Change my battery'
    messages = ["Change my battery asshole", "Hey bitch change my battery"]

    msgIdx = random.randint(0,len(messages)-1)
    contents = [messages[msgIdx], ", i'm down to " + str(vltge) + "volts"]
    emails = ["broughtong92@gmail.com", "filipmajer93@gmail.com", "krajnt1@fel.cvut.cz"]
    #emails = ["broughtong92@gmail.com", "filipmajer93@gmail.com"]
    for i in emails:
        print("sending to " + i)
        yag_smtp_connection.send(i, subject, contents)
    time.sleep(1000)

if __name__ == "__main__":
    rospy.init_node("battery_email")
    rospy.Subscriber("/status", HuskyStatus, callback)
    rospy.spin()


