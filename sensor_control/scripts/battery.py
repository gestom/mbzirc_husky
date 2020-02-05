#!/usr/bin/env python
import yagmail
import rospy
import time
from husky_msgs.msg import HuskyStatus

on = True

def callback(msg):
    global on
    if float(msg.battery_voltage) < 23 and on:
        on = False
        sendEmail()

def sendEmail():
    print("sending battery warning email")
    yag_smtp_connection = yagmail.SMTP( user="husky.chrono@gmail.com", password="huskyhusky", host='smtp.gmail.com')
    subject = 'Change my battery'
    contents = ['Hello this is husky speaking', 'Change my battery']
    yag_smtp_connection.send('broughtong92@gmail.com', subject, contents)
    time.sleep(1000)

if __name__ == "__main__":
    rospy.init_node("battery_email")
    rospy.Subscriber("/status", HuskyStatus, callback)
    rospy.spin()


