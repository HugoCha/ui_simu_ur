#! /usr/bin/env python3
import sys

from PyQt5 import QtCore

class Clock():
    """
    A class to manage the UI clock
    """

    def __init__(self, init_time, timeedit):
        self.timer0 = QtCore.QTimer()
        self.init_time = init_time
        self.time = QtCore.QTime(init_time[0], init_time[1], init_time[2])
        self.timer0.setInterval(1000)
        self.timer0.timeout.connect(self.countdown)
        self.timeedit = timeedit
    
    def pause(self):
        """
        Function to pause the countdown
        """
        self.timer0.stop()
    
    def play(self):
        """
        Function to play the countdown
        """
        self.timer0.start()

    def reset(self):
        """
        Function to reset the clock
        """
        self.time = QtCore.QTime(self.init_time[0], self.init_time[1], self.init_time[2])
        self.timeedit.setTime(self.time)
        self.pause()

    def is_over(self):
        """
        Function to check if the clock is finished 
        """
        return (self.time == QtCore.QTime(0,0,0))

    def countdown(self):
        """
        Function called when timeout overflow 1 sec
        """
        if (self.is_over()):
            self.timer0.stop()
        else:
            self.time = self.time.addSecs(-1)
            self.timeedit.setTime(self.time)
        
        # print(self.time.toString("mm:ss"))

