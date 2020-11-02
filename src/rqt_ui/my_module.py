#!/usr/bin/env python

import os, sys
import rospy
import rospkg
from copy import deepcopy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *


from clock import Clock
from simu_ur.srv import StateService

from std_msgs.msg import String
from simu_ur.msg import Game

PACKAGE_NAME = "ui_simu_ur"
import roslib; roslib.load_manifest(PACKAGE_NAME)


MINUTE = 3
SECOND = 0

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        
        
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                        dest="quiet",
                        help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Load all the ui files created with Qt designer
        self.load_ui_file()
        

        # Init Main Widget
        self.init_main_widget()

        self.init_signal_popup()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Suscribe to block detection in order to activate a popup when = "true"
        self.block_dectect_sub = rospy.Subscriber("/block/Detect", String, self.detection_callback, queue_size=1)

        #Suscribe to update the game
        self.game_state = Game()
        self.game_sub = rospy.Subscriber("/Game", Game, self.game_callback, queue_size=1)
        self.start_game = False
        self.first_click_play = True
        self.home = False

        # Publisher of block detection change if a block is
        self.process_picknplace = "false"
        self.is_process = False
        self.process_picknplace_pub = rospy.Publisher("UI/Process", String, queue_size=1)

    

    def load_ui_file(self):
        """
        Load the ui file from resource folder
        """
        # Create QWidget
        self._widget = QWidget()
        self.work_in_progress_msg = QWidget()
       
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path(PACKAGE_NAME), 'resource', 'Ur3_cube.ui')
        wip_msg_file = os.path.join(rospkg.RosPack().get_path(PACKAGE_NAME), 'resource', 'workinprogress.ui')
        
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        loadUi(wip_msg_file, self.work_in_progress_msg)


    ############################################### 
    ######## Handle of the main window ############
    ###############################################     
    def init_main_widget(self):
        """
        initialization of the main window
        """
        self._widget.setObjectName('MyPluginUi')

        self._widget.move(QApplication.desktop().screen().rect().center()- self._widget.rect().center())

        # Init clock
        self.clock = Clock((0,MINUTE,SECOND), self._widget.clock_display)
        self.clock.reset()

        # Init push buttons
        self._widget.play.clicked.connect(self.play_ui)
        self._widget.pause.clicked.connect(self.pause_ui)
        self._widget.stop.clicked.connect(self.stop_ui)
        self._widget.home.clicked.connect(self.home_ui)
        
        # Init LCD number record
        self.init_record()

        # Init LCD number score
        self.update_score(0)
    
    
    def play_ui(self):
        """
        Function callback when play is clicked
        """

        if ( self.first_click_play ):
            self.stop_ui()
        self.first_click_play = False
        self.reset_signal()
        # Play clock  
        self.clock.play()
        self.start_game = True

        # Service of pickandplace.py
        rospy.wait_for_service("/set_robot_state")
        try: 
            player = rospy.ServiceProxy("/set_robot_state", StateService)
            player(0,1,0,0)
        except rospy.ServiceException as e:
            print("Service call failed: ",e)

        
        #self.work_in_progress()
    
    def pause_ui(self):
        """
        Function callback when pause is clicked
        """
        
        # Pause clock 
        self.clock.pause()
        self.start_game = False
        # Service of pickandplace.py
        rospy.wait_for_service("/set_robot_state")
        
        try: 
            player = rospy.ServiceProxy("/set_robot_state", StateService)
            player(1,0,0,0)
        except rospy.ServiceException as e:
            print("Service call failed: ",e)

        
    
    def stop_ui(self):
        """
        Function callback when stop is clicked
        """
        
        # Reset clock 
        self.clock.reset() 
        self.start_game = False
        self.first_click_play = False
        self.home = False
        self.signal.init_signal.emit()
    
    def home_ui(self):
        """
        Function callback to return to home position
        """
        # If the robot is not on pause or stop it's not possible to go to home position
        if (not(self.start_game)):
            self.first_click_play = False
            self.home = True
            self.signal.init_signal.emit()


    def init_record(self):
        """
        Function to keep in memory the record and load it at the initialization
        """
        f = open(os.path.join(rospkg.RosPack().get_path(PACKAGE_NAME), 'resource', 'record.txt'), "r")
        content = f.read()
        if (content==""):
            self.record_value = 0
        else:
            self.record_value = int(content)
        self._widget.record.display(self.record_value)
        f.close()

    def update_score(self, new_score):
        """
        Function to update score and the record if it is beaten
        """
        self._widget.score.display(new_score)
        if (new_score > self.record_value):
            f = open(os.path.join(rospkg.RosPack().get_path(PACKAGE_NAME), 'resource', 'record.txt'), "w")
            f.write(str(new_score))
            f.close()
            self._widget.record.display(new_score)
    
    def game_callback(self, data):
        """
        Function to update interface when score is received, progress and error
        """
        if (self.game_state.score != data.score):
            self.update_score(data.score)
            self.reset_signal()

        if (data.progress >= 90):
            self.process_picknplace = "false"

        self.update_wip(data.progress)
        self.game_state = deepcopy(data)


    def init_signal_popup(self):
        self.signal = MySignal()
        self.signal.detection_signal.connect(self.detection_pop_up)
        self.signal.pause_popup_signal.connect(self.pause_pop_up)
        self.signal.init_signal.connect(self.initialisation_pop_up)
        self.signal.game_over_signal.connect(self.game_over_pop_up)
        self.signal.error_signal.connect(self.error_popup)

        # Init Pop up work in progress
        self.init_wip()

        # Init pop up when a cube is detected
        self.init_detection_msg()
        self.init_pause_msg()
        self.init_intialisation_msg()
        self.init_game_over_msg()
        self.init_error_msg()
        
    def reset_signal(self):
        """
        Function called to reset all value for processing the cube
        """
        self.process_picknplace = "false"
        self.cancel = False
        self.detection_open = False
        self.is_process = False
        self.game_over_open = False
        self.init_is_open = False
        self.close_wip()

    #####################################################
    ######## Handle of initialisation pop up ############
    #####################################################

    def init_intialisation_msg(self):
        """
        Initialization of the pop-up when reset/Initialisation
        """
        self.initialisation_msg = QMessageBox()
        self.initialisation_msg.setIcon(QMessageBox.Warning)
        self.initialisation_msg.setWindowTitle("Home")
        self.initialisation_msg.setText("The robot will go to secured pose. Is the working space cleared?")
        self.initialisation_msg.setStandardButtons(QMessageBox.Cancel | QMessageBox.Ok)
        self.initialisation_msg.setDefaultButton(QMessageBox.Ok)

        self.initialisation_msg.buttonClicked.connect(self.initialisation_popup_clicked)

        self.init_is_open = False
    
    def initialisation_pop_up(self):
        """
        Function called when the signal initialisation_signal is emitted
        """
        self.close_wip()
        self.init_is_open = True
        self.initialisation_msg.exec_()

    def initialisation_popup_clicked(self):
        self.reset_signal()
        # Service of pickandplace.py
        rospy.wait_for_service("/set_robot_state")
        try: 
            player = rospy.ServiceProxy("/set_robot_state", StateService)
            player(0,0,0,1) if (self.home) else player(0,0,1,0)
        except rospy.ServiceException as e:
            print("Service call failed: ",e)


    ########################################################### 
    ######## Handle of the Work in progress pop up ############
    ###########################################################       
    def init_wip(self):
        """
        Initialization of the pop up work in progress
        """
        # Center the window
        self.work_in_progress_msg.move(QApplication.desktop().screen().rect().center()- self.work_in_progress_msg.rect().center())
        # Set a fixed size
        self.work_in_progress_msg.setFixedSize(605, 125)
        # Set the value of the progress bar
        self.work_in_progress_msg.progress.setValue(0)

        self.wip_is_open = False

    def update_wip(self, new_value):
        self.work_in_progress_msg.progress.setValue(new_value)
        if (new_value == 100):
            self.close_wip()
    
    def close_wip(self):
        self.wip_is_open = False
        self.work_in_progress_msg.close()

    def work_in_progress(self):
        """
        Function called when the progess bar msg from pick and place is received
        """
        self.wip_is_open = True
        self.work_in_progress_msg.show()  

    ################################################################## 
    ######## Handle of the Continue/block detected pop up ############
    ##################################################################

    def init_detection_msg(self):
        """
        Initialization of the pop-up when a block is detected
        """
        self.detection_msg = QMessageBox()
        self.detection_msg.setIcon(QMessageBox.Question)
        self.detection_msg.setWindowTitle("Block detected")
        self.detection_msg.setText("A cube is detected. Do you want to process it?")
        self.detection_msg.setStandardButtons(QMessageBox.Cancel | QMessageBox.Ok)
        self.detection_msg.setDefaultButton(QMessageBox.Cancel)
        self.detection_msg.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.detection_msg.buttonClicked.connect(self.detection_popup_clicked)
        
        self.detection_open = False
        self.cancel = False

    
    def detection_pop_up(self):
        """
        Function called when the signal detection_signal is emitted
        """
        self.detection_open = True
        self.detection_msg.exec_()

    def detection_popup_clicked(self, i):
        """
        Function called when a button is clicked
        """
        if (i.text() == "&OK"): 
            self.process_picknplace = "true"
            self.is_process = True
            self.work_in_progress()
            
            
        elif (i.text()=="&Cancel"):
            
            self.process_picknplace = "false"
            self.cancel = True
            self.signal.pause_popup_signal.emit()

        self.detection_open = False
    
    
    def detection_callback(self, data):
        """
        The function called when the node receives a new block_detect msg
        """
        #print("Block pose receives : ", data)
        #self.process_picknplace = "false"
        self.block_dectect = data.data
        if (self.clock.is_over() and not(self.game_over_open)):
            self.first_click_play = True
            self.signal.game_over_signal.emit()

        elif (self.block_dectect == "no_block_detected" and not(self.is_process) and self.process_picknplace=="false" 
        and not(self.cancel) and not(self.detection_open) and self.start_game and not(self.game_over_open)
        and not(self.init_is_open) and not(self.error_open)):
            self.signal.error_signal.emit()

        elif (self.block_dectect == "block_detected" and self.is_process == False and self.process_picknplace=="false" 
        and self.cancel == False and self.detection_open==False and self.start_game == True and not(self.game_over_open)
        and not(self.init_is_open)):
                if (self.error_open):
                    self.error_close()
                self.error_open = False
                self.signal.detection_signal.emit()

        self.process_picknplace_pub.publish(self.process_picknplace)
    
    ######################################################### 
    ######## Handle of the Error detected pop up ############
    #########################################################

    def init_error_msg(self):
        """
        Initialization of the pop-up when a block is detected
        """
        self.error_msg = QMessageBox()
        self.error_msg.setIcon(QMessageBox.Question)
        self.error_msg.setWindowTitle("Robot Information")
        
        self.error_msg.setStandardButtons(QMessageBox.Cancel | QMessageBox.Ok)
        self.error_msg.setDefaultButton(QMessageBox.Ok)
        self.error_msg.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.error_msg.buttonClicked.connect(self.error_popup_clicked)
        
        self.error_open = False
        self.cancel = False

    def error_close(self):
        self.error_msg.close()

    def error_popup(self):
        if (self.game_state.error == "true"):
            self.error_msg.setText("The robot is not in secured pose. Do you still want to continue?")
        else:
            self.error_msg.setText("The robot is in secured pose. Do you want to continue?")
        self.close_wip()
        self.error_open = True
        self.error_msg.exec_()
    
    def error_popup_clicked(self, i):
        if (i.text() == "&OK"): 
            pass
            
        elif (i.text()=="&Cancel"):
            
            self.process_picknplace = "false"
            self.cancel = True
            self.signal.pause_popup_signal.emit()
    
    ################################################# 
    ######## Handle of the Cancel pop up ############
    #################################################

    def init_pause_msg(self):
            """
            Pop up when cancel is clicked
            """
            self.pause_msg = QMessageBox()
            self.pause_msg.setWindowTitle("Cancel")
            self.pause_msg.setIcon(QMessageBox.Question)
            self.pause_msg.setText("You clicked Cancel. Do you want to pause?")
            self.pause_msg.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
            self.pause_msg.setDefaultButton(QMessageBox.No)

            self.pause_msg.buttonClicked.connect(self.pause_popup_clicked)
    
    def pause_pop_up(self):
            """
            Function called when the user awnser no to process the cube
            """
            self.close_wip()
            self.pause_msg.exec_()
                
    def pause_popup_clicked(self,i):
        print(i.text())
        if (i.text() == "&Yes"):
            self.pause_ui()
        else:
            self.cancel = False

    ####################################################
    ######## Handle of the Game over pop up ############
    ####################################################
    
    def init_game_over_msg(self):
        """
        Pop up when the clock is over
        """
        self.gameover_msg = QMessageBox()
        self.gameover_msg.setWindowTitle("Game Over")
        self.gameover_msg.setIcon(QMessageBox.Information)
        self.detection_msg.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.gameover_msg.setStandardButtons(QMessageBox.Ok)
        self.gameover_msg.setDefaultButton(QMessageBox.Ok)

        self.game_over_open = False

        self.gameover_msg.buttonClicked.connect(self.stop_ui)

    def game_over_pop_up(self):
        """
        Open when signal for game over is emitted
        """
        if (self.game_state.score < self.record_value):
            text = "Congratulations ! You scored " + str(self.game_state.score) + " points"
        else:
            text = "Congratulations ! You scored " + str(self.game_state.score) + " points, It's a new record !!"
        self.gameover_msg.setText(text)
        self.game_over_open = True
        self.gameover_msg.exec_()


class MySignal(QObject):
    detection_signal = pyqtSignal()
    error_signal = pyqtSignal()
    pause_popup_signal = pyqtSignal()
    init_signal = pyqtSignal()
    game_over_signal = pyqtSignal()

if (__name__ == '__main__'):
	app = QApplication(sys.argv)
	myWindow = MyPlugin()
	myWindow._widget.show()
    
    # sys.exit(app.exec_())
    #rospy.sleep(1)
    #myWindow.update_score(2)
	