#!/usr/bin/env python
version = "version 1.00"

"""
speed-track written by Claude Pageau pageauc@gmail.com
Raspberry (Pi) - python opencv2 Speed tracking using picamera module

This is a raspberry pi python opencv2 speed tracking demonstration program.
It will detect speed in the field of view and use opencv to calculate the
largest contour and return its x,y coordinate.  The image is tracked for
a specified threshold length and then the speed is calculated.
Note: Variables for this program are stored in speed_settings.py
  
Some of this code is based on a YouTube tutorial by
Kyle Hounslow using C here https://www.youtube.com/watch?v=X6rPdRZzgjg

Here is a my YouTube video demonstrating a previous speed tracking demo
program using a Raspberry Pi B2 https://youtu.be/09JS7twPBsQ

Requires a Raspberry Pi with a RPI camera module installed and configured

Install Dependencies from a logged in SSH session per quotends below

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y python-opencv python-picamera python-image python-pyexiv2

"""

print("-------------------------------------------------------------------------------------------------")
print("speed-track.py %s using python2 and OpenCV2    written by Claude Pageau" % ( version ))
print("Loading Please Wait ....")

import os
mypath=os.path.abspath(__file__)       # Find the full path of this python script
baseDir=mypath[0:mypath.rfind("/")+1]  # get the path location only (excluding script name)
baseFileName=mypath[mypath.rfind("/")+1:mypath.rfind(".")]
progName = os.path.basename(__file__)
progNameVer = "%s %s" %(progName, version)

import Adafruit_PWM_Servo_Driver as i2cpwm
import io
import time
import datetime
import cv2
import pyexiv2
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw
from picamera.array import PiRGBArray
from picamera import PiCamera

import logging
import threading
from Queue import Queue

logger = logging.getLogger(__name__)
logset = logging.getLogger(__name__ + '.settings')
logcal = logging.getLogger(__name__ + '.calibration')
logfps = logging.getLogger(__name__ + '.fps')
logfile = logging.getLogger(__name__ + '.file')

PI = 3.141526

# Check for variable file to import and error out if not found.
configFilePath = baseDir + "speed_settings.py"
if not os.path.exists(configFilePath):
    print("ERROR - Missing speed_settings.py file - Could not import Configuration file %s" % (configFilePath))
    quit()
else:
    # Read Configuration variables from config.py file
    from speed_settings import *

# Calculate conversion from camera pixel width to actual speed.
px_to_mph = float(( CAMERA_WIDTH / IMAGE_VIEW_FT ) * 5280 / 3600)

if SPEED_MPH:
    speed_units = "mph"
    speed_conv = 1.0 * px_to_mph
else:
    speed_units = "kph"
    speed_conv = 1.609344 * px_to_mph

quote = '"'  # Used for creating quote delimited log file of speed data
    
#----------------------------------------------------------------------------------------------
def get_fps( start_time, frame_count ):
    # Calculate and display frames per second processing
    if frame_count >= 100:
        duration = float( time.time() - start_time )
        FPS = float( frame_count / duration )
        logfps.info("Processing Speed Motion Stream at %(FPS).2f fps for Last %(frame_count)i Frames" %locals())
        frame_count = 0
        start_time = time.time()
    else:
        frame_count += 1
    return start_time, frame_count   
    
#-----------------------------------------------------------------------------------------------
def show_settings():
    settings = (
        '',
        'Note: To Send Full Output to File Use command -   python -u ./%(progName)s | tee -a log.txt',
        '      Set log_data_to_file=True to Send speed_Data to CSV File %(baseFileName)s.log',
        "-------------------------------------- Settings -------------------------------------------------",
        "",
        'Message Display . verbose=%(verbose)s  display_fps=%(display_fps)s calibrate=%(calibrate)s',
        "Logging ......... Log_data_to_file=%(log_data_to_file)s  log_filename=%(baseFileName)s.log (CSV format)",
        "                  Log if max_speed_over > %(max_speed_over)i %(speed_units)s",
        "Speed Trigger ... If  track_len_trig > %(track_len_trig)i px",
        "Exclude Events .. If  x_diff_min < %(x_diff_min)i or x_diff_max > %(x_diff_max)i px",
        "                  If  y_upper < %(y_upper)i or y_lower > %(y_lower)i px",
        "                  If  event_timeout > %(event_timeout)i seconds Start New Track",
        "                  If  max_speed_over < %(max_speed_over)i %(speed_units)s",
        "Speed Photo ..... Size=%(image_width)ix%(image_height)i px  WINDOW_BIGGER=%(WINDOW_BIGGER)i  VFlip=%(CAMERA_VFLIP)s  HFlip=%(CAMERA_HFLIP)s ",
        "                  image_path=%(image_path)s  image_Prefix=%(image_prefix)s  image_text_bottom=%(image_text_bottom)s",
        "Motion Settings . Size=%(CAMERA_WIDTH)ix%(CAMERA_HEIGHT)i px  IMAGE_VIEW_FT=%(IMAGE_VIEW_FT)i  speed_units=%(speed_units)s  VFlip=%(CAMERA_VFLIP)s  HFlip=%(CAMERA_HFLIP)s",
        "OpenCV Settings . MIN_AREA=%(MIN_AREA)i sq-px  BLUR_SIZE=%(BLUR_SIZE)i  THRESHOLD_SENSITIVITY=%(THRESHOLD_SENSITIVITY)i  CIRCLE_SIZE=%(CIRCLE_SIZE)i px",
        "                  gui_window_on=%(gui_window_on)s (Display OpenCV Status Windows on GUI Desktop)",
        "",
        "-------------------------------------------------------------------------------------------------",
    )
    logset.info('\n'.join(settings),globals())
  
def take_calibration_image(filename, cal_image):
    # Create a calibration image for determining value of IMG_VIEW_FT variable       
    cv2.line( cal_image,( 0,y_upper ),( CAMERA_WIDTH, y_upper ),(255,0,0), 1 )
    cv2.line( cal_image,( 0,y_lower ),( CAMERA_WIDTH, y_lower ),(255,0,0), 1 )
    for i in range ( 10, CAMERA_WIDTH - 9, 10 ):
        cv2.line( cal_image,( i ,y_upper - 5 ),( i, y_upper + 30 ),(255,0,0), 1 )
    calmsg = (
        "",
        "----------------------------------- Create Calibration Image --------------------------------------",
        "",
        "    Instructions for using %s image to calculate value for IMG_VIEW_FT variable" % ( filename ),
        "",
        "1 - Use a known size reference object in the image like a vehicle at the required distance.",
        "2 - Calculate the px to FT conversion using the reference object and the image y_upper marks at every 10 px",
        "3 - Calculate IMG_VIEW_FT per formula below See speed-track.md for details",
        "",
        "    IMG_VIEW_FT = (%(CAMERA_WIDTH)i * Ref_Obj_ft) / num_px_for_Ref_Object",
        "    eg. (%i * 18) / 80 = %.1f" % (CAMERA_WIDTH, (CAMERA_WIDTH * 18)/80),
        "",
        "4 - Update the IMG_VIEW_FT variable in the speed_settings.py file",
        "5 - Perform a test using a vehicle at a known speed to verify calibration.",
        "6 - Make sure y_upper and y_lower variables are correctly set for the roadway to monitor",
        "",
        "    Calibration Image Saved To %s%s" % ( baseDir, filename, ),
        "",
        "---------------------------- Press cntl-c to Quit Calibration Mode --------------------------------",
        "",
    )

    logcal.info('\n'.join(calmsg),globals())
    return cal_image
  
#-----------------------------------------------------------------------------------------------       
def get_image_name(path, prefix):
    # build image file names by number sequence or date/time
    rightNow = datetime.datetime.now()
    filename = "%s/%s%04d%02d%02d-%02d%02d%02d.jpg" % ( path, prefix ,rightNow.year, rightNow.month, rightNow.day, rightNow.hour, rightNow.minute, rightNow.second )     
    return filename  

          
#-----------------------------------------------------------------------------------------------
def image_write(image_filename, text_to_print):
    # function to write date/time stamp directly on top or bottom of images.
    FOREGROUND = ( 255, 255, 255 )  # rgb settings for white text foreground
    text_colour = "White"
    font_size = 20

    # centre text and compensate for graphics text being wider
    x =  int(( image_width / 2) - (len( text_to_print ) * font_size / 4) )
    if image_text_bottom:
        y = ( image_height - 50 )  # show text at bottom of image 
    else:
        y = 10  # show text at top of image
    TEXT = text_to_print
    font_path = '/usr/share/fonts/truetype/freefont/FreeSansBold.ttf'
    font = ImageFont.truetype( font_path, font_size, encoding='unic' )
    text = TEXT.decode( 'utf-8' )

    # Read exif data since ImageDraw does not save this metadata
    metadata = pyexiv2.ImageMetadata(image_filename) 
    metadata.read()
    
    img = Image.open( image_filename )
    draw = ImageDraw.Draw( img )
    # draw.text((x, y),"Sample Text",(r,g,b))
    draw.text( ( x, y ), text, FOREGROUND, font=font )
    img.save( image_filename )
    metadata.write()    # Write previously saved exif data to image file       
    msgStr = " Image Saved - " + text_to_print
    logger.info('image_write - %s', msgStr)
    return
         
#----------------------------------------------------------------------------------------------    
def speed_camera():
    show_settings()
    if verbose:
        if calibrate:
            logcal.info('In Calibration Mode ....')
        if gui_window_on:
            print("Press lower case q on OpenCV GUI Window to Quit program")
            print("or ctrl-c in this terminal session to Quit")
        else:
            print("Press ctrl-c in this terminal session to Quit")
        print("")         

    msg_str = "Initializing Pi Camera ...."
    logger.info('speed_camera - %s', msg_str)

    frameQueue = Queue()
    angleQueue = Queue()
    angleQueue.put(0)

    workerThread = MotionDetection(frameQueue, angleQueue)
    workerThread.daemon = True
    workerThread.start()

    frameGrabber = FrameGrabber(frameQueue, angleQueue)
    frameGrabber.daemon = True
    frameGrabber.start()

    global shutdownEvent
    shutdownEvent = threading.Event()

    msgStr = "Start Speed Motion Tracking"
    logger.info('speed_camera - %s', msgStr)

    while True:
        time.sleep(1)

class ServoController(object):
    waittime = 1

    def __init__(self, initialAngle = 0 ):
        self.pwm = i2cpwm.PWM(pwm_address)
        self.pwm.setPWMFreq(pwm_freq)
        self.angle = None
        self.setCameraAngle(initialAngle)

    def setCameraAngle(self, angle):
        '''set the camera angle in radians [-pi/2,pi/2]'''
        if self.angle != angle:
            pulse = int((angle/PI+.5)*(servo_left-servo_right)+servo_right)
            self.pwm.setPWM(servo_pin, 0, pulse)
            self.angle = angle
            time.sleep(self.waittime)

class FrameGrabber(threading.Thread):
    def __init__(self, frameQueue, angleQueue):
        threading.Thread.__init__(self)
        self.frameQueue = frameQueue
        self.angleQueue = angleQueue
        self.camera = PiCamera()
        self.camera.hflip = CAMERA_HFLIP
        self.camera.vflip = CAMERA_VFLIP          
        self.camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
        self.camera.framerate = CAMERA_FRAMERATE
        self.rawCapture = PiRGBArray(self.camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))
        self.startCaptureTime = time.time()
        self.sc = ServoController()

    def run(self):
        # allow the camera to warmup
        runtime = time.time()
        sleeptime = min(runtime - self.startCaptureTime,1)
        time.sleep(sleeptime)

        # capture frames from the camera
        while not shutdownEvent.isSet():
            angle = self.angleQueue.get(block=True)
            self.sc.setCameraAngle(angle)
            ts = time.time()
            self.camera.capture(self.rawCapture, format='bgr', use_video_port=True)
            self.frameQueue.put((ts,self.rawCapture.array,angle))
            self.rawCapture.truncate(0)

class MotionDetection(threading.Thread):
    def __init__(self, frameQueue, angleQueue):
        threading.Thread.__init__(self)
        self.frameQueue = frameQueue
        self.angleQueue = angleQueue

    def run(self):
        ave_speed = 0.0
        frame_count = 0
        fps_time = time.time()
        first_event = True   # Start a New Motion Track      
        event_timer = time.time()
        start_pos_x = 0
        end_pos_x = 0

        # This is the first time through the loop so initialize grayimage1
        # Only needs to be done once                    
        self.angleQueue.put(0)
        imagetime, image2, angle = self.frameQueue.get(block=True)
        grayimage1 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        event_timer = time.time()
        # Initialize prev_image used for taking speed image photo
        prev_image = image2

        while not shutdownEvent.isSet():
            self.angleQueue.put(0)
            imagetime, image2, angle = self.frameQueue.get(block=True)

            if time.time() - event_timer > event_timeout:   # Check if event timed out
                # event_timer exceeded so reset for new track             
                event_timer = time.time()
                first_event = True
                start_pos_x = 0
                end_pos_x = 0
            
            if display_fps:   # Optionally show motion image processing loop fps 
                fps_time, frame_count = get_fps( fps_time, frame_count )
            
            # initialize variables          
            motion_found = False
            biggest_area = MIN_AREA
            cx = 0
            cy = 0
            cw = 0
            ch = 0

            # Convert to gray scale, which is easier
            grayimage2 = cv2.cvtColor( image2, cv2.COLOR_BGR2GRAY )

            # Get differences between the two greyed images
            differenceimage = cv2.absdiff( grayimage1, grayimage2 )

            # Blur difference image to enhance motion vectors
            differenceimage = cv2.blur( differenceimage,(BLUR_SIZE,BLUR_SIZE ))

            # Get threshold of blurred difference image based on THRESHOLD_SENSITIVITY variable
            retval, thresholdimage = cv2.threshold( differenceimage,THRESHOLD_SENSITIVITY,255,cv2.THRESH_BINARY )

            # Get all the contours found in the threshold image
            contours, hierarchy = cv2.findContours( thresholdimage,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE )
            total_contours = len( contours )

            # Update grayimage1 to grayimage2 ready for next image2
            grayimage1 = grayimage2
        
            # find contour with biggest area
            for c in contours:
                # get area of next contour
                found_area = cv2.contourArea(c)
                if found_area > biggest_area:
                    motion_found = True
                    biggest_area = found_area
                    ( x, y, w, h ) = cv2.boundingRect(c)
                    cx = x + w/2   # put circle in middle of width
                    cy = y + h/2   # put circle in middle of height
                    cw = w
                    ch = h
                
            if motion_found:
                if cy > y_upper and cy < y_lower:      # movement is within valid y range
                    if first_event:   # This is the first valid motion event
                        first_event = False
                        start_pos_x = cx
                        end_pos_x = cx
                        track_start_time = time.time()                                   
                        msgStr = "New Track    - Motion at cx=%3i cy=%3i Area=%i  %i contours" % ( cx, cy, biggest_area, total_contours )
                        logger.info('speed_camera - %s', msgStr)
                    else:
                        if ( abs( cx - end_pos_x ) > x_diff_min and abs( cx - end_pos_x ) < x_diff_max ):
                            # movement is within acceptable distance range of last event
                            end_pos_x = cx
                            leftright = ((end_pos_x - start_pos_x) > 0)
                            tot_track_dist = abs( end_pos_x - start_pos_x )
                            tot_track_time = abs( time.time() - track_start_time )
                            ave_speed = float((abs( tot_track_dist / tot_track_time)) / speed_conv)
                            if tot_track_dist > track_len_trig:
                                # Track length exceeded so take process speed photo                                     
                                if ave_speed > max_speed_over or calibrate:
                                    if leftright:
                                        self.angleQueue.put(-PI/4)
                                    else:
                                        self.angleQueue.put(PI/4)
                                    # Resized and process prev image before saving to disk
                                    angle = 0
                                    while angle == 0:
                                        self.angleQueue.put(angle) 
                                        ts,prev_image,angle = self.frameQueue.get()

                                    if calibrate:       # Create a calibration image                                             
                                        filename = get_image_name( image_path, "calib-" )                                               
                                        prev_image = take_calibration_image( filename, prev_image )                    
                                    else:
                                        speed_prefix = str(int(round(ave_speed))) + "-" + image_prefix                                               
                                        filename = get_image_name( image_path, speed_prefix)
                                    big_image = cv2.resize(prev_image,(image_width, image_height))                                            
                                    cv2.imwrite(filename, big_image)
                                    msgStr = " Event Add   - Motion at cx=%3i cy=%3i Dist=%i SPEED %.1f %s track_len=%3i of %i px  Area=%i sq-px  %i contours" % ( cx, cy, abs( end_pos_x - start_pos_x ), ave_speed, speed_units, abs( start_pos_x - end_pos_x), track_len_trig, biggest_area, total_contours )
                                    logger.info('speed_camera - %s', msgStr)
                                    # Format and Save Data to CSV Log File
                                    log_time = datetime.datetime.now()                                               
                                    log_csv_time = "%s%04d%02d%02d%s,%s%02d%s,%s%02d%s" % ( quote, log_time.year, log_time.month, log_time.day, quote, quote, log_time.hour, quote, quote, log_time.minute, quote)                                          
                                    # Add Text to image                                                
                                    image_text = "SPEED %.1f %s - %s" % ( ave_speed, speed_units, filename )
                                    image_write( filename, image_text )
                                    logfile.info('',extra={'speed':ave_speed,'unit':speed_units,'photopath':filename,'width':cw,'height':ch,'area':cw*ch})
                                    msgStr = "End Track    - Tracked %i px in %.1f sec  Area=%i sq-px %i contours" % ( tot_track_dist, tot_track_time, biggest_area, total_contours )
                                    logger.info('speed_camera - %s', msgStr)
                                else:
                                    msgStr = "End Track    - Skip Photo SPEED %.1f %s max_speed_over=%i  %i px in %.1f sec  Area=%i sq-px  %i contours" % ( ave_speed, speed_units, max_speed_over, tot_track_dist, tot_track_time, biggest_area, total_contours )
                                    logger.info('speed_camera - %s', msgStr)
                                # Reset Variables for next cycle through loop
                                start_pos_x = 0
                                end_pos_x = 0
                                first_event = True                         
                            else:
                                msgStr = " Event Add   - Motion at cx=%3i cy=%3i Dist=%i SPEED %.1f %s track_len=%3i of %i px  Area=%i sq-px  %i contours" % ( cx, cy, abs( cx - end_pos_x ), ave_speed, speed_units, abs( start_pos_x - end_pos_x), track_len_trig, biggest_area, total_contours )
                                end_pos_x = cx
                                logger.info('speed_camera - %s', msgStr)
                            prev_image = image2
                        else:
                            msgStr = " Out Range   - ByPass at cx=%3i cy=%3i Dist=%i is <%i or >%i px  Area=%i sq-px  %i contours" % ( cx, cy, abs( cx - end_pos_x ), x_diff_min, x_diff_max, biggest_area, total_contours  )                                    
                            logger.info('speed_camera - %s', msgStr)
                    
                    if gui_window_on:
                        # show small circle at motion location 
                        cv2.circle( image2,( cx,cy ),CIRCLE_SIZE,( 0,255,0 ), 2 )
                        if ave_speed > 0:
                            speed_text = str('%5.1f'  % ave_speed ) 
                            cv2.putText( image2, speed_text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)                           
                    event_timer = time.time()  # Reset event_timer since valid motion was found
            
            if gui_window_on:
                #cv2.imshow('Difference Image',differenceimage)                        
                cv2.line( image2,( 0, y_upper ),( CAMERA_WIDTH, y_upper ),(255,0,0),1 )
                cv2.line( image2,( 0, y_lower ),( CAMERA_WIDTH, y_lower ),(255,0,0),1 )
                big_w = CAMERA_WIDTH * WINDOW_BIGGER
                big_h = CAMERA_HEIGHT * WINDOW_BIGGER
                image2 = cv2.resize( image2,( big_w, big_h ))                         
                cv2.imshow('Threshold Image', thresholdimage)
                cv2.imshow('Movement Status', image2)
                # Close Window if q pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    print("End Motion Tracking ......")
                    shutdownEvent.set()
                    break

#-----------------------------------------------------------------------------------------------    
if __name__ == '__main__':

    datefmt = '%Y%m%d_%H:%M:%S'
    if verbose:
        logger.setLevel(logging.INFO)
        formatter = logging.Formatter(fmt='%(asctime)s %(message)s', datefmt=datefmt)
        streamHandler = logging.StreamHandler()
        streamHandler.setFormatter(formatter)
        logger.addHandler(streamHandler)

        logset.setLevel(logging.INFO)
        formatter = logging.Formatter('%(message)s')
        streamHandler = logging.StreamHandler()
        streamHandler.setFormatter(formatter)
        logset.propagate = False
        logset.addHandler(streamHandler)

    if log_data_to_file:
        logfile.setLevel(logging.INFO)
        log_file_path = baseDir + baseFileName + ".log"

        if not os.path.exists(log_file_path):
            with open(log_file_path,'ab') as f:
                f.write('"YYYYMMDD","HH","MM","Speed","Unit","    Speed Photo Path        ","W","H","Area"\n')

        fmt = '%(asctime)s "%(speed)s", "%(unit)s", "%(photopath)s", %(width)s, %(height)s, %(area)s'
        formatter = logging.Formatter(fmt=fmt, datefmt='"%Y%m%d","%H","%M","%S",')
        fileHandler = logging.FileHandler(log_file_path,mode='ab')
        fileHandler.setFormatter(formatter)
        logfile.addHandler(fileHandler)
        logfile.propagate = False
        
    if calibrate:
        logcal.setLevel(logging.INFO)
        formatter = logging.Formatter('%(message)s')
        streamHandler = logging.StreamHandler()
        streamHandler.setFormatter(formatter)
        logcal.propagate = False
        logcal.addHandler(streamHandler)

    if display_fps:
        logfps.setLevel(logging.INFO)
        formatter = logging.Formatter(fmt='%(asctime)s get_fps - %(message)s',datefmt=datefmt)
        streamHandler = logging.StreamHandler()
        streamHandler.setFormatter(formatter)
        logfps.propagate = False
        logfps.addHandler(streamHandler)

    try:
        speed_camera()
    except KeyboardInterrupt as e:
        pass
    finally:
        print("")
        print("+++++++++++++++++++++++++++++++++++")
        print("%s - Exiting Program" % progName)
        print("+++++++++++++++++++++++++++++++++++")
        print("")                           
                            


