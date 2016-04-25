# ---------------- User Configuration Settings for speed-track.py ---------------------------------

# Display and Log settings
verbose = True              # display basic status information on console
calibrate = False           # Create a speed_calibrate.jpg file with markers to calculate a px to FT conversion
display_fps = True         # show average frame count every 100 loops
gui_window_on = True       # Display desktop GUI openCV threshold window. Set to False if running from ssh console only.
#gui_window_on = False       # Display desktop GUI openCV threshold window. Set to False if running from ssh console only.
log_data_to_file = True     # save log data as CSV comma separated values
max_speed_over = 0          # 0=All or speed=Record incident only if greater than specified speed  

# Motion Event Settings
# ---------------------
track_len_trig  = 150  # Length of track to trigger speed photo
event_timeout = 2      # Number of seconds to wait between motion events before starting new track
IMAGE_VIEW_FT = 5     # Set the width in feet for the road width that the camera width sees
SPEED_MPH = True       # Set the speed conversion  kph = False  mph=True
x_diff_min = 3         # min px away from last motion event x pos
x_diff_max = 155       # max px away for last motion event x pos
y_upper = 90          # Exclude motion events above this point to top of image.
y_lower = 145          # Exclude motion events below this point to bottom of image.

# Motion Camera Settings
CAMERA_HFLIP = True    # Flip the camera image horizontally if required
CAMERA_VFLIP = True    # Flip the camera image vertically if required
CAMERA_WIDTH = 320     # Set the image stream width for opencv motion scanning default is 320
CAMERA_HEIGHT = 240    # Set the image stream height for opencv motion scanning  default is 240
CAMERA_FRAMERATE = 7  # Framerate of camera video frame capture default is 15
WINDOW_BIGGER = 3      # resize multiplier for speed photo image and if gui_window_on=True then makes opencv window bigger 

# Speed Photo Camera Image Settings
image_width  = CAMERA_WIDTH * WINDOW_BIGGER        # Set width of trigger point image to save 
image_height = CAMERA_HEIGHT * WINDOW_BIGGER       # Set height of trigger point image to save
image_path = "images"   # folder name to store images 
image_prefix = "speed-" # image name prefix
image_text_bottom = True  # True = Show image text at bottom otherwise at top

# OpenCV Motion Tracking Settings
MIN_AREA = 500     # excludes all contours less than or equal to this Area
CIRCLE_SIZE = 1    # diameter of circle to show motion location in window
BLUR_SIZE = 10     # OpenCV setting for Gaussian difference image blur 
THRESHOLD_SENSITIVITY = 25  # OpenCV setting for difference image threshold

#servo settings
pwm_address = 0x40 # The I2C address for the Adafruit PWM HAT
pwm_freq = 60      # The update frequency for the PWM chip
servo_pin = 0      # The ADAFruit PWM HAT pin for the camera servo
servo_left = 560   # The Pulse count for the extreme left angle on the servo
servo_right = 100  # The pulse count for the extreme right angle on the servo


#--------------------------- End of User Settings -------------------------------------------------
