# this is a simple message that adds a name to an image
# this is so that multiple cameras can publish their images
# over one topic which is required since many rovers have many 
# cameras and it is idfficult to have so many topic subscriptions
# over rosbridge to the web page
string name
sensor_msgs/CompressedImage image
