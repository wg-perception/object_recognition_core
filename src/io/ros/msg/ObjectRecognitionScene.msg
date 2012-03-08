# This message type describes a potential scene configuration: a set of objects that can explain the scene
object_recognition_core/ObjectRecognition[] objects

##################################################### SEARCH ###########################################################

# The confidence is between 0 and 1: e.g. you have a technique that can find cans of soda
# but cannot differentiate between Coke and Sprite. If it only has those two in the DB, it will give a confidence of 0.5
# for each scene (each containing one or the other object)
float32 confidence
