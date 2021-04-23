#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class

import time
import speech_recognition as sr

import roslib
import rospy
import pyttsx3
import os

from gtts import gTTS

use_gtts = True


from std_msgs.msg import String

rospy.init_node("speech", anonymous=True)
speech_publisher = rospy.Publisher("/speech/utterances",String, queue_size = 1)
speech_engine = pyttsx3.init() # object creation


# called when audio detected
def listen_callback(recognizer, audio):
    try:
        # to use another API key, use `recognizer.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        utterance = recognizer.recognize_google(audio)
        rospy.loginfo(f'heard: "{utterance}"')
        speech_publisher.publish(utterance)
    except sr.UnknownValueError:
        print("no words detected")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def say(text):
    rospy.loginfo(f'saying "{text}"')

    if use_gtts:
        tts = gTTS("uh " + text)
        tts.save('out.mp3')
    else:
        speech_engine.save_to_file("uh "+text, "out.mp3")
        speech_engine.runAndWait()
    rospy.loginfo(f'mp3 done')
    speaker_volume_percent = rospy.get_param("/speaker_volume_percent", 5.0)
    os.system(f"play --no-show-progress --volume {speaker_volume_percent / 100.0} out.mp3")
    os.system("rm out.mp3")


r = sr.Recognizer()
m = sr.Microphone()

# set microphone to adjust senstivity automatically as background noise changes
#sr.dynamic_energy_threshold = True
with m as source:
    r.adjust_for_ambient_noise(source)
stop_listening = r.listen_in_background(m, listen_callback, phrase_time_limit = 5.0)
print("Listening in the background")

def say_callback(ros_string):
    global stop_listening
    global r
    global m
    stop_listening(wait_for_stop=True)
    say(ros_string.data)
    stop_listening = r.listen_in_background(m, listen_callback)

rospy.Subscriber("speech/say", String, say_callback)

rospy.spin()

stop_listening(wait_for_stop=False)

print("Done")