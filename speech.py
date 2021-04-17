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
        print(f'"{utterance}"')
        say(f"I heard you say {utterance}")
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
    os.system("play -v0.3 out.mp3")
    os.system("rm out.mp3")

def say_callback(ros_string):
    say(ros_string.data)

r = sr.Recognizer()
m = sr.Microphone()

# set microphone to adjust senstivity automatically as background noise changes
#sr.dynamic_energy_threshold = True
with m as source:
    r.adjust_for_ambient_noise(source)
stop_listening = r.listen_in_background(m, listen_callback)
print("Listening in the background")

rospy.Subscriber("speech/say", String, say_callback)

rospy.spin()

stop_listening(wait_for_stop=False)

print("Done")