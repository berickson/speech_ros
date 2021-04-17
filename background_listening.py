#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class

import time
import speech_recognition as sr

# called when audio detected
def callback(recognizer, audio):
    

    try:
        # to use another API key, use `recognizer.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        utterance = recognizer.recognize_google(audio)
        print(f'"{utterance}"')
    except sr.UnknownValueError:
        print("no words detected")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))


r = sr.Recognizer()
m = sr.Microphone()

# set microphone to adjust senstivity automatically as background noise changes
#sr.dynamic_energy_threshold = True
with m as source:
    r.adjust_for_ambient_noise(source)
stop_listening = r.listen_in_background(m, callback)
print("Listening in the background")

while True:
    time.sleep(0.1)  # we're still listening even though the main thread is doing other things

stop_listening(wait_for_stop=False)

print("Done")