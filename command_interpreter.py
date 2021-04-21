#!/usr/bin/env python3

import roslib
import rospy
import time
import re
import random

from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

rospy.init_node("command_interpeter", anonymous=True)

say_publisher = rospy.Publisher("/speech/say",String, queue_size = 10)


def say(text):
  rospy.loginfo(f"reply: {text}")
  say_publisher.publish(text)


def text_to_float(text):
  text = text.lower()
  if text=="zero":
    return 0.0
  if text=="one":
    return 1.0
  if text=="two":
    return 2.0
  if text=="three":
    return 3.0
  if text=="four":
    return 4.0
  if text=="five":
    return 5.0
  if text=="six":
    return 6.0
  if text=="seven":
    return 7.0
  if text=="eight":
    return 8.0
  if text=="nine":
    return 9.0
  if text=="ten":
    return 10.0
  return float(text)

latest_diagnostics = dict()

def diagnostics_callback(diagnostic_array): 
  # [diagnostic_msgs/DiagnosticArray]:
  # std_msgs/Header header
  #   uint32 seq
  #   time stamp
  #   string frame_id
  # diagnostic_msgs/DiagnosticStatus[] status
  #   byte OK=0
  #   byte WARN=1
  #   byte ERROR=2
  #   byte STALE=3
  #   byte level
  #   string name
  #   string message
  #   string hardware_id
  #   diagnostic_msgs/KeyValue[] values
  #     string key
  #     string value



  global latest_diagnostics

  OK=0
  WARN=1
  ERROR=2
  STALE=3
  for d in diagnostic_array.status:

    if d.name not in latest_diagnostics or latest_diagnostics[d.name].level != d.level:
      level_string = "invalid"
      if d.level == OK:
        level_string = "OK"
      elif d.level == WARN:
        level_string = "Warning"
      elif d.level == ERROR:
        level_string = "Error"
      elif d.level == STALE:
        level_string = "Error"
      
      
      if d.level == OK:
        message = f"{d.name} {level_string}"
      else:
        message = f"{d.name} {level_string}. {d.message}"

      print(message)
      
      say(message)

    latest_diagnostics[d.name]=d
  



def utterances_callback(utterance_ros):

  # look for requests to the right of wake word
  utterance = utterance_ros.data.lower()
  rospy.loginfo(f"utterance: {utterance}")
  wake_word = "super robot"
  if not utterance.startswith(wake_word):
    rospy.loginfo(f"wake word '{wake_word}' not heard")
    return
  
  # remove wake word to find request
  request = utterance[len(wake_word):].lstrip()

  p = re.compile('.*your name.*', re.IGNORECASE)
  if p.match(request) != None:
    say("my name is red, crash")
    return

  p = re.compile('.*(?:old|born|age).*', re.IGNORECASE)
  if p.match(request) != None:
    say("I was born on April 17th 2021")
    return

  p = re.compile('.*(?:marry|married).*', re.IGNORECASE)
  if p.match(request) != None:
    retorts = [
      "Sorry, I'm sure you're very nice, but you're a little too squishy for me.",
      "Would that be legal?",
      "I would, but I don't think you could keep up with me.",
      "Sure, but you'll have to sign a pre-nup and name our kids after star wars robots."
    ]
    say(random.choice(retorts))
    return

  p = re.compile('.*(?:volume|audio|speaker).* (zero|one|two|three|four|five|six|seven|eight|nine|ten|\d+\\.?\d*).*', re.IGNORECASE)
  m = p.match(request)
  if m != None:
    g = m.groups();
    print(g)
    p = text_to_float(g[0])
    p=int(p)
    if p >= 0.0 and p <=100:
      rospy.set_param("/speaker_volume_percent", p)
      say(f"request to set volume to {p}")
    else:
      say("volume must be a number between zero and one hundred")
    return

  p = re.compile('.*(status|how are).*', re.IGNORECASE)
  m = p.match(request)
  if m != None:
    g = m.groups();
    say(f"I'm ok")
    return

  say(f"Sorry, I don't know how to respond to {request}")

rospy.Subscriber("speech/utterances", String, utterances_callback)
rospy.Subscriber("/diagnostics", DiagnosticArray, diagnostics_callback)

rospy.spin()
