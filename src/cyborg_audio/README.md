# README
This repository contains the source code for the NTNU Cyborg's Audio Module (ROS Node).

Node name: cyborg_audio
Language: Python

## Requirements:
* vlc
* pyttsx3

## Features:
* Playback, activated by publishing on topic cyborg_audio/playback. Playback can be preempted by publishing "PreemptPlayback" on the same topic. Executional feedback is provided on topic cyborg_audio/feedback_playback.
* Text to speech, activated by publishing on topic cyborg_audio/text_to_speech. Execution can be preempted by publishing "PreemptUtterance" on the same topic. Executional feedback is provided on topic cyborg_audio/feedback_text_to_speech.

## Usage:
$ rosrun cyborg_audio audio.py