import time
import sys
import os
import numpy as np
import wave, struct


#speech recognition imports
import speech_recognition as sr
import whisper

#for latency timing
import time

#chat-gpt
import openai
openai.api_key = #insert your API key

################################################################
def speech_recognition():

	print("Recognising speech...")
	print("tiny whisper speech recognition")
	whisper_time = time.time()
	model = whisper.load_model("tiny")
	result = model.transcribe("harvard.wav") #CHANGE TO YOUR DIRECTORY
	print(result)
	whisper_time = time.time()-whisper_time
	print("whisper tiny model time is ")
	print(whisper_time)
	print("base whisper speech recognition")
	whisper_time = time.time()
	model = whisper.load_model("base")
	result = model.transcribe("harvard.wav") #CHANGE TO YOUR DIRECTORY
	print(result)
	whisper_time = time.time()-whisper_time
	print("whisper base model time is ")
	print(whisper_time)
	print("small whisper speech recognition")
	whisper_time = time.time()
	model = whisper.load_model("small")
	result = model.transcribe("harvard.wav") #CHANGE TO YOUR DIRECTORY
	print(result)
	whisper_time = time.time()-whisper_time
	print("whisper small model time is ")
	print(whisper_time)


def error(msg):
	print(msg)
	sys.exit(0)



if __name__ == "__main__":
	speech_recognition()

