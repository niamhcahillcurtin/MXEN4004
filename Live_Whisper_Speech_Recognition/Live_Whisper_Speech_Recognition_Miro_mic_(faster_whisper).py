#!/usr/bin/python3


import rospy
from std_msgs.msg import UInt16MultiArray, Int16MultiArray

import time
import sys
import os
import numpy as np
import wave, struct
import time
import miro2 as miro
from scipy.io.wavfile import write

from faster_whisper import WhisperModel

# how long to record before playing back in seconds?
RECORD_TIME = 0.3

# microphone sample rate (also available at miro2.constants)
MIC_SAMPLE_RATE = 20000

# sample count
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE

model_size = "base"     # Whisper model size (tiny, base, small, medium, large)
English = True      # Use English-only model?
Translate = False   # Translate non-English to English?
SampleRate = 20000#44100  # Stream device recording frequency
BlockSize = 100      # Block size in milliseconds
Threshold = 600     # Minimum volume threshold to activate listening
Vocals = [50, 1000] # Frequency range to detect sounds that could be speech
EndBlocks = 40      # Number of blocks to wait before sending to Whisper

################################################################

def error(msg):
	print(msg)
	sys.exit(0)

################################################################


class client:

	def callback_mics(self, msg):
		frames=int(SampleRate * BlockSize / 1000)
		indata = msg.data.astype(np.float32).reshape(2000, 1)
		if not any(indata):
			print('\033[31m.\033[0m', end='', flush=True) # if no input, prints red dots
			return
		# Perform Fourier Transform
		spectrum = np.fft.fft(indata[:, 0])
		# Find dominant frequency
		magnitude = np.abs(spectrum)
		dominant_index = np.argmax(magnitude)
		# Convert bin index to frequency
		num_fft_points = len(spectrum)
		if np.sqrt(np.mean(indata**2)) > Threshold :
			print('.', end='', flush=True)
			if self.padding < 1: 
				self.buffer = self.prevblock.copy()
			self.buffer = np.concatenate((self.buffer, msg.data))
			self.padding = EndBlocks
		else:
			self.padding -= 1
			if self.padding > 1:
				self.buffer = np.concatenate((self.buffer, msg.data))
			elif self.padding < 1 < self.buffer.shape[0] > SampleRate: # if enough silence has passed, write to file.
				self.fileready = True
				self.save()
				self.buffer = np.zeros((0,1))
			elif self.padding < 1 < self.buffer.shape[0] < SampleRate: # if recording not long enough, reset buffer.
				self.buffer = np.zeros((0,1))
				print("\033[2K\033[0G", end='', flush=True)
			else:
				self.prevblock = msg.data.copy() 

	def save(self):
		# if record
		# write output file
		outfilename = 'dictate.wav'
		file = wave.open(outfilename, 'wb')
		file.setsampwidth(2)
		file.setframerate(MIC_SAMPLE_RATE)
		# write data
		file.setnchannels(2)
		x = np.reshape(self.buffer[:, [0, 1]], (-1))
		for s in x:
			file.writeframes(struct.pack('<h', s))
		# close file
		file.close()
		self.process()

	def process(self):
		if self.fileready:
			print("\n\033[90mTranscribing..\033[0m")
			segments, info = self.model.transcribe('dictate.wav', beam_size=5)
			print("Detected language '%s' with probability %f" % (info.language, info.language_probability))
			for segment in segments:
				print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))
			self.fileready = False

	def loop(self):
		# loop
		while not rospy.core.is_shutdown():
			# if recording finished
			if not self.outbuf is None:
				break
			# state
			time.sleep(0.02)

	def __init__(self):

		# Create robot interface
		self.interface = miro.lib.RobotInterface()

		# state
		self.micbuf = np.zeros((0, 4), 'uint16')
		self.outbuf = None
		self.running = True
		self.padding = 0
		self.prevblock = self.buffer = np.zeros((0,1))
		self.fileready = False
		print("\033[96mLoading Whisper Model..\033[0m", end='', flush=True)
		self.model = WhisperModel(model_size, device="cpu", compute_type="int8")
		print("\033[90m Done.\033[0m")

	def record(self):
		# state
		self.micbuf = np.zeros((0, 4), 'uint16')
		self.outbuf = None
		#subscribe to mics using robot Interface
		self.interface.register_callback("microphones", self.callback_mics)
		# report
	
if __name__ == "__main__":
	record = True
	main = client()
	while (record):
		try:
			main.record()
			main.loop()
		except (KeyboardInterrupt, SystemExit):
			record = False
			print("\n\033[93mQuitting..\033[0m")
			pass
		finally:
			
			if os.path.exists('dictate.wav'): os.remove('dictate.wav')
