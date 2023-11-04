#!/usr/bin/python3
#
#	@section COPYRIGHT
#	Copyright (C) 2021 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of "The Software", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	"The Software" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of "The
#	Software" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of "The
#	Software".
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	

import rospy
from std_msgs.msg import UInt16MultiArray, Int16MultiArray

import time
import sys
import os
import numpy as np
import wave, struct

import miro2 as miro

#speech recognition imports
import speech_recognition as sr
import whisper

#for latency timing
import time

#chat-gpt
import openai
openai.api_key = "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"  

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 4000 samples will
# buffer for half of a second, for instance.
BUFFER_STUFF_SAMPLES = 4000

# messages larger than this will be dropped by the receiver,
# however, so - whilst we can stuff the buffer more than this -
# we can only send this many samples in any single message.
MAX_STREAM_MSG_SIZE = (4096 - 48)

# using a margin avoids sending many small messages - instead
# we will send a smaller number of larger messages, at the cost
# of being less precise in respecting our buffer stuffing target.
BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

# how long to record before playing back in seconds?
RECORD_TIME = 5

# microphone sample rate (also available at miro2.constants)
MIC_SAMPLE_RATE = 20000

# sample count
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE

HOUNDIFY_CLIENT_ID = #insert Houndify client ID
HOUNDIFY_CLIENT_KEY = #insert client key

################################################################
def speech_recognition():

	print("Recognising speech...")
	print("whisper speech recognition")
	whisper_time = time.time()
	model = whisper.load_model("base")
	result = model.transcribe("/tmp/client_audio.wav")
	print(result)
	whisper_time = time.time()-whisper_time
	print("whisper time is ")
	print(whisper_time)
	
	speech_recog_setup_time = time.time()
	#creates Recognizer instance
	r = sr.Recognizer()
	#opens and reads the file in tmp/client_audio.wav as an audiofile type
	recorded_file = sr.AudioFile('/tmp/client_audio.wav')
	#uses this content as an instance called source
	with recorded_file as source:
		#changes the data to an audiodata instance through the method record
		audio = r.record(source)
	speech_recog_setup_time = time.time() - speech_recog_setup_time
	#google
	print("google speech recognition")
	google_time = time.time()
	google_prompt  = r.recognize_google(audio)
	print("google time is ")
	print(google_prompt)
	google_time = time.time()-google_time + speech_recog_setup_time
	print(google_time)
	#sound hound
	print("sound hound speech recognition")
	sound_hound_time = time.time()
	sound_hound_prompt  = r.recognize_houndify(audio, client_id= HOUNDIFY_CLIENT_ID, client_key=HOUNDIFY_CLIENT_KEY)
	print(sound_hound_prompt)
	sound_hound_time = time.time()-sound_hound_time + speech_recog_setup_time
	print("sound hound time is ")
	print(sound_hound_time)
	#sphinx
	print("sphinx speech recognition")
	sphinx_time = time.time()
	sphinx_prompt  = r.recognize_sphinx(audio)
	print(sphinx_prompt)
	sphinx_time = time.time()-sphinx_time + speech_recog_setup_time
	print("sphinx time is ")
	print(sphinx_time)
	

		
	
#	confirm_prompt = input("Prompt is "+prompt+". Ask chat? (Y/n)")
#	print(confirm_prompt)
#	if confirm_prompt == "Y":
#		print("asking chat-gpt "+prompt+". Please wait...")
#		completion = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=[{"role": "user", "content": prompt}])
#		print(completion.choices[0].message.content)
#	else:
#		print("Close and reopen program to retry")
	
	
################################################################

################################################################

def error(msg):
	print(msg)
	sys.exit(0)

################################################################



################################################################

class client:

	def callback_stream(self, msg):

		self.buffer_space = msg.data[0]
		self.buffer_total = msg.data[1]
		self.buffer_stuff = self.buffer_total - self.buffer_space

	def callback_mics(self, msg):

		# if recording
		if not self.micbuf is None:

			# append mic data to store
			self.micbuf = np.concatenate((self.micbuf, msg.data))

			# report
			sys.stdout.write(".")
			sys.stdout.flush()

			# finished recording?
			if self.micbuf.shape[0] >= SAMPLE_COUNT:

				# end recording
				self.outbuf = self.micbuf
				self.micbuf = None
				print (" OK!")

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# if recording finished
			if not self.outbuf is None:
				break

			# state
			time.sleep(0.02)

		# write output file
		outfilename = '/tmp/client_audio.wav'
		file = wave.open(outfilename, 'wb')
		file.setsampwidth(2)
		file.setframerate(MIC_SAMPLE_RATE)


		print ("writing two channels to file (LEFT and RIGHT)...")
		file.setnchannels(2)
		x = np.reshape(self.outbuf[:, [0, 1]], (-1))
		for s in x:
			file.writeframes(struct.pack('<h', s))

		# close file
		file.close()
		print ("wrote output file at", outfilename)
		speech_recognition()
	

	def __init__(self):

		# Create robot interface
		self.interface = miro.lib.RobotInterface()

		# state
		self.micbuf = np.zeros((0, 4), 'uint16')
		self.outbuf = None
		self.buffer_stuff = 0
		self.playchan = 0
		self.playsamp = 0

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/control/stream"
		print ("publish", topic)
		self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

		# subscribe
		topic = topic_base_name + "/sensors/stream"
		print ("subscribe", topic)
		self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)

		#subscribe to mics using robot Interface
		self.interface.register_callback("microphones", self.callback_mics)

		# report
		print ("recording from 4 microphones for", RECORD_TIME, "seconds...")

if __name__ == "__main__":

	main = client()
	main.loop()
