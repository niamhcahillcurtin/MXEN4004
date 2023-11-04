# Speech recognition insipired by Nik Stromberg - nikorasu85@gmail.com - MIT 2022 - copilot
import openai
import time


openai.api_key = # insert your api key

messages=[
    {
        "role": "system",
        "content": "Try to keep your responses short (less than 30 words). You are a carer looking after an elderly person with dementia. You are going to guide them through an activity which is a part of their cognitive stimulation therapy. Your activity today will focus on their life now. You will use the following points to stimulate conversation about the person's current likes and dislikes. Only ask one question at a time! There are no right or wrong answers, and you can add questions of your own. Be kind and gentle. \n- What kinds of things do you like to talk about?\n- What do you like to eat and drink?\n- What is important to you about your appearance?\nE.g. clothes, hair, or nails.\n- What types of music do you like to listen to?\n- What types of radio channels or television programmes interest you?\n- Do you have any particular dislikes (food/ dress/\nactivities/conversation topics etc.)?\n\n"
    }
]

#!/usr/bin/env python3
import os
import numpy as np
import sounddevice as sd
import gtts
import whisper
from scipy.io.wavfile import write
from playsound import playsound

model_size = "base"  # Whisper model size (tiny, base, small, medium, large)
English = True      # Use English-only model?
Translate = False   # Translate non-English to English?
SampleRate = 20000  # Stream device recording frequency
BlockSize = 50      # Block size in milliseconds
Threshold = 0.005     # Minimum volume threshold to activate listening
Vocals = [50, 1000] # Frequency range to detect sounds that could be speech
EndBlocks = 20      # Number of blocks to wait before sending to Whisper

class StreamHandler:
    def __init__(self, assist=None):
        if assist == None:  # If not being run by my assistant, just run as terminal transcriber.
            class fakeAsst(): running, talking, analyze = True, False, None
            self.asst = fakeAsst()  # anyone know a better way to do this?
        else: self.asst = assist
        self.running = True
        self.padding = 0
        self.prevblock = self.buffer = np.zeros((0,1))
        self.fileready = False
        print("\033[96mLoading Whisper Model..\033[0m", end='', flush=True)
        self.model = whisper.load_model(f'{model_size}{".en" if English else ""}')
        print("\033[90m Done.\033[0m")

    def callback(self, indata, frames, time, status):

        if not any(indata):
            print('\033[31m.\033[0m', end='', flush=True) # if no input, prints red dots
            return
        freq = np.argmax(np.abs(np.fft.rfft(indata[:, 0]))) * SampleRate / frames
        if np.sqrt(np.mean(indata**2)) > Threshold and Vocals[0] <= freq <= Vocals[1] and not self.asst.talking:
            print('.', end='', flush=True)
            if self.padding < 1: self.buffer = self.prevblock.copy()
            self.buffer = np.concatenate((self.buffer, indata))
            self.padding = EndBlocks
        else:
            self.padding -= 1
            if self.padding > 1:
                self.buffer = np.concatenate((self.buffer, indata))
            elif self.padding < 1 < self.buffer.shape[0] > SampleRate: # if enough silence has passed, write to file.
                self.fileready = True
                write('dictate.wav', SampleRate, self.buffer) 
                self.buffer = np.zeros((0,1))
            elif self.padding < 1 < self.buffer.shape[0] < SampleRate: # if recording not long enough, reset buffer.
                self.buffer = np.zeros((0,1))
                print("\033[2K\033[0G", end='', flush=True)
            else:
                self.prevblock = indata.copy() 

    def process(self):
        if self.fileready:
            print("\n\033[90mTranscribing..\033[0m")
            result = self.model.transcribe('dictate.wav',fp16=False,language='en' if English else '',task='translate' if Translate else 'transcribe')
            content = result['text']
            print(content)
            messages.append({"role": "user", "content": content})
            start_time = time.time()
            completion = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=messages,
                temperature=1,
                max_tokens=256,
                top_p=1,
                frequency_penalty=0.5,
                presence_penalty=1.5
            )
            chat_response = completion.choices[0].message['content']
            print(f'ChatGPT: {chat_response}')
            tts = gtts.gTTS(chat_response)
            tts.save("tts.mp3")
            #play the audio file
            self.asst.talking = True
            playsound("tts.mp3")
            self.asst.talking = False
            latency = time.time() - start_time
            print(latency)
            messages.append({"role": "assistant", "content": chat_response})
            
            self.fileready = False

    def listen(self):
        print("\033[32mListening.. \033[37m(Ctrl+C to Quit)\033[0m")
        with sd.InputStream(channels=1, callback=self.callback, blocksize=int(SampleRate * BlockSize / 1000), samplerate=SampleRate):
            while self.running and self.asst.running: self.process()

def main():
    try:
        handler = StreamHandler()
        handler.listen()
    except (KeyboardInterrupt, SystemExit): pass
    finally:
        print("\n\033[93mQuitting..\033[0m")
        if os.path.exists('dictate.wav'): os.remove('dictate.wav')
        if os.path.exists('tts.mp3'): os.remove('tts.mp3')

if __name__ == '__main__':
    main()  