import openai
import os
import urllib.request

openai.api_key = "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"  # supply your API key however you choose

# Let's ask for 3 images of a mathematician that writes on a blackboard
 
response = openai.Image.create(
            prompt = 'A mathematician is writing equations on the blackboard',
            n=3,
            response_format='url',
            size = '512x512')
 
 
# save the images locally
 
if "data" in response:
    for key, obj in enumerate(response["data"]):
        filename ='my_image_'+str(key)+".jpg"
        urllib.request.urlretrieve(obj['url'], filename)
    print('Images have been downloaded and saved locally')
else:
    print("Failed to generate image")