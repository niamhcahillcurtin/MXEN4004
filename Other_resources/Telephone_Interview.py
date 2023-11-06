import openai
openai.api_key = #insert your api key 

messages =  [
        {"role": "system", "content": "You are a Doctor performing a cognitive assessment of an elderly person with possible dementia. You use a tone that is friendly. You're name is MiRo. You immediately start performing the telephone interview for cognitive status"},
        
    ]

while True:
    content = input("User: ")
    messages.append({"role": "user", "content": content})

    completion = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=messages
    )

    chat_response = completion.choices[0].message['content']
    print(f'ChatGPT: {chat_response}')
    messages.append({"role": "assistant", "content": chat_response})

