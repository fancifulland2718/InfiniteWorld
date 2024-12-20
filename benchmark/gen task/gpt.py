# 大模型口接口
import base64
import requests

def gpt4(prompt_system, prompt):   # LLM
    # OpenAI API Key
    api_key = ""

    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }
    payload = {
        "model": "gpt-4o",
        "messages": [
            {
                "role": "system",
                "content": prompt_system
            },
            {
                "role": "user",
                "content": prompt
            }
        ]
    }
    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)

    output = response.json()
    print(output["usage"])
    print(output["choices"][0]['message'])
    return output["choices"][0]['message']["content"]

def gpt3_5(prompt_system, prompt):
    # OpenAI API Key
    api_key = ""

    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }
    payload = {
        "model": "gpt-4o-mini",
        "messages": [
            {
                "role": "system",
                "content": prompt_system
            },
            {
                "role": "user",
                "content": prompt
            }
        ]
    }
    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)

    output = response.json()
    print(output["usage"])
    print(output["choices"][0]['message'])
    return output["choices"][0]['message']["content"]