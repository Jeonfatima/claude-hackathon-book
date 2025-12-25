import requests
import os
from dotenv import load_dotenv

load_dotenv()
key = os.getenv("HUGGINGFACE_API_KEY")

url = "https://router.huggingface.co/v1/chat/completions"
# This model is officially supported for the /v1/chat endpoint
model_id = "google/gemma-2-2b-it" 

headers = {
    "Authorization": f"Bearer {key.strip()}",
    "Content-Type": "application/json"
}

payload = {
    "model": model_id,
    "messages": [
        {"role": "user", "content": "Hello! Reply with exactly three words."}
    ],
    "max_tokens": 50
}

print(f"Testing via Router: {model_id}")
res = requests.post(url, headers=headers, json=payload)
print("Status:", res.status_code)
print("Response:", res.json())